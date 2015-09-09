////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief NovAtel OEMStar GPS Receiver Application.
////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

#include "oemstar.h"
#include <sys/attribs.h>
#include "fmucomm.h"
#include "uart.h"
#include "coretime.h"

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

// Length of transmitter static buffer.
//
// Note: This value must be larger than the largest message required to be
// transmitted to the OEMStar.
//
#define OEMSTAR_TX_DATA_LEN 2048

// The type of GPS receiver communicating.
//  0 = NovAtel OEMStar
#define OEMSTAR_GPS_TYPE 0

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

static bool oemstar_rx_overflow_latch = false;
static bool oemstar_tx_overflow_latch = false;
static bool oemstar_tx_err_latch      = false;

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

static void OEMStarCmdFwd( void );
static void OEMStarRspFwd( void );

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

void OEMStarTask( void )
{
    // Forward GPS command (HOST -> GPS).
    OEMStarCmdFwd();
    
    // Forward GPS responses and logs (GPS -> HOST).
    OEMStarRspFwd();
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************

// Forward received Ethernet messages (i.e. Commands) to the GPS receiver.
//
// Note: Only a single message can be transmitted at a time; that is, if an
// additional GPS command is received while a previous one is being forwarded
// over UART, the newly received GPS command will be ignored and the TX overflow
// flag will be set.
//
static void OEMStarCmdFwd( void )
{
    static uint8_t       uart_tx_data[ OEMSTAR_TX_DATA_LEN ];
    static UART_TX_BUF_S uart_tx_buf = { &uart_tx_data[ 0 ] , 0, true };
    
    const FMUCOMM_RX_PKT*          gps_cmd_pkt_p;
    const FMUCOMM_HOST_GPS_CMD_PL* gps_cmd_pl_p;
    
    bool setSuccess;
    
    gps_cmd_pkt_p = FMUCommGet( FMUCOMM_TYPE_GPS_CMD );
    
    // GPS command packet received ?
    if( gps_cmd_pkt_p->valid == true )
    {
        // UART already busy ?
        if( uart_tx_buf.tx_done == false )
        {
            // Module transmitter buffer has overflow.  Identify loss of 
            // transmitted data.
            oemstar_tx_overflow_latch = true;
        }
        else
        {
            // Typecast received packet to GPS command type to access packet
            // content.
            gps_cmd_pl_p = (FMUCOMM_HOST_GPS_CMD_PL*) gps_cmd_pkt_p->pl_p;
            
            // Copy in module buffer the data to be transmitted.
            //
            // Note: this is performed rather using the buffered GPS Ethernet
            // data directly as maintaining of Ethernet data is only guaranteed 
            // for a single execution cycle.  Since the transmission rate of
            // UART data is at a slower enough rate to not be completed in a 
            // single software execution cycle, the data must be locally copied
            // to guarantee integrity during transmission.
            //
            memcpy( &uart_tx_buf.data_p[ 0 ],
                    &gps_cmd_pl_p->GPSData[ 0 ],
                    gps_cmd_pkt_p->wrap.length );
            
            // Setup the length of the buffer data.
            uart_tx_buf.data_len = gps_cmd_pkt_p->wrap.length;
            
            // Queue the data for transmission.
            setSuccess = UARTSet( &uart_tx_buf );
            
            // Queuing of data unsuccessful ?
            if( setSuccess == false )
            {
                oemstar_tx_err_latch = true;
            }
        }
    }
}

// Forward received GPS receiver messages (i.e. Responses and Logs) to the
// Ethernet.
static void OEMStarRspFwd( void )
{
    static FMUCOMM_GPS_DATA_PL gps_data_pl;
    static uint16_t            gps_data_len = 0;
    
    static uint32_t rx_time_us = 0;
    
    const UART_RX_BUF_S* uartRxBuf;
          bool           setSuccess;
    
    // Get UART buffered data.
    uartRxBuf = UARTGet( UART_MODULE_1 );
    
    // UART data received ?
    if( uartRxBuf->data_len != 0 )
    {
        // Record time data is received.
        rx_time_us = CoreTime32usGet();
        
        // Module buffer contains available space for UART data ?
        if( ( sizeof( gps_data_pl.gpsData ) - gps_data_len ) >= uartRxBuf->data_len )
        {
            // Copy UART data into module buffer.
            memcpy( &gps_data_pl.gpsData[ gps_data_len ],
                    &uartRxBuf->data[ 0 ],
                    uartRxBuf->data_len );
            
            gps_data_len += uartRxBuf->data_len;
        }
        else
        {
            // Module receiver buffer has overflow.  Identify loss of received
            // data.
            oemstar_rx_overflow_latch = true;
        }
    }
    
    // Module buffer contains data ?
    if( gps_data_len != 0 )
    {
        // No UART data received for 1ms, or module buffer is
        // greater than 3/4 full ?
        //
        // Note: 1ms timeout derived from data transmission rate and hardware
        // buffering configuration.
        //  UART data byte:
        //      - 1 start bit
        //      - 8 data bits
        //      - 1 stop bit
        //  Hw configuration:
        //      - 115200 nominal transfer rate.
        //      - 6 byte deep buffer before servicing.
        // Therefore, with 100% UART receiver utilization, the OEMStar 
        // application module will receive fresh data every:
        //  - (1/115200) * 10 * 6 = 0.52 ms.
        // Additional margin is added to the timeout to treat non-ideal
        // conditions.
        //
        // Note: An Ethernet packet is transmitted when the internal module
        // buffer is 3/4 full.  This is performed so that internal storage does
        // not overflow.  Segmenting of responses to Ethernet packets is 
        // unlikely in this case.
        //
        if( ( CoreTime32usGet() - rx_time_us > 1000 ) ||
            ( gps_data_len                   > 1125 ) )
        {
            // Populate additional GPS data fields.
            gps_data_pl.fmuTime = CoreTime64usGet();
            gps_data_pl.gpsType = OEMSTAR_GPS_TYPE;
            
            // Queue Ethernet data for transmission.
            setSuccess = FMUCommSet( FMUCOMM_TYPE_GPS_DATA, 
                                     (uint8_t*) &gps_data_pl, 
                                      sizeof( gps_data_pl.fmuTime ) + sizeof( gps_data_pl.gpsType ) + gps_data_len );
            
            // Ethernet data queued successfully ?
            if( setSuccess == true )
            {
                // Clear the module buffer length to reset buffer.
                gps_data_len = 0;
            }
        }
    }
}