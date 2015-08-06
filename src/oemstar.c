////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief 
////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

#include <sys/attribs.h>
#include "oemstar.h"
#include "fmucomm.h"
#include "uart.h"
#include "coretime.h"

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

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

void OEMStarTask()
{
    // Forward received Ethernet messages (i.e. Commands) to the GPS receiver.
    OEMStarCmdFwd();
    
    // Forward received GPS receiver messages (i.e. Responses and Logs) to the
    // Ethernet.
    OEMStarRspFwd();
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************

// Note: Only a single message can be transmitted at a time; that is, if an
// additional GPS command is received while a previous one is being forwarded
// over UART, the newly received GPS command will be ignored and the TX overflow
// flag will be set.
//
static void OEMStarCmdFwd( void )
{
    static uint8_t       uart_tx_data[ 1024 ];
    static UART_TX_BUF_S uart_tx_buf = { &uart_tx_data[ 0 ] , 0, true };
    
    bool setSuccess;
    
    const FMUCOMM_HOST_PKT* gps_cmd_pkt_p;
    
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
                    &gps_cmd_pkt_p->pl_p[ 0 ],
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

static void OEMStarRspFwd( void )
{
    struct
    {
        uint8_t  data[ 1024 ];
        uint16_t data_len;
    } static OEMStarRxBuf = { {0}, 0 }; // Start the buffer as empty.
    
    static uint32_t rx_time_us;
    
    const UART_RX_BUF_S* uartRxBuf;
          bool           setSuccess;
    
    // Get UART buffered data.
    uartRxBuf = UARTGet();
    
    // UART data received ?
    if( uartRxBuf->data_len != 0 )
    {
        // Record time data is received.
        rx_time_us = CoreTime32usGet();
        
        // Module buffer contains available space for UART data ?
        if( ( 1024 - OEMStarRxBuf.data_len ) >= uartRxBuf->data_len )
        {
            // Copy UART data into module buffer.
            memcpy( &OEMStarRxBuf.data[ OEMStarRxBuf.data_len ],
                    &uartRxBuf->data[ 0 ],
                    uartRxBuf->data_len );
            
            OEMStarRxBuf.data_len += uartRxBuf->data_len;
        }
        else
        {
            // Module receiver buffer has overflow.  Identify loss of received
            // data.
            oemstar_rx_overflow_latch = true;
        }
    }
    
    // Module buffer contains data ?
    if( OEMStarRxBuf.data_len != 0 )
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
        // buffer is 3/4 full.  This is performed so than internal storage does
        // not overflow.  Segmenting of responses to Ethernet packets is 
        // unlikely in this case.
        //
        if( ( CoreTime32usGet() - rx_time_us > 1000 ) ||
            ( OEMStarRxBuf.data_len          > 768  ) )
        {
            // Queue Ethernet data for transmission.
            setSuccess = FMUCommSet( FMUCOMM_TYPE_GPS_DATA, 
                                     &OEMStarRxBuf.data[ 0 ], 
                                     OEMStarRxBuf.data_len );
            
            // Ethernet data queued successfully ?
            if( setSuccess == true )
            {
                // Clear the module buffer length to reset buffer.
                OEMStarRxBuf.data_len = 0;
            }
        }
    }
}