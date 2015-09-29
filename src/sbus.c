////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief S.Bus decode module.
////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

#include "sbus.h"
#include "uart.h"
#include "ts.h"

// *****************************************************************************
// ************************** Macros *******************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

/// The number of bytes which an S.Bus payload packet.
#define SBUS_PAYLOAD_LEN 23

#define SBUS_HEADER_VALUE 0x0F      ///< Value of S.Bus Header byte.
#define SBUS_FOOTER_VALUE 0x00      ///< Value of S.Bus Footer byte.

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

static uint16_t sbus_ch_data[ SBUS_CH_MAX ];

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

static void SBusProcess( uint8_t byte_in );

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

void SBusTask( void )
{
    const UART_RX_BUF_S* uartRxBuf_p;
    uint16_t rx_data_byte_idx;
        
    // Get UART buffered data.
    uartRxBuf_p = UARTGet( UART_MODULE_2 );

    // Process each byte of the received data.
    for( rx_data_byte_idx = 0;
         rx_data_byte_idx < uartRxBuf_p->data_len;
         rx_data_byte_idx++ )
    {
        // Execute the state machine with the received data.
        SBusProcess( uartRxBuf_p->data[ rx_data_byte_idx ] );
    }
}

void SBusListGet( uint16_t ch_list[ SBUS_CH_MAX ] )
{
    memcpy( &ch_list[ 0 ], &sbus_ch_data[ 0 ], sizeof( sbus_ch_data ) );
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************

////////////////////////////////////////////////////////////////////////////////
/// @brief  Process a byte of received S.Bus data.
///
/// @param  byte_in
///             The byte of S.Bus data.
///
/// This function implements a state machine for determining the Start/Data/End
/// bytes of the S.Bus data.  Once an entire data payload is received, the 
/// payload is decoded and populate in module data variable \a sbus_ch_data.
////////////////////////////////////////////////////////////////////////////////
static void SBusProcess( uint8_t byte_in )
{
    /// Structure defining the contents of an S.Bus data payload.  Each of the
    /// channel fields (ch1:ch16) occupies 11 bytes.
    typedef union
    {
        uint8_t data_array[ SBUS_PAYLOAD_LEN ];
        
        struct __attribute__ ((packed))
        {
            uint32_t ch1          : 11;
            uint32_t ch2          : 11;
            uint32_t ch3_lo       : 10;
                 
            uint32_t ch3_hi       :  1;
            uint32_t ch4          : 11;
            uint32_t ch5          : 11;
            uint32_t ch6_lo       :  9;
                 
            uint32_t ch6_hi       :  2;
            uint32_t ch7          : 11;
            uint32_t ch8          : 11;
            uint32_t ch9_lo       :  8;
                 
            uint32_t ch9_hi       :  3;
            uint32_t ch10         : 11;
            uint32_t ch11         : 11;
            uint32_t ch12_lo      :  7;
                 
            uint32_t ch12_hi      :  4;
            uint32_t ch13         : 11;
            uint32_t ch14         : 11;
            uint32_t ch15_lo      :  6;
                 
            uint32_t ch15_hi      :  5;
            uint32_t ch16         : 11;
            uint32_t ch17         :  1; // digital channel
            uint32_t ch18         :  1; // digital channel
            uint32_t frame_lost   :  1;
            uint32_t failsafe_act :  1;
        };
        
    } SBUS_DATA_U;
    
    static enum
    {
        SM_END,
        SM_START,
        SM_DATA,         
                
    } sm_state = SM_END;
    
    static SBUS_DATA_U sbus_data;
    
    // Note: The state machine is started by checking that the end/footer byte
    // is received followed by the start/header byte.  This checking of two
    // consecutive bytes provides a better detection of the start of a data
    // packet - i.e. rather than only checking the start/header byte.
    switch( sm_state )
    {
        case SM_END:
        {
            // End byte received ?
            if( byte_in == SBUS_FOOTER_VALUE )
            {
                sm_state++;
            }
            
            break;
        }
        case SM_START:
        {
            // Start byte received ?
            if( byte_in == SBUS_HEADER_VALUE )
            {
                sm_state++;
            }
            else
            {
                // Start byte needs to be next received byte following the
                // end byte of the previous message.
                sm_state = SM_END;
            }
            
            break;
        }
        case SM_DATA:
        {
            static uint8_t ch_data_idx = 0;
            
            sbus_data.data_array[ ch_data_idx ] = byte_in;
            
            ch_data_idx++;
            if( ch_data_idx >= SBUS_PAYLOAD_LEN )
            {
                // Reset for reception of next data stream.
                ch_data_idx = 0;    
                sm_state    = SM_END;
                        
                // Process the received data.
                sbus_ch_data[  0 ] = sbus_data.ch1;
                sbus_ch_data[  1 ] = sbus_data.ch2;
                sbus_ch_data[  2 ] = ( sbus_data.ch3_hi << 10 ) | sbus_data.ch3_lo;
                sbus_ch_data[  3 ] = sbus_data.ch4;
                sbus_ch_data[  4 ] = sbus_data.ch5;
                sbus_ch_data[  5 ] = ( sbus_data.ch6_hi <<  9 ) | sbus_data.ch6_lo;
                sbus_ch_data[  6 ] = sbus_data.ch7;
                sbus_ch_data[  7 ] = sbus_data.ch8;
                sbus_ch_data[  8 ] = ( sbus_data.ch9_hi <<  8 ) | sbus_data.ch9_lo;
                sbus_ch_data[  9 ] = sbus_data.ch10;
                sbus_ch_data[ 10 ] = sbus_data.ch11;
                sbus_ch_data[ 11 ] = ( sbus_data.ch12_hi << 7 ) | sbus_data.ch12_lo;
                sbus_ch_data[ 12 ] = sbus_data.ch13;
                sbus_ch_data[ 13 ] = sbus_data.ch14;
                sbus_ch_data[ 14 ] = ( sbus_data.ch15_hi << 6 ) | sbus_data.ch15_lo;
                sbus_ch_data[ 15 ] = sbus_data.ch16;
            }
            
            break;
        }
    }
}