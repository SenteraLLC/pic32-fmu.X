////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Universal Asynchronous Receiver Transmitter (UART) driver.
////////////////////////////////////////////////////////////////////////////////

#ifndef UART_H_
#define	UART_H_

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

#include <xc.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

#define UART_RX_BUF_DATA_LEN 8 // 1024

// Note: Receiver data buffer size must be scaled to accommodate maximum amount
// of received data for the execution cycle time.
typedef struct
{
    uint8_t  data[ 8 ];     // NOTE: NEED TO VERIFY THAT THIS SIZE IS LARGE ENOUGH TO NEVER OVERFLOW WITH THE OEMSTAR OPERATION.
    uint16_t data_len;
    
} UART_RX_BUF_S;

typedef struct
{
    uint8_t* data_p;
    uint16_t data_len;
    bool     tx_done;
    
} UART_TX_BUF_S;

// *****************************************************************************
// ************************** Declarations *************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

void UARTInit( void );
void UARTStartup( void );
void UARTTask( void );

const UART_RX_BUF_S* UARTGet( void );
bool UARTSet( UART_TX_BUF_S* tx_buf_p );

#endif	// UART_H_