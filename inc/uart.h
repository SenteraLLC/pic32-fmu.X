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

// Length of the UART reception buffer.
//
// Note: Receiver data buffer size must be scaled to accommodate maximum amount
// of received data for the execution cycle time.
//
#define UART_RX_BUF_DATA_LEN 1024

// Definition of buffer for reception of UART data.
typedef struct
{
    uint8_t  data[ UART_RX_BUF_DATA_LEN ];
    uint16_t data_len;
    
} UART_RX_BUF_S;

// Definition of buffer for transmission of UART data.
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

// Initialize the UART hardware.
void UARTInit( void );

// Startup the UART module operation.
//
// Note: Alternate control flow supplied for turning on the
// UART communication to manage software receiver buffer overflow. If a large
// amount of time exists between enabling the module and the first execution
// of 'Task', then the receiver hardware buffer could overflow and
// received data be lost.
//
void UARTStartup( void );

// Manage receiver buffer for receiving of new data.
//
// Note: This function must occur at a lower priority that the receiver ISR.
//
void UARTTask( void );

// Get the freshest UART received data.
const UART_RX_BUF_S* UARTGet( void );

// Queue UART data for transmission.
bool UARTSet( UART_TX_BUF_S* tx_buf_p );

#endif	// UART_H_