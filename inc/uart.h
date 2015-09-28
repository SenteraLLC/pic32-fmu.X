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

// Enumeration of supported UART hardware modules.
typedef enum
{
    UART_MODULE_1,
    UART_MODULE_2,       
    UART_MODULE_MAX,
            
}UART_MODULE_E;

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

////////////////////////////////////////////////////////////////////////////////
/// @brief  UART Initialization.
///
/// This function initializes UART hardware.
////////////////////////////////////////////////////////////////////////////////
void UARTInit( void );

////////////////////////////////////////////////////////////////////////////////
/// @brief  UART Startup.
///
/// This function perform UART startup tasks.  Alternate control flow supplied 
/// for turning on the UART communication to manage software receiver buffer 
/// overflow. If a large amount of time exists between enabling the module and 
/// the first execution of 'Task', then the receiver hardware buffer could 
/// overflow and received data be lost.
////////////////////////////////////////////////////////////////////////////////
void UARTStartup( void );

////////////////////////////////////////////////////////////////////////////////
/// @brief  UART periodic task.
///
/// @note   This function must occur at a lower priority that the receiver ISR.
///
/// This function  manages receiver buffer for receiving of new data.
////////////////////////////////////////////////////////////////////////////////
void UARTTask( void );

////////////////////////////////////////////////////////////////////////////////
/// @brief  Get received UART data.
///
/// @param  mod_sel
///             The UART hardware module for which data is requested.
///
/// This function return a pointer to the freshest UART received data.
////////////////////////////////////////////////////////////////////////////////
const UART_RX_BUF_S* UARTGet( UART_MODULE_E mod_sel );

// Queue UART data for transmission.

////////////////////////////////////////////////////////////////////////////////
/// @brief  Queue UART data for transmission.
///
/// @param  tx_buf_p
///             Pointer to UART control and communication data to transmit.
///
/// @return Status of queue process.
///             false - Queue unsuccessful.
///
/// This function queues the supplied data to transmission if the supplied
/// data is valid, and their is space available in module data.
////////////////////////////////////////////////////////////////////////////////
bool UARTSet( UART_TX_BUF_S* tx_buf_p );

#endif	// UART_H_