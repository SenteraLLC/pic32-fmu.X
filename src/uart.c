////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Universal Asynchronous Receiver Transmitter (UART) driver.
////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

#include "uart.h"
#include <sys/attribs.h>

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

#define  UART_TX_FIFO_LEN_MASK     0x03
#define  UART_TX_FIFO_LEN          ( UART_TX_FIFO_LEN_MASK + 1 )
#define mUART_TX_FIFO_NEXT_IDX(x)  ( ( (x) + 1 ) & UART_TX_FIFO_LEN_MASK )

#define  UART_RX_CB_LEN_MASK       0x03
#define  UART_RX_CB_LEN            ( UART_RX_CB_LEN_MASK + 1 )
#define mUART_RX_CB_NEXT_IDX(x)    ( ( (x) + 1 ) & UART_RX_CB_LEN_MASK )
#define mUART_RX_CB_PREV_IDX(x)    ( ( (x) + ( UART_RX_CB_LEN - 1 ) ) & UART_RX_CB_LEN_MASK )



typedef struct
{
    UART_TX_BUF_S* buf[ UART_TX_FIFO_LEN ];
    uint8_t        head_idx; // Index of data to output.
    uint8_t        tail_idx; // Index of data to input - i.e. empty slot.
    
} UART_TX_FIFO_S;

// NOTE: FIFO is maintained using a "keep one slot empty" method. 
static UART_TX_FIFO_S tx_fifo =
{
    { NULL, NULL, NULL, NULL },
    0,
    0,
};





typedef struct
{
    UART_RX_BUF_S  cb[ UART_RX_CB_LEN ];
    uint8_t        cb_idx;
    
} UART_RX_CB_S;

static UART_RX_CB_S rx_cb =
{
    {
        { { 0 }, 0 },
        { { 0 }, 0 },
        { { 0 }, 0 },
        { { 0 }, 0 },
    },
    
    0,
};

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

static bool     uart_oerr_latch = false;
static uint16_t uart_err_cnt    = 0;

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

void UARTBufRx( void );
void UARTBufTX( void );

uint8_t UARTRead( uint8_t* data_p, uint8_t data_len );
uint8_t UARTWrite( const uint8_t* data_p, uint8_t data_len );

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

void UARTInit( void )
{
    U1MODEbits.ON = 0;          // Turn module off .
    
    U1MODEbits.UEN   = 0b00;    // Don't use U1CTS and U1RTS pins.
    U1MODEbits.PDSEL = 0b00;    // 8-bit data, no parity.
    U1MODEbits.STSEL = 0;       // 1 stop bit.
    
    U1STAbits.UTXISEL = 0b10;   // Interrupt generated while transmit buffer empty.
    U1STAbits.URXEN   = 1;      // Received enabled.
    U1STAbits.UTXEN   = 1;      // Transmitter enabled.
    U1STAbits.URXISEL = 0b10;   // Interrupt generated while receiver 3/4 full.
    
    // Baud Rate = Fpb   / ( 16 * ( U1BRG + 1 ) )
    //           = 80MHZ / ( 16 * ( 42    + 1 ) )
    //           = 116,279
    //
    // Error %   = ( 116279 - 115200 ) / 115200
    //           = 0.94 %
    //
    U1BRG = 42;
    
    // Enable the receive interrupt
    IEC0bits.U1RXIE = 1;
    
    // Set U1Tx and U1RX interrupts to priority '1'.
    IPC6bits.U1IP = 1;  
}

/// @note Alternate control flow from 'Init' is supplied for turning on the
/// UART communication to manage software receiver buffer overflow. If a large
/// amount of time exists between enabling the module and the first execution
/// of 'Task', then the receiver hardware buffer will likely overflow and
/// received data will be lost.
void UARTStartup( void )
{
    // Turn module on.
    U1MODEbits.ON = 0;
}

/// @brief Manage receiver buffer for receiving of new data.
///
/// @note  This function must occur at a lower priority that the ISR ???
///
/// @note  Data over UART is assumed to be a continuous stream; therefore,
/// the hardware buffer is never flush as it's assumed the 3/4 full interrupt
/// will occur.  That is, it's assumed that unread data will not sit in
/// the receiver hardware buffer (e.g. 1/2 full of data) for an extended
/// period of time.
void UARTTask( void )
{
    uint8_t cb_next_idx;
    
    // Setup next circular buffer index for receiving data.
    //
    // Note: 'rx_cb.cb_idx' only modified by this function and update
    // is atomic; therefore, no race condition exists.
    //
    cb_next_idx = mUART_RX_CB_NEXT_IDX( rx_cb.cb_idx );
    
    rx_cb.cb[ cb_next_idx ].data_len = 0;
    rx_cb.cb_idx = cb_next_idx;
}

void __ISR ( _UART_1_VECTOR, IPL1SOFT) UART1ISR( void ) 
{
    // Receiver caused interrupt ?
    if( IFS0bits.U1RXIF == 1 )
    {
        // Service the receiver.
        UARTBufRx();
    }
    
    // Transmitter caused interrupt ?
    if( IFS0bits.U1TXIF == 1 )
    {
        // Service the transmitter.
        UARTBufTX();
    }
}

/// @brief Return pointer to freshest received data available for processing.
const UART_RX_BUF_S* UARTGet( void )
{
    uint8_t rx_cb_get_idx;
    
    // Get the previous index from the receiver circular buffer.  The current
    // index is used for receiving data while the previous index is the 
    // freshest data available for processing.
    rx_cb_get_idx = mUART_RX_CB_PREV_IDX( rx_cb.cb_idx );
    
    return ( &rx_cb.cb[ rx_cb_get_idx ] );
}

/// @brief Setup a buffer for transmission if space available.
bool UARTSet( UART_TX_BUF_S* tx_buf_p )
{
    bool success = false;
    
    // Supplied buffer is valid ?
    if( tx_buf_p != NULL )
    {
        // FIFO is not already full ?
        if( mUART_TX_FIFO_NEXT_IDX( tx_fifo.tail_idx ) != tx_fifo.head_idx )
        {
            success = true;
            
            // Queue the buffer for transmission.
            tx_fifo.buf[ tx_fifo.tail_idx ] = tx_buf_p;
            
            // Identify buffer transmission as incomplete.
            tx_fifo.buf[ tx_fifo.tail_idx ]->tx_done = false;
            
            // Update FIFO control variable.
            //
            // Note: 'tail_idx' only modified by this function and update
            // is atomic; therefore, no race condition exists.
            //
            tx_fifo.tail_idx = mUART_TX_FIFO_NEXT_IDX( tx_fifo.tail_idx );
            
            // Enable the transmit interrupt to send the buffered data.
            IEC0bits.U1TXIE = 1;
        }
    }
        
    return success;
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************

void UARTBufRx( void )
{
    uint8_t read_cnt;
    
    // Buffer is not already full ?
    if( rx_cb.cb[ rx_cb.cb_idx ].data_len < 128 )
    {
        // Receive available data.
        read_cnt = UARTRead( &rx_cb.cb[ rx_cb.cb_idx ].data_p[ rx_cb.cb[ rx_cb.cb_idx ].data_len ],
                             128 - rx_cb.cb[ rx_cb.cb_idx ].data_len );
        
        // Manage buffer control variables.
        rx_cb.cb[ rx_cb.cb_idx ].data_len += read_cnt;
    }
    
    // Clear the interrupt flag.
    //
    // Note: this must be performed after the condition which caused the 
    // interrupt (i.e. received data) is addressed.
    //
    IFS0bits.U1RXIF = 0;
}

/// @brief Read data from hardware buffer into supplied buffer. 
///
/// Serial communication with the GPS is verified for integrity using a CRC.
/// Therefore lower-level communication failures (e.g. parity-bit failure) are
/// not checked as this yields additional software complexity with essentially
/// identical failure detection capability.
uint8_t UARTRead( uint8_t* data_p, uint8_t data_len )
{
    uint8_t data_cnt = 0;
    uint8_t rx_byte;
    
    // Hardware buffer overflow has occurred ?
    if( U1STAbits.OERR == 1 )
    {
        // Latch identification of overflow error.
        uart_oerr_latch = true;
        
        // Clear the hardware overflow flag.
        //
        // Note: The overflow flag needs to be cleared because all UART
        // reception is inhibited while this hardware flag is set.
        //
        U1STAbits.OERR = 0;
    }
    
    // Read data from the hardware buffer until it is empty, or the receive
    // software buffer is full
    while( ( U1STAbits.URXDA == 1 ) &&
           ( data_len        != 0 ) )
    {
        // Read byte from hardware buffer.
        rx_byte = (uint8_t) U1RXREG;
        
        // Supplied buffer is valid ?
        //
        // Note: byte is still read from hardware but not saved if supplied
        // buffer is invalid so that hardware buffer is still emptied and newer
        // data can be received (i.e hardware buffer will not overflow).
        //
        if( data_p != NULL )
        {
            // Parity or framing error with received byte ?
            if( ( U1STAbits.PERR == 1 ) ||
                ( U1STAbits.FERR == 1 ) )
            {
                uart_err_cnt++;
            }
            
            // Store data into supplied buffer.
            data_p[ data_cnt ] = rx_byte;
            data_cnt++;
        }
    }
    
    return data_cnt;
}

/// @brief Write next chunk of data to be transmitted.
///
/// @note  Executed in higher-priority thread than 'set'.
void UARTBufTX( void )
{
    uint8_t write_cnt;
    
    // FIFO contains data ?
    if( tx_fifo.head_idx != tx_fifo.tail_idx )
    {
        // Transmit available data.
        write_cnt = UARTWrite( tx_fifo.buf[ tx_fifo.head_idx ]->data_p,
                               tx_fifo.buf[ tx_fifo.head_idx ]->data_len );
        
        // Manage buffer control variables.
        tx_fifo.buf[ tx_fifo.head_idx ]->data_p   += write_cnt;
        tx_fifo.buf[ tx_fifo.head_idx ]->data_len -= write_cnt;
        
        // All buffer data has been transmitted ?
        if( tx_fifo.buf[ tx_fifo.head_idx ]->data_len == 0 )
        {
            // Identify buffer transmission as complete.
            tx_fifo.buf[ tx_fifo.head_idx ]->tx_done = true;
            
            // Move FIFO head to next index.
            //
            // Note: 'head_idx' only modified by this function and update
            // is atomic; therefore, no race condition exists.
            //
            tx_fifo.head_idx = mUART_TX_FIFO_NEXT_IDX( tx_fifo.head_idx );
        }
    }
    
    // FIFO is empty ?
    if( tx_fifo.head_idx == tx_fifo.tail_idx )
    {
        // Disable transmitter interrupt since no additional data to send.
        IEC0bits.U1TXIE = 0;
    }
    
    // Clear the interrupt flag.
    //
    // Note: this must be performed after the condition which caused the 
    // interrupt (i.e. transmitter empty) is addressed.
    //
    IFS0bits.U1TXIF = 0;
}

/// @brief Write supplied data to the hardware transmit buffer.
uint8_t UARTWrite( const uint8_t* data_p, uint8_t data_len )
{
    uint8_t data_cnt = 0;
    
    // Load the hardware buffer while it is not full and there is additional
    // data to send
    while( ( U1STAbits.UTXBF == 0 ) &&
           ( data_len        != 0 ) )
    {
        // Store a byte to the hardware buffer.
        U1TXREG = data_p[ data_cnt ];
        
        // Update control values.
        data_cnt++;
        data_len--;
    }
    
    return data_cnt;
}