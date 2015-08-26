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

// Constant(s) and Function-like Marco(s) for managing the transmitter FIFO.
// - LEN_MASK: Bit-mask for the size of the FIFO.
// - LEN:      Length of FIFO - must be power of '2'.
// - NEXT_IDX: Next index value in buffer, loop back to start at end of FIFO.
//
#define  UART_TX_FIFO_LEN_MASK     0x03
#define  UART_TX_FIFO_LEN          ( UART_TX_FIFO_LEN_MASK + 1 )
#define mUART_TX_FIFO_NEXT_IDX(x)  ( ( (x) + 1 ) & UART_TX_FIFO_LEN_MASK )

// Constant(s) and Function-like Marco(s) for managing the receiver circular
// buffer.
// - LEN_MASK: Bit-mask for the size of the circular buffer.
// - LEN:      Length of circular buffer - must be power of '2'.
// - NEXT_IDX: Next index value in buffer, loop back to start at end of circular
//             buffer.
// - PREV_IDX: Previous index value in buffer, loop back to end at start of
//             circular buffer.
//
#define  UART_RX_CB_LEN_MASK       0x03
#define  UART_RX_CB_LEN            ( UART_RX_CB_LEN_MASK + 1 )
#define mUART_RX_CB_NEXT_IDX(x)    ( ( (x) + 1 ) & UART_RX_CB_LEN_MASK )
#define mUART_RX_CB_PREV_IDX(x)    ( ( (x) + ( UART_RX_CB_LEN - 1 ) ) & UART_RX_CB_LEN_MASK )

// Structure of FIFO for transmitting data.
typedef struct
{
    UART_TX_BUF_S* buf_arr[ UART_TX_FIFO_LEN ]; // Array of transmission buffer pointers.
    uint8_t        head_idx;                    // Index of data to output.
    uint8_t        tail_idx;                    // Index of data to input - i.e. 1st empty slot.
    
} UART_TX_FIFO_S;

// Structure of circular buffer for receiving data.
typedef struct
{
    UART_RX_BUF_S  buf_arr[ UART_RX_CB_LEN ];   // Array of reception buffers.
    uint8_t        buf_idx;                     // Current array index for receiving data.
    
} UART_RX_CB_S;

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

// UART hardware receiver overflow latch.
static bool uart_oerr_latch = false;

// Number of UART receiver hardware errors (parity or framing) since reset.
static uint16_t uart_err_cnt = 0;

// Circular buffer for receiving UART data.
static UART_RX_CB_S uart_rx_cb =
{
    {
        { { 0 }, 0 },   // Start buffer as empty.
        { { 0 }, 0 },   // Start buffer as empty.
        { { 0 }, 0 },   // Start buffer as empty.
        { { 0 }, 0 },   // Start buffer as empty.
    },
    
    0,                  // Buffer index to start of circular buffer.
};

// FIFO for transmitting UART data.
//
// NOTE: FIFO is maintained using a "keep one slot empty" method.  This method
// is used for being able to identify an empty buffer (i.e. head equals tail)
// and a full buffer (i.e. tail + 1 equals head).
//
// Note: A FIFO is maintained to accommodate the slower transmission rate of
// the UART interface.  Multiple transmission buffers can be queue for
// transmission by the application software, allowing for maximum utilization
// of the UART interface and non-blocking of the application software.
//
static UART_TX_FIFO_S uart_tx_fifo =
{
    { NULL, NULL, NULL, NULL }, // To transmitter buffer pointers to start.
    0,                          // Head index to start of FIFO.
    0,                          // Tail index to start of FIFO.
};

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

static void UARTBufRx( void );
static void UARTBufTX( void );

static uint8_t UARTRead( uint8_t* data_p, uint16_t data_len );
static uint8_t UARTWrite( const uint8_t* data_p, uint16_t data_len );

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

void UARTInit( void )
{
    U1MODEbits.ON = 0;          // Turn module off.
    
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
    
    // Set U1Tx and U1Rx interrupts to priority '1' and sub-priority to '0'.
    IPC6bits.U1IP = 1;
    IPC6bits.U1IS = 0;
    
    // Clear the U1Tx and U1Rx interrupt flags.
    IFS0bits.U1RXIF = 0;
    IFS0bits.U1TXIF = 0;
    
    // Enable the receive interrupt
    IEC0bits.U1RXIE = 1;
}

void UARTStartup( void )
{
    // Turn module on.
    U1MODESET = _U1MODE_ON_MASK;
}

void UARTTask( void )
{
    uint8_t cb_next_idx;
    
    // Flush any remaining data into the reception buffer before updating
    // the circular buffer.  Trigger the receiver interrupt to read any
    // remaining UART data.
    IFS0SET = _IFS0_U1RXIF_MASK;
    
    cb_next_idx = mUART_RX_CB_NEXT_IDX( uart_rx_cb.buf_idx );
    
    // Setup next circular buffer index for receiving data.
    //
    // Note: data length of next element reset before updating buffer index
    // to prevent race condition of ISR updating buffer data between two
    // buffer management operations (i.e. variable serves as semaphore for
    // multi-threaded data).
    //
    // Note: 'uart_rx_cb.buf_idx' only modified by this function and update
    // is atomic.
    //
    uart_rx_cb.buf_arr[ cb_next_idx ].data_len = 0;
    uart_rx_cb.buf_idx = cb_next_idx;
}

const UART_RX_BUF_S* UARTGet( void )
{
    uint8_t prev_idx;
    
    // Get the previous index from the receiver circular buffer.  The current
    // index is used for receiving data while the previous index is the 
    // freshest data available for processing.
    prev_idx = mUART_RX_CB_PREV_IDX( uart_rx_cb.buf_idx );
    
    return ( &uart_rx_cb.buf_arr[ prev_idx ] );
}

bool UARTSet( UART_TX_BUF_S* tx_buf_p )
{
    bool success = false;
    
    // Supplied buffer is valid ?
    if( tx_buf_p != NULL )
    {
        // FIFO is not already full ?
        if( mUART_TX_FIFO_NEXT_IDX( uart_tx_fifo.tail_idx ) != uart_tx_fifo.head_idx )
        {
            success = true;
            
            // Queue the buffer for transmission.
            uart_tx_fifo.buf_arr[ uart_tx_fifo.tail_idx ] = tx_buf_p;
            
            // Identify buffer transmission as incomplete.
            uart_tx_fifo.buf_arr[ uart_tx_fifo.tail_idx ]->tx_done = false;
            
            // Update FIFO control variable.
            //
            // Note: Tail index update is performed last in buffer management
            // operation to eliminate race conditions. (i.e. variable serves
            // as semaphore for multi-threaded buffer).
            //
            // Note: Tail index only modified by this function and update
            // is atomic.
            //
            uart_tx_fifo.tail_idx = mUART_TX_FIFO_NEXT_IDX( uart_tx_fifo.tail_idx );
            
            // Enable the transmit interrupt to send the buffered data.
            IEC0SET = _IEC0_U1TXIE_MASK;
        }
    }
        
    return success;
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************

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

// Service the receiver interrupt.  The receiver hardware buffer content is
// copied into module data.
static void UARTBufRx( void )
{
    UART_RX_BUF_S* rx_buf_elem;
    uint8_t        read_cnt;

    // Perform intermediate access of current receiver buffer element used 
    // for storing data to increase processing speed and reduce line length.
    rx_buf_elem = &uart_rx_cb.buf_arr[ uart_rx_cb.buf_idx ];
    
    // Receive available data.
    read_cnt = UARTRead( &rx_buf_elem->data[ rx_buf_elem->data_len ],
                         ( UART_RX_BUF_DATA_LEN - rx_buf_elem->data_len ) );

    // Manage buffer control variables.
    rx_buf_elem->data_len += read_cnt;
    
    // Clear the interrupt flag.
    //
    // Note: this must be performed after the condition which caused the 
    // interrupt (i.e. received data) is addressed.
    //
    IFS0CLR = _IFS0_U1RXIF_MASK;
}

// Read data from hardware buffer into supplied buffer. 
//
// Note: It's assumed that higher-level integrity checks are included within
// the serial data (e.g. checksum, CRC, etc.) for verifying content.  Low-level
// communication failures (e.g. parity-bit failure) are internally recorded
// to provide visibility during development.
//
// uint8_t* data_p - pointer to buffer for storing received data.
//
// uint8_t data_len - number of bytes available in data_p for storing received
// data
//
// Note: the function will stored data within data_p until data_p is full.
// When data_p is full, receiver hardware must still be read to service hardware
// operation.  Read hardware data when data_p is full is lost.
//
static uint8_t UARTRead( uint8_t* data_p, uint16_t data_len )
{
    uint8_t data_cnt = 0;
    uint8_t rx_byte;
    
    // Hardware buffer overflow has occurred ?
    if( U1STAbits.OERR == 1 )
    {
        // Latch identification of overflow error.
        uart_oerr_latch = true;
        
        // Clear the hardware overflow hardware flag.
        //
        // Note: The overflow flag needs to be cleared because all UART
        // reception is inhibited while this hardware flag is set.
        //
        U1STACLR = _U1STA_OERR_MASK;
    }
    
    // Read data from the hardware buffer until it is empty.
    while( U1STAbits.URXDA == 1 )
    {
        // Parity or framing error with received byte ?
        if( ( U1STAbits.PERR == 1 ) ||
            ( U1STAbits.FERR == 1 ) )
        {
            uart_err_cnt++;
        }
        
        // Read byte from hardware buffer.
        rx_byte = (uint8_t) U1RXREG;
        
        // Supplied buffer is valid and not full ?
        //
        // Note: byte is still read from hardware but not saved if supplied
        // buffer is invalid so that hardware buffer is still emptied and newer
        // data can be received (i.e hardware buffer will not overflow).
        //
        if( ( data_p   != NULL ) &&
            ( data_len != 0    ) )
        {
            // Store data into supplied buffer.
            data_p[ data_cnt ] = rx_byte;
            data_cnt++;
            data_len--;
        }
    }
    
    return data_cnt;
}

// Service the transmitter interrupt.  Write next chunk of data to be
// transmitted to the transmitter hardware buffer.
static void UARTBufTX( void )
{
    UART_TX_BUF_S* tx_buf_elem;
    uint8_t        write_cnt;
    
    // FIFO contains data ?
    if( uart_tx_fifo.head_idx != uart_tx_fifo.tail_idx )
    {
        // Perform intermediate access of current transmitter buffer element 
        // used  for writing data to increase processing speed and reduce line
        // length.
        tx_buf_elem = uart_tx_fifo.buf_arr[ uart_tx_fifo.head_idx ];
        
        // Transmit available data.
        write_cnt = UARTWrite( tx_buf_elem->data_p,
                               tx_buf_elem->data_len );
        
        // Manage buffer control variables.
        tx_buf_elem->data_p   += write_cnt;
        tx_buf_elem->data_len -= write_cnt;
        
        // All buffer data has been transmitted ?
        if( tx_buf_elem->data_len == 0 )
        {
            // Identify buffer transmission as complete.
            tx_buf_elem->tx_done = true;
            
            // Move FIFO head to next index.
            //
            // Note: Head index update is performed last in buffer management
            // operation to eliminate race conditions. (i.e. variable serves
            // as semaphore for multi-threaded buffer).
            //
            // Note: Head index only modified by this function and update
            // is atomic.
            //
            uart_tx_fifo.head_idx = mUART_TX_FIFO_NEXT_IDX( uart_tx_fifo.head_idx );
        }
    }
    
    // FIFO is empty ?
    if( uart_tx_fifo.head_idx == uart_tx_fifo.tail_idx )
    {
        // Disable transmitter interrupt since no additional data to send.
        IEC0CLR = _IEC0_U1TXIE_MASK;
    }
    
    // Clear the interrupt flag.
    //
    // Note: this must be performed after the condition which caused the 
    // interrupt (i.e. transmitter empty) is addressed.
    //
    IFS0CLR = _IFS0_U1TXIF_MASK;
}

// Write supplied data to the hardware transmit buffer.
static uint8_t UARTWrite( const uint8_t* data_p, uint16_t data_len )
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