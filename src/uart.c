////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Universal Asynchronous Receiver Transmitter (UART) driver.
////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

#include <sys/attribs.h>

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

#include "uart.h"
#include "ts.h"

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
static bool uart_oerr_latch[ UART_MODULE_MAX ] = { false, false };

// Number of UART receiver hardware errors (parity or framing) since reset.
static uint16_t uart_err_cnt[ UART_MODULE_MAX ] = { 0 , 0 };

// Circular buffer for receiving UART data.
static UART_RX_CB_S uart_rx_cb[ UART_MODULE_MAX ] =
{
    // UART_MODULE_1
    {
        {
            { { 0 }, 0 },   // Start buffer as empty.
            { { 0 }, 0 },   // Start buffer as empty.
            { { 0 }, 0 },   // Start buffer as empty.
            { { 0 }, 0 },   // Start buffer as empty.
        },

        0,                  // Buffer index to start of circular buffer.
    },
    
    // UART_MODULE_2
    {
        {
            { { 0 }, 0 },   // Start buffer as empty.
            { { 0 }, 0 },   // Start buffer as empty.
            { { 0 }, 0 },   // Start buffer as empty.
            { { 0 }, 0 },   // Start buffer as empty.
        },

        0,                  // Buffer index to start of circular buffer.
    },
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

static void UARTBufRx( UART_MODULE_E mod_sel );
static void UARTBufTX( void );

static uint8_t UARTRead( UART_MODULE_E mod_sel, uint8_t* data_p, uint16_t data_len );
static uint8_t UARTWrite( const uint8_t* data_p, uint16_t data_len );

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

void UARTInit( void )
{
    //
    // UART1 INITIALIZATION ----------------------------------------------------
    //
    
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
    
    //
    // UART2 INITIALIZATION ----------------------------------------------------
    //
    
    U2MODEbits.ON     = 0;      // Turn module off.
    
    U2MODEbits.UEN    = 0b00;   // Don't use U1CTS and U1RTS pins.
    U2MODEbits.RXINV  = 1;      // Receiver polarity is inverted (idle sate is '0').
    U2MODEbits.PDSEL  = 0b01;   // 8-bit data, even parity.
    U2MODEbits.STSEL  = 1;      // 2 stop bits.
    
    U2STAbits.URXEN   = 1;      // Received enabled.
    U2STAbits.URXISEL = 0b10;   // Interrupt generated while receiver 3/4 full.
    
    // Baud Rate = Fpb   / ( 16 * ( U1BRG + 1 ) )
    //           = 80MHZ / ( 16 * ( 49    + 1 ) )
    //           = 100,000
    //
    // Error %   = 0%
    //
    U2BRG = 49;
    
    // Set U2Rx interrupts to priority '1' and sub-priority to '0'.
    IPC8bits.U2IP = 1;
    IPC8bits.U2IS = 0;
    
    // Clear the U2Rx interrupt flags.
    IFS1bits.U2RXIF = 0;
    
    // Enable the receive interrupt
    IEC1bits.U2RXIE = 1;
}

void UARTStartup( void )
{
    // Turn modules on.
    U1MODESET = _U1MODE_ON_MASK;
    U2MODESET = _U2MODE_ON_MASK;
}

void UARTTask( void )
{
    uint8_t cb1_next_idx;
    uint8_t cb2_next_idx;
    
    // Flush any remaining data into the reception buffersa before updating
    // the circular buffers.  Trigger the receiver interrupts to read any
    // remaining UART data.
    IFS0SET = _IFS0_U1RXIF_MASK;
    IFS1SET = _IFS1_U2RXIF_MASK;
    
    cb1_next_idx = mUART_RX_CB_NEXT_IDX( uart_rx_cb[ UART_MODULE_1 ].buf_idx );
    cb2_next_idx = mUART_RX_CB_NEXT_IDX( uart_rx_cb[ UART_MODULE_2 ].buf_idx );
    
    // Setup next circular buffer indexes for receiving data.
    //
    // Note: data length of next element reset before updating buffer index
    // to prevent race condition of ISR updating buffer data between two
    // buffer management operations (i.e. variable serves as semaphore for
    // multi-threaded data).
    //
    // Note: 'uart_rx_cb.buf_idx' only modified by this function and update
    // is atomic.
    //
    uart_rx_cb[ UART_MODULE_1 ].buf_arr[ cb1_next_idx ].data_len = 0;
    uart_rx_cb[ UART_MODULE_1 ].buf_idx = cb1_next_idx;
    
    uart_rx_cb[ UART_MODULE_2 ].buf_arr[ cb2_next_idx ].data_len = 0;
    uart_rx_cb[ UART_MODULE_2 ].buf_idx = cb2_next_idx;
}

const UART_RX_BUF_S* UARTGet( UART_MODULE_E mod_sel )
{
    uint8_t prev_idx;
    
    // Get the previous index from the receiver circular buffer.  The current
    // index is used for receiving data while the previous index is the 
    // freshest data available for processing.
    prev_idx = mUART_RX_CB_PREV_IDX( uart_rx_cb[ mod_sel ].buf_idx );
    
    return ( &uart_rx_cb[ mod_sel ].buf_arr[ prev_idx ] );
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

////////////////////////////////////////////////////////////////////////////////
/// @brief  Hardware buffer availability SPI1 interrupt.
///
/// This function empties the received hardware buffer and fills the hardware
/// transmit buffer to keep 100% utilization of the UART interface when
/// transferring data.
////////////////////////////////////////////////////////////////////////////////
void __ISR ( _UART_1_VECTOR, IPL1SOFT) UART1ISR( void ) 
{
ts_start(UART1_ISR);
    // Receiver caused interrupt ?
    //
    // Note: Since multiple interrupts can trigger the ISR, both the enabling
    // of the interrupt source, and the status flag must be checked.  That is,
    // it is possible for the status flag to be set (Rx contains data) but the
    // receiver interrupt to be disabled.
    //
    if( ( IEC0bits.U1RXIE == 1 ) &&
        ( IFS0bits.U1RXIF == 1 ) )
    {
        // Service the receiver.
        UARTBufRx( UART_MODULE_1 );
    }
    
    // Transmitter caused interrupt ?
    //
    // Note: Since multiple interrupts can trigger the ISR, both the enabling
    // of the interrupt source, and the status flag must be checked.  That is,
    // it is possible for the status flag to be set (Tx is empty) but the
    // transmitter interrupt to be disabled.
    //
    if( ( IEC0bits.U1TXIE == 1 ) &&
        ( IFS0bits.U1TXIF == 1 ) )
    {
        // Service the transmitter.
        UARTBufTX();
    }
ts_end(UART1_ISR);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Hardware buffer availability SPI2 interrupt.
///
/// @note   Only receiver is enabled for UART2, so if interrupt occurs it must
/// be b/c of the receiver.
///
/// This function empties the received hardware buffer to keep 100% utilization 
/// of the UART interface when receiving data.
////////////////////////////////////////////////////////////////////////////////
void __ISR ( _UART_2_VECTOR, IPL1SOFT) UART2ISR( void ) 
{
ts_start(UART2_ISR);
    // Service the receiver.
    UARTBufRx( UART_MODULE_2 );
ts_end(UART2_ISR);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Buffer received UART data.
///
/// @param  mod_sel
///             The hardware module to read received data.
///
/// This function manages received hardware to receive data.
////////////////////////////////////////////////////////////////////////////////
static void UARTBufRx( UART_MODULE_E mod_sel )
{ 
    UART_RX_BUF_S* rx_buf_elem;
    uint8_t        read_cnt;

    // Perform intermediate access of current receiver buffer element used 
    // for storing data to increase processing speed and reduce line length.
    rx_buf_elem = &uart_rx_cb[ mod_sel ].buf_arr[ uart_rx_cb[ mod_sel ].buf_idx ];
    
    // Receive available data.
    read_cnt = UARTRead( mod_sel,
                         &rx_buf_elem->data[ rx_buf_elem->data_len ],
                         ( UART_RX_BUF_DATA_LEN - rx_buf_elem->data_len ) );

    // Manage buffer control variables.
    rx_buf_elem->data_len += read_cnt;
    
    // Clear the interrupt flag.
    //
    // Note: this must be performed after the condition which caused the 
    // interrupt (i.e. received data) is addressed.
    //
    switch( mod_sel )
    {
        case UART_MODULE_1:
        {
            IFS0CLR = _IFS0_U1RXIF_MASK;
            break;
        }
        case UART_MODULE_2:
        {
            IFS1CLR = _IFS1_U2RXIF_MASK;
            break;
        }
        default:
        {
            Nop(); // Do nothing, invalid module number.
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Read UART data.
///
/// @param  mod_sel
///             Hardware module to read received data.
/// @param  data_p
///             Software buffer to store received data.
/// @param  data_len
///             Length of data to read (in bytes).
///
/// @return Number of bytes of read data.
///
/// @note   Higher-level integrity checks should be performed for the serial 
///         data (e.g. checksum, CRC, etc.) for verifying content.  Low-level
///         communication failures (e.g. parity-bit failure) are internally 
///         recorded to provide visibility during development.
///
/// @note   The function will stored data within \p data_p until \p data_p is 
///         full.  When \p data_p is full, receiver hardware must still be read 
///         to service hardware operation.  Read hardware data when \p data_p is
///         full is lost.
///
/// This function stored received UART data from hardware buffer to module data.
////////////////////////////////////////////////////////////////////////////////
static uint8_t UARTRead( UART_MODULE_E mod_sel, uint8_t* data_p, uint16_t data_len )
{
    typedef struct
    {
        volatile __U1STAbits_t* uart_sta_union_p;
        volatile uint32_t*      uart_staclr_u32_p;
        volatile uint32_t*      uart_rxreg_u32_p;
        
    } UART_READ_REG_S;
    
    // Note: __U1StAbits_t and __U2StAbits_t are identical bit fields.  Pointer
    // typecast to allow common behavior of function.
    static const UART_READ_REG_S uart_reg_p[ UART_MODULE_MAX ] =
    {
        {                 &U1STAbits, &U1STACLR, &U1RXREG },    // UART_MODULE_1
        { (__U1STAbits_t*)&U2STAbits, &U2STACLR, &U2RXREG },    // UART_MODULE_2
    };
    
    uint8_t data_cnt = 0;
    uint8_t rx_byte;    
    
    // Hardware buffer overflow has occurred ?
    if( uart_reg_p[ mod_sel ].uart_sta_union_p->OERR == 1 )
    {
        // Latch identification of overflow error.
        uart_oerr_latch[ mod_sel ] = true;
        
        // Clear the hardware overflow hardware flag.
        //
        // Note: The overflow flag needs to be cleared because all UART
        // reception is inhibited while this hardware flag is set.
        //
        *uart_reg_p[ mod_sel ].uart_staclr_u32_p = _U1STA_OERR_MASK;
    }
    
    // Read data from the hardware buffer until it is empty.
    while( uart_reg_p[ mod_sel ].uart_sta_union_p->URXDA == 1 )
    {
        // Parity or framing error with received byte ?
        if( ( uart_reg_p[ mod_sel ].uart_sta_union_p->PERR == 1 ) ||
            ( uart_reg_p[ mod_sel ].uart_sta_union_p->FERR == 1 ) )
        {
            uart_err_cnt[ mod_sel ]++;
        }
        
        // Read byte from hardware buffer.
        rx_byte = (uint8_t) *uart_reg_p[ mod_sel ].uart_rxreg_u32_p;
        
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

////////////////////////////////////////////////////////////////////////////////
/// @brief  Transmit UART data.
///
/// This function manages transmitter hardware to transmit data.
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
/// @brief  Write UART data.
///
/// @param  data_p
///             Software buffer of transmit data.
/// @param  data_len
///             Length of data to write (in bytes).
///
/// @return Number of bytes of transmitted data.
///
/// This function sets UART hardware with transmit data.
////////////////////////////////////////////////////////////////////////////////
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