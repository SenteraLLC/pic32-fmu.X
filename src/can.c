////////////////////////////////////////////////////////////////////////////////
/// @file   
/// @brief  Controller Area Network (CAN) driver. 
////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

#include <sys/kmem.h>

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

#include "can.h"

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

// FIFO0  - Tx - Servo Command                  - 32 message buffer
// FIFO1  - Tx - Node Status                    -  1 message buffer
// FIFO2  - Tx - Node Version                   -  1 message buffer
// FIFO3  - Tx - Configuration Write Request    -  1 message buffer
// FIFO4  - Tx - Configuration Read Request     -  1 message buffer
//
// FIFO5  - Rx - S-Node Servo Status            - 20 message buffer
// FIFO6  - Rx - S-Node VSENSE Data             - 20 message buffer
// FIFO7  - Rx - S-Node Node Status             - 20 message buffer
// FIFO8  - Rx - S-Node Node Version            - 20 message buffer
// FIFO9  - Rx - Configuration Write Response   -  1 message buffer
// FIFO10 - Rx - Configuration Read Response    -  1 message buffer
//
// FIFO11 - Not used.
// FIFO12 - Not used.
// :
// FIFO31 - Not used.
// 
// Each transmit message requires 4 words (16 bytes) and each receive 
// message requires 4 words (16 bytes).  Therefore, the total message allocation
//  size is:
//  = ( 4 words * 36 ) + ( 4 words * 82 ) = 472 words
//
static uint32_t can_msg_buf[ 472 ];

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

static void CANTxBuildHeader( CAN_TX_MSG_TYPE_E tx_msg_type, 
                              uint8_t dest_id, 
                              uint32_t msg_buf[ 4 ] );

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

void CANInit( void )
{
    // CAN_EN transceiver pin (pin 27) configuration
    AD1PCFGbits.PCFG12  = 1;    // AN12: Analog input pin in digital mode.
    ODCBbits.ODCB12     = 0;    // RB12: Normal output (i.e. not open-drain)
    TRISBbits.TRISB12   = 0;    // RB12: Output
    LATBbits.LATB12     = 1;    // RB12: Logic High
    
    // CAN2 transmit pin (pin 21) configuration
    AD1PCFGbits.PCFG8   = 1;    // AN8: Analog input pin in digital mode.
    
    // CAN2 Receive pin (pin 29) configuration
    AD1PCFGbits.PCFG14  = 1;    // AN14: Analog input pin in digital mode.
    
    // Enable the CAN2 module.
    C2CONbits.ON = 1;
    
    // Request hardware configuration mode and wait for entry.
    C2CONbits.REQOP = 0b100;
    while( C2CONbits.OPMOD != 0b100 );
    
    // Fsys  = 80MHz
    // Fbaud = 1Mbps
    //
    // Ftq := Time Quantum Frequency (selected as 10MHz - i.e. 10 TQ per bit)
    //
    // BPR = ( Fsys  / ( 2 * Ftq   ) ) - 1
    //     = ( 80MHz / ( 2 * 10MHz ) ) - 1
    //     = 3
    //
    // Synch Segment     = 1 TQ (constant)
    // Propagation Delay = 3 TQ
    // Phase Segment 1   = 3 TQ
    // Phase Segment 2   = 3 TQ
    // SJW               = Min( 4, Prop, PS1, PS2 ) = 3 TQ
    //
    C2CFGbits.BRP       = 3;
    C2CFGbits.SJW       = 0b10;
    C2CFGbits.PRSEG     = 0b010;
    C2CFGbits.SEG1PH    = 0b010;
    C2CFGbits.SAM       = 1;
    C2CFGbits.SEG2PHTS  = 1;
    C2CFGbits.SEG2PH    = 0b010;
    
    // Initialize C2FIFOBA register with physical address of CAN message
    // allocation.
    C2FIFOBA = KVA_TO_PA( can_msg_buf );
    
    // FIFO0  - Tx - Servo Command (highest priority)
    C2FIFOCON0bits.FSIZE = 0b11111;
    C2FIFOCON0bits.TXEN  = 1;
    C2FIFOCON0bits.TXPRI = 0b11;
    
    // FIFO1  - Tx - Node Status (lowest priority)
    C2FIFOCON1bits.FSIZE = 0b0;
    C2FIFOCON1bits.TXEN  = 1;
    C2FIFOCON1bits.TXPRI = 0b00;
    
    // FIFO2  - Tx - Node Version (lowest priority)
    C2FIFOCON2bits.FSIZE = 0b0;
    C2FIFOCON2bits.TXEN  = 1;
    C2FIFOCON2bits.TXPRI = 0b00;
    
    // FIFO3  - Tx - Configuration Write Request (low-intermediate priority)
    C2FIFOCON3bits.FSIZE = 0b0;
    C2FIFOCON3bits.TXEN  = 1;
    C2FIFOCON3bits.TXPRI = 0b01;
    
    // FIFO4  - Tx - Configuration Read Request (low-intermediate priority)
    C2FIFOCON4bits.FSIZE = 0b0;
    C2FIFOCON4bits.TXEN  = 1;
    C2FIFOCON4bits.TXPRI = 0b01;
    
    // FIFO5  - Rx - S-Node Servo Status
    C2FIFOCON5bits.FSIZE = 0b10011;
    C2FIFOCON5bits.TXEN  = 0;
    C2FIFOCON5bits.DONLY = 0;
    
    // FIFO6  - Rx - S-Node VSENSE Data
    C2FIFOCON5bits.FSIZE = 0b10011;
    C2FIFOCON5bits.TXEN  = 0;
    C2FIFOCON5bits.DONLY = 0;
    
    // FIFO7  - Rx - S-Node Node Status
    C2FIFOCON5bits.FSIZE = 0b10011;
    C2FIFOCON5bits.TXEN  = 0;
    C2FIFOCON5bits.DONLY = 1;
    
    // FIFO8  - Rx - S-Node Node Version
    C2FIFOCON5bits.FSIZE = 0b10011;
    C2FIFOCON5bits.TXEN  = 0;
    C2FIFOCON5bits.DONLY = 0;    
    
    // FIFO9  - Rx - Configuration Write Response
    C2FIFOCON5bits.FSIZE = 0b0;
    C2FIFOCON5bits.TXEN  = 0;
    C2FIFOCON5bits.DONLY = 0;
    
    // FIFO10 - Rx - Configuration Read Response
    C2FIFOCON6bits.FSIZE = 0b0;
    C2FIFOCON6bits.TXEN  = 0;
    C2FIFOCON6bits.DONLY = 0;
    
    // Mask 0
    //  - Data Type ID          28-19   Filtered.
    //  - Transfer Type         18-17   Filtered.
    //  - Source Node ID        16-10   Not-Filtered.
    //  - Reserved              9-7     Not-Filtered.
    //  - Destination Node ID   6-0     Not-Filtered.
    //  
    C2RXM0bits.SID  = 0x7FF;
    C2RXM0bits.EID  = 0x20000;
    C2RXM0bits.MIDE = 1;        // Match only message types.
    
    // Mask 1
    //  - Data Type ID          28-19   Filtered.
    //  - Transfer Type         18-17   Filtered.
    //  - Source Node ID        16-10   Not-Filtered.
    //  - Reserved              9-7     Not-Filtered.
    //  - Destination Node ID   6-0     Filtered.
    //  
    C2RXM1bits.SID  = 0x7FF;
    C2RXM1bits.EID  = 0x2007F;
    C2RXM1bits.MIDE = 1;        // Match only message types.

    // Filter 0
    //  - Data Type ID          = 20   (S-Node Servo Status)
    //  - Transfer Type         = 0b10 (Message Broadcast)
    //
    C2RXF0bits.SID          = 0x029;
    C2RXF0bits.EID          = 0x00000;
    C2RXF0bits.EXID         = 1;        // Match messages only with extended ID.
    C2FLTCON0bits.FSEL0     = 5;        // Store messages in FIFO5.
    C2FLTCON0bits.MSEL0     = 0;        // Use acceptance mask 0.
    C2FLTCON0bits.FLTEN0    = 1;        // Enable Filter 0.
    
    // Filter 1
    //  - Data Type ID          = 21   (S-Node VSENSE Data)
    //  - Transfer Type         = 0b10 (Message Broadcast)
    //
    C2RXF1bits.SID          = 0x02B;
    C2RXF1bits.EID          = 0x00000;
    C2RXF1bits.EXID         = 1;        // Match messages only with extended ID.
    C2FLTCON0bits.FSEL1     = 6;        // Store messages in FIFO6.
    C2FLTCON0bits.MSEL1     = 0;        // Use acceptance mask 0.
    C2FLTCON0bits.FLTEN1    = 1;        // Enable Filter 1.
    
    // Filter 2
    //  - Data Type ID          = 770  (S-Node Node Version)
    //  - Transfer Type         = 0b10 (Message Broadcast)
    //
    C2RXF2bits.SID          = 0x605;
    C2RXF2bits.EID          = 0x00000;
    C2RXF2bits.EXID         = 1;        // Match messages only with extended ID.
    C2FLTCON0bits.FSEL2     = 7;        // Store messages in FIFO7.
    C2FLTCON0bits.MSEL2     = 0;        // Use acceptance mask 0.
    C2FLTCON0bits.FLTEN2    = 1;        // Enable Filter 2.
    
    // Filter 3
    //  - Data Type ID          = 771  (S-Node Node Version)
    //  - Transfer Type         = 0b10 (Message Broadcast)
    //
    C2RXF3bits.SID          = 0x607;
    C2RXF3bits.EID          = 0x00000;
    C2RXF3bits.EXID         = 1;        // Match messages only with extended ID.
    C2FLTCON0bits.FSEL3     = 8;        // Store messages in FIFO8.
    C2FLTCON0bits.MSEL3     = 0;        // Use acceptance mask 0.
    C2FLTCON0bits.FLTEN3    = 1;        // Enable Filter 3.
    
    // Filter 4
    //  - Data Type ID          = 800  (Configuration Write Response)
    //  - Transfer Type         = 0b00 (Service Response)
    //  - Destination Node ID   = 0    (FMU ID)
    //
    C2RXF4bits.SID          = 0x640;
    C2RXF4bits.EID          = 0x00000;
    C2RXF4bits.EXID         = 1;        // Match messages only with extended ID.
    C2FLTCON1bits.FSEL4     = 9;        // Store messages in FIFO9.
    C2FLTCON1bits.MSEL4     = 1;        // Use acceptance mask 1.
    C2FLTCON1bits.FLTEN4    = 1;        // Enable Filter 4.
    
    // Filter 5
    //  - Data Type ID          = 801  (Configuration Read Response)
    //  - Transfer Type         = 0b00 (Service Response)
    //  - Destination Node ID   = 0    (FMU ID)
    //
    C2RXF5bits.SID          = 0x642;
    C2RXF5bits.EID          = 0x00000;
    C2RXF5bits.EXID         = 1;        // Match messages only with extended ID.
    C2FLTCON1bits.FSEL5     = 10;       // Store messages in FIFO10.
    C2FLTCON1bits.MSEL5     = 1;        // Use acceptance mask 1.
    C2FLTCON1bits.FLTEN5    = 1;        // Enable Filter 5.
    
    // Request hardware normal mode and wait for entry.
    C2CONbits.REQOP = 0;
    while( C2CONbits.OPMOD != 0 );
}

void CANTxSet ( CAN_TX_MSG_TYPE_E tx_msg_type, uint8_t dest_id, const uint32_t payload[ 2 ] )
{
    // Structure definition of HW elements corresponding the a message type.
    typedef struct
    {
        volatile uint32_t* fifoua_p;
        volatile uint32_t* fifoint_p;
        volatile uint32_t* fifoconset_p;
        volatile uint32_t* fifocon_p;
       
    } TX_HW_MAP_S;
    
    // Mapping of message types to hardware elements for transmitting the message.
    static const TX_HW_MAP_S tx_hw_map[ CAN_TX_MSG_NUM_OF ] = 
    {
        { &C2FIFOUA0, &C2FIFOINT0, &C2FIFOCON0SET, &C2FIFOCON0 },   // CAN_TX_MSG_SERVO_CMD
        { &C2FIFOUA1, &C2FIFOINT1, &C2FIFOCON1SET, &C2FIFOCON1 },   // CAN_TX_MSG_NODE_STATUS
        { &C2FIFOUA2, &C2FIFOINT2, &C2FIFOCON2SET, &C2FIFOCON2 },   // CAN_TX_MSG_NODE_VER
        { &C2FIFOUA3, &C2FIFOINT3, &C2FIFOCON3SET, &C2FIFOCON3 },   // CAN_TX_MSG_CFG_WRITE_REQ
        { &C2FIFOUA4, &C2FIFOINT4, &C2FIFOCON4SET, &C2FIFOCON4 },   // CAN_TX_MSG_CFG_READ_REQ
    };
    
    uint8_t     payload_idx;
    uint32_t*   msg_p;
    
    // Transmit FIFO is not already full ?
    if( ( *tx_hw_map[ tx_msg_type ].fifoint_p & _C1FIFOINT0_TXNFULLIF_MASK ) != 0 )
    {
        // Get the address of the message buffer to write to from the C2FIFOUAn
        // register. Convert this physical address to virtual address.
        msg_p = PA_TO_KVA1( *tx_hw_map[ tx_msg_type ].fifoua_p );
        
        // Copy the payload to the transmit buffer.
        for( payload_idx = 0;
             payload_idx < 2;
             payload_idx++ )
        {
            // Note: First 2 words of hardware buffer are used for CAN ID,
            // DLC, and control bits.
            msg_p[ payload_idx + 2 ] = payload[ payload_idx ];
        }

        // Build the CAN message header.
        CANTxBuildHeader( tx_msg_type, dest_id, msg_p );
        
        // Increment the FIFO index.
        *tx_hw_map[ tx_msg_type ].fifoconset_p = _C1FIFOCON0_UINC_MASK;
        
        // Request (i.e. set request bit to '1') the transmission.
        *tx_hw_map[ tx_msg_type ].fifoconset_p = _C1FIFOCON0_TXREQ_MASK;
    }
}

bool CANRxGet ( CAN_RX_MSG_TYPE_E rx_msg_type, uint8_t* src_id_p, uint32_t payload[ 2 ] )
{
    // Structure definition of HW elements corresponding the a message type.
    typedef struct
    {
        volatile uint32_t* fifoua_p;
        volatile uint32_t* fifoint_p;
        volatile uint32_t* fifoconset_p;
       
    } RX_HW_MAP_S;
    
    static const RX_HW_MAP_S rx_hw_map[ CAN_RX_MSG_NUM_OF ] = 
    {
        { &C2FIFOUA5,  &C2FIFOINT5,  &C2FIFOCON5SET  },  // CAN_RX_MSG_SNODE_SERVO_STATUS
        { &C2FIFOUA6,  &C2FIFOINT6,  &C2FIFOCON6SET  },  // CAN_RX_MSG_SNODE_VSENSE_DATA
        { &C2FIFOUA7,  &C2FIFOINT7,  &C2FIFOCON7SET  },  // CAN_RX_MSG_SNODE_STATUS
        { &C2FIFOUA8,  &C2FIFOINT8,  &C2FIFOCON8SET  },  // CAN_RX_MSG_SNODE_VERSION
        { &C2FIFOUA9,  &C2FIFOINT9,  &C2FIFOCON9SET  },  // CAN_RX_MSG_CFG_WRITE_RESP
        { &C2FIFOUA10, &C2FIFOINT10, &C2FIFOCON10SET },  // CAN_RX_MSG_CFG_READ_RESP
    };
    
    uint8_t     payload_idx;
    bool        data_rx_flag = false;
    uint32_t*   msg_p;
    
    // Message in receiver FIFO ?
    if( ( *rx_hw_map[ rx_msg_type ].fifoint_p & _C1FIFOINT0_RXNEMPTYIF_MASK ) != 0 )
    {
        // Identify data as received.
        data_rx_flag  = true;
        
        // Get the address of the message buffer to read from the C2FIFOUA
        // register. Convert this physical address to virtual address.
        msg_p = PA_TO_KVA1( *rx_hw_map[ rx_msg_type ].fifoua_p );
        
        // Source ID pointer is valid ?
        if( src_id_p != NULL )
        {
            // Copy the source ID into the supplied variable.  The source ID is
            // stored in bits 20-26 of hardware buffer word 1.
            *src_id_p = (uint8_t) ( ( msg_p[ 1 ] >> 20 ) & 0x7F );
        }

        // Copy payload into supplied buffer.
        for ( payload_idx = 0;
              payload_idx < 2;
              payload_idx++ )
        {
            // Note: First 2 words of hardware buffer are used for CAN ID,
            // DLC, and control bits.
            payload[ payload_idx ] = msg_p[ payload_idx + 2 ];
        }
        
        // Increment the FIFO index to identify message as received.
        *rx_hw_map[ rx_msg_type ].fifoconset_p = _C1FIFOCON0_UINC_MASK;
    }
 
    return data_rx_flag;
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************

static void CANTxBuildHeader ( CAN_TX_MSG_TYPE_E tx_msg_type, 
                               uint8_t dest_id, 
                               uint32_t msg_buf[ 4 ] )
{
    // Union defining the contents of the CAN ID field.
    typedef union
    {
        struct
        {
            uint32_t dest_id    :  7;   // bits 6 - 0
            uint32_t            :  3;   // bits 9 - 7 (reserved)
            uint32_t src_id     :  7;   // bits 16-10
            uint32_t tsf_type   :  2;   // bits 18-17
            uint32_t data_type  : 10;   // bits 28-19
        };
        
        struct
        {
            uint32_t id_lo     : 18;   // bits 17 - 0
            uint32_t id_hi     : 11;   // bits 28-18
        };
        
    } CAN_ID_U;
    
    // Structure defining the contents of the CAN header and footer.
    typedef struct
    {
        uint8_t  data_len;
        CAN_ID_U can_id;
        
    } CAN_DATA_S;
    
    // Structure defining the contents of the CAN message header within the
    // hardware buffer.
    typedef union
    {
        struct
        {
            // Word 1.
            uint32_t sid    : 11;   // bits  0 - 10
            uint32_t        : 21;   // bits 11 - 31

            // Word 2.
            uint32_t dlc    :  4;   // bits  0 -  3
            uint32_t rb0    :  1;   // bits  4
            uint32_t        :  3;   // bits  5 -  7
            uint32_t rb1    :  1;   // bits  8
            uint32_t rtr    :  1;   // bits  9
            uint32_t eid    :  18;  // bits 10 - 27
            uint32_t ide    :  1;   // bits 28
            uint32_t srr    :  1;   // bits 29
            uint32_t        :  2;   // bits 30 - 31
        };
        
        uint32_t data_u32[ 2 ];
        
    } CAN_HW_HEADER_U;
    
    // Defined messages' header content and message length.
    static const CAN_DATA_S tx_can_data[ CAN_TX_MSG_NUM_OF ] =
    {
        // CAN_TX_MSG_SERVO_CMD
        {
            6,                  // data_len (bytes)
            
            {
                {
                    0,          // dest_id      - N/A, set real-time.
                    0,          // src_id       - FMU (ID = 0).        
                    0b11,       // tsf_type     - Message unicast.
                    10,         // data_type    - 10 identifies Servo Command Message.
                },
            },
        },
        
        // CAN_TX_MSG_NODE_STATUS
        {
            4,                  // data_len (bytes)
            
            {
                {
                    0,          // dest_id      - N/A, broadcast message.
                    0,          // src_id       - FMU (ID = 0).        
                    0b10,       // tsf_type     - Message broadcast.
                    770,        // data_type    - 770 identifies Node Status Message.
                },
            },
        },
        
        // CAN_TX_MSG_NODE_VER
        {
            8,                  // data_len (bytes)
            
            {
                {
                    0,          // dest_id      - N/A, broadcast message.
                    0,          // src_id       - FMU (ID = 0).       
                    0b10,       // tsf_type     - Message broadcast.
                    771,        // data_type    - 771 identifies Node Version Message.
                },
            },
        },
        
        // CAN_TX_MSG_CFG_WRITE_REQ
        {
            0,                  // data_len     - Variable length, set during run-time.
            
            {
                {
                    0,          // dest_id      - N/A, set real-time.
                    0,          // src_id       - FMU (ID = 0).         
                    0b01,       // tsf_type     - Service Request.
                    800,        // data_type    - 800 identifies Configuration Write Request Message.
                },
            },
        },
        
        // CAN_TX_MSG_CFG_READ_REQ
        {
            2,                  // data_len (bytes)  
            
            {
                {
                    0,          // dest_id      - N/A, set real-time.
                    0,          // src_id       - FMU (ID = 0).         
                    0b01,       // tsf_type     - Service Request.
                    801,        // data_type    - 801 identifies Configuration Read Request Message.
                },
            },
        },
    };
    
    //
    // START OF OPERATIONAL CODE AND LOCAL VARIABLE DEFINITIONS ----------------
    //
    
    CAN_ID_U can_id;
    
    // Define the header content.  Set fields which are identical independent
    // of the message type.
    CAN_HW_HEADER_U tx_hw_header =
    {
        {         
            0, // sid - updated in fxn.
            0, // dlc - updated in fxn.
            0, // rb0 - always dominant.
            0, // rb1 - always dominant.
            0, // rtr - always dominant for data frames.
            0, // eid - updated in fxn.
            1, // ide - always recessive.
            1, // srr - always recessive.    
        },
    };
    
    // Copy the CAN ID from NVM and update the destination ID.
    can_id         = tx_can_data[ tx_msg_type ].can_id;
    can_id.dest_id = dest_id;
    
    // Populate the hardware header format with the CAN ID.
    tx_hw_header.sid = can_id.id_hi;
    tx_hw_header.eid = can_id.id_lo;
    tx_hw_header.dlc = tx_can_data[ tx_msg_type ].data_len;
            
    // Handle the special case of Configuration Write Request which has a
    // variable length.
    if( tx_msg_type == CAN_TX_MSG_CFG_WRITE_REQ )
    {
        // For the Configuration Write Request message, the first 16-bits within
        // the payload (bits 0-16 of word 2) identifies the type of data 
        // returned.  Possible data includes:
        //
        //  Payload             Description             Length (in bytes)
        //  0                   Node ID                 1
        //  1-6                 PWM coefficients        4 (each)
        //  7-12                VSENSE1 Coefficients    4 (each)
        //  13-18               VSENSE2 Coefficients    4 (each)
        //
        // The data length (dlc) for each Configuration Write Request Message is 
        // 2 bytes for the type identifier plus the value's length.
        //
        if( ( msg_buf[ 2 ] & 0x0000FFFF ) == 0 )
        {
            tx_hw_header.dlc = 2 + 1;
        }
        else
        {
            tx_hw_header.dlc = 2 + 4;
        }
    }
    else
    {
        // The message is not a Configuration Write Request.  The length is
        // computed statically.
        tx_hw_header.dlc = tx_can_data[ tx_msg_type ].data_len;
    }
    
    // Copy the hardware header into the transmit buffer.
    msg_buf[ 0 ] = tx_hw_header.data_u32[ 0 ];
    msg_buf[ 1 ] = tx_hw_header.data_u32[ 1 ];
}