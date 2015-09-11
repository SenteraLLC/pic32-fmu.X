////////////////////////////////////////////////////////////////////////////////
/// @file   
/// @brief  Controller Area Network (CAN) driver. 
////////////////////////////////////////////////////////////////////////////////

#ifndef CAN_H_
#define	CAN_H_

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

/// List of transmitted messages.
typedef enum
{
    CAN_TX_MSG_SERVO_CMD,
    CAN_TX_MSG_NODE_STATUS,
    CAN_TX_MSG_NODE_VER,
    CAN_TX_MSG_CFG_WRITE_REQ,
    CAN_TX_MSG_CFG_READ_REQ,
    
    CAN_TX_MSG_NUM_OF
    
} CAN_TX_MSG_TYPE_E;

/// List of received messages.
typedef enum
{
    CAN_RX_MSG_SNODE_SERVO_STATUS,
    CAN_RX_MSG_SNODE_VSENSE_DATA,
    CAN_RX_MSG_SNODE_STATUS,
    CAN_RX_MSG_SNODE_VERSION,
    CAN_RX_MSG_CFG_WRITE_RESP,
    CAN_RX_MSG_CFG_READ_RESP,
    
    CAN_RX_MSG_NUM_OF
    
} CAN_RX_MSG_TYPE_E;

//
// TRANSMIT MESSAGES -----------------------------------------------------------
//

/// Payload content of Servo Command message.
typedef union
{
    uint32_t data_u32[ 2 ];
    
    struct __attribute__ ((packed))
    {
        uint16_t cmd_type;
        uint16_t cmd_pwm;
        uint16_t cmd_pos;
    };
    
} CAN_TX_SERVO_CMD_U;

/// Payload content of Node Status message.
typedef union
{
    uint32_t data_u32[ 2 ];
    
    struct __attribute__ ((packed))
    {
        uint16_t reset_condition;
        uint16_t reset_detail;
    };
    
} CAN_TX_NODE_STATUS_U;

/// Payload content of Node Version message.
typedef union
{
    uint32_t data_u32[ 2 ];
    
    struct __attribute__ ((packed))
    {
        uint8_t  node_type;
        uint8_t  rev_ver;
        uint8_t  min_ver;
        uint8_t  maj_ver;
        uint32_t serial_num;
    };
    
} CAN_TX_NODE_VER_U;

/// Payload content of Configuration Write Request message.
typedef union
{
    uint32_t data_u32[ 2 ];
    
    struct __attribute__ ((packed))
    {
        uint16_t cfg_sel;
        
        union
        {
            uint8_t cfg_val_u8;
            int32_t cfg_val_i32;
        };

    };
    
} CAN_TX_WRITE_REQ_U;

/// Payload content of Configuration Read Request message.
typedef union
{
    uint32_t data_u32[ 2 ];
    
    struct __attribute__ ((packed))
    {
        uint16_t cfg_sel;

    };
    
} CAN_TX_READ_REQ_U;

//
// RECEIVE MESSAGES -----------------------------------------------------------
//
        
/// Payload content of Servo-node Servo Status Message.
typedef union
{
    uint32_t data_u32[ 2 ];
    
    struct __attribute__ ((packed))
    {
        uint16_t cmd_type_echo;
        uint16_t pwm_act;
        uint16_t servo_voltage;
        uint16_t servo_current;
    };
    
} CAN_RX_MSG_SNODE_SERVO_STATUS_U;

/// Payload content of Servo-node VSENSE Data message.
typedef union
{
    uint32_t data_u32[ 2 ];
    
    struct __attribute__ ((packed))
    {
        uint16_t vsense1_raw;
        int16_t  vsense1_cor;
        uint16_t vsense2_raw;
        int16_t  vsense2_cor;
    };
    
} CAN_RX_MSG_SNODE_VSENSE_DATA_U;

/// Payload content of Node Servo-node Status message.
typedef union
{
    uint32_t data_u32[ 2 ];
    
    struct __attribute__ ((packed))
    {
        uint16_t reset_condition;
        uint16_t reset_detail;
    };
    
} CAN_RX_MSG_SNODE_STATUS_U;

/// Payload content of Node Servo-node Version message.
typedef union
{
    uint32_t data_u32[ 2 ];
    
    struct __attribute__ ((packed))
    {
        uint8_t  node_type;
        uint8_t  rev_ver;
        uint8_t  min_ver;
        uint8_t  maj_ver;
        uint32_t serial_num;
    };
    
} CAN_RX_MSG_SNODE_VERSION_U;

/// Payload content of Configuration Write Response message.
typedef union
{
    uint32_t data_u32[ 2 ];
    
    struct __attribute__ ((packed))
    {
        uint16_t cfg_sel;
        uint16_t fault_status;
    };
    
} CAN_RX_WRITE_RESP_U;

/// Payload content of Configuration Read Response message.
typedef union
{
    uint32_t data_u32[ 2 ];
    
    struct __attribute__ ((packed))
    {
        uint16_t cfg_sel;
        
        union
        {
            uint8_t cfg_val_u8;
            int32_t cfg_val_i32;
        };
        
    };
    
} CAN_RX_READ_RESP_U;

// *****************************************************************************
// ************************** Declarations *************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

////////////////////////////////////////////////////////////////////////////////
/// @brief  Initialize CAN hardware.
////////////////////////////////////////////////////////////////////////////////
void CANInit( void );

////////////////////////////////////////////////////////////////////////////////
/// @brief  Queue CAN message for transmission.
///
/// @param  tx_msg_type
///             Type of message transmitted.
/// @param  dest_id
///             The destination node's ID, to receive the message.
/// @param  payload
///             Payload of message to transmit.
///
/// @note   When not applicable, the calling application should set
///         \p dest_id to zero.
////////////////////////////////////////////////////////////////////////////////
void CANTxSet( CAN_TX_MSG_TYPE_E tx_msg_type, 
               uint8_t dest_id, 
               const uint32_t payload[ 2 ] );

////////////////////////////////////////////////////////////////////////////////
/// @brief  Read received CAN message.
///
/// @param  rx_msg_type
///             Type of message received.
/// @param  src_id_p
///             Variable for storing the received message's source ID.
/// @param  payload
///             Buffer for storing the received message's payload.
///
/// @note   \p src_id_p can be set to NULL when the Source ID is not applicable
///         or not needed.
///
/// @return true  - returned data is value.  
///         false - returned data is invalid.
////////////////////////////////////////////////////////////////////////////////
bool CANRxGet ( CAN_RX_MSG_TYPE_E rx_msg_type, 
                uint8_t* src_id_p,
                uint32_t payload[ 2 ] );

#endif	// CAN_H_