////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Servo-node management.
////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

#include "snode.h"
#include "fmucomm.h"
#include "can.h"
#include "coretime.h"

// *****************************************************************************
// ************************** Macros *******************************************
// *****************************************************************************

// The number of Servo-Nodes available in the module data buffer.
#define SNODE_RX_BUF_SIZE 20

// Maximum number of CAN message read on each 'Task' execution.
#define SNODE_CTRL_DATA_MSG_MAX 4

// Number of micro-seconds after which an Servo-Node is invalidated, when
// fresh CAN data is not received.
#define SNODE_CTRL_DATA_TIMEOUT_US 50000

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

// Structure defining the contents of a Servo-node received data.
typedef struct
{
    bool     valid;
    uint32_t rx_time_us;
    
    CAN_RX_MSG_SNODE_SERVO_STATUS_U servo_status;
    CAN_RX_MSG_SNODE_VSENSE_DATA_U  vsense_data;
    CAN_RX_MSG_SNODE_STATUS_U       node_status;
    CAN_RX_MSG_SNODE_VERSION_U      node_version;

} SNODE_RX_DATA;

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

// Module data is managed with one level of pointer indirection to easy sorting
// of CAN node data.  The ID Buffer is indexed with a Servo-node ID to access
// its data.  All valid pointers within the ID buffer reference an element of 
// the Data Buffer.  A NULL pointer in the ID Buffer identifies CAN Node data 
// which is invalid/not-received.
//
// The Data Buffer is defined with a size smaller than the ID Buffer.  This
// is because to manage memory use since fewer number of Servo-nodes are
// required to be supported at any one time.
//
static SNODE_RX_DATA  snode_rx_data_buf[ SNODE_RX_BUF_SIZE ];
static SNODE_RX_DATA* snode_rx_data_id_p[ 128 ] = { 0 };

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

static void SNodeCtrlDataTask( void );
static void SNodeCtrlCmdTask( void );

static void SNodeCANRx( void );
static void SNodeCANTimeout( void );
static uint8_t SNodeCtrlDataBuild( FMUCOMM_CTRL_SURFACE_DATA_PL* eth_ctrl_data_msg );
static void SNodeBufSetup( uint8_t node_id );

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

void SNodeTask( void )
{
    SNodeCtrlDataTask();
    SNodeCtrlCmdTask();
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************

////////////////////////////////////////////////////////////////////////////////
/// @brief  Servo-node control data task.
///
/// Manage the process of storing data and construction of the Control
/// Surface Data Ethernet packet.
////////////////////////////////////////////////////////////////////////////////
static void SNodeCtrlDataTask( void )
{
    static enum
    {            
        SM_AGE_EVAL,     
        SM_ETH_BUILD,    
        SM_RX_MSGS,
                
    } taskState = SM_RX_MSGS;
    
    static uint32_t time_base_us;
    static bool     tx_msg_ready = false;
           uint8_t  surface_num_of;
           bool     queue_ok;
    
    FMUCOMM_CTRL_SURFACE_DATA_PL eth_ctrl_data_msg;

    // Required time has elapsed since last Control Surface Data transmission ?
    if( CoreTime32usGet() - time_base_us >= 10000 )
    {
        // Update time-base with timeout for next evaluation.
        time_base_us += 10000;
        
        // Transition to start the evaluation and build of a new Ethernet
        // packet.
        taskState = SM_AGE_EVAL;
    }
    
    switch( taskState )
    {
        case( SM_AGE_EVAL ):
        {
            // Inhibit request to transmit the Ethernet message.  New data is to
            // be constructed for the transmission.
            tx_msg_ready = false;
            
            // Evaluate age of received CAN data for each Servo-Node. Old/stall
            // servo nodes are to be invalidated.
            SNodeCANTimeout();
            
            taskState++;
            
            break;
        }
        case( SM_ETH_BUILD ):
        {
            // Build fresh Ethernet data for transmission.
            surface_num_of = SNodeCtrlDataBuild( &eth_ctrl_data_msg );
            
            // Identify request to transmit a message.
            tx_msg_ready = true;
            
            taskState++;
            
            break;
        }
        case( SM_RX_MSGS ):
        {
            // Read CAN messages into module data.
            SNodeCANRx();
            
            break;
        }
    }
    
    // Ethernet message ready for transmission ?
    if( tx_msg_ready == true )
    {
        // Queue the message for transmission.
        queue_ok = FMUCommSet( FMUCOMM_TYPE_CTRL_SURFACE_DATA, 
                               (uint8_t*) &eth_ctrl_data_msg,
                               sizeof( FMUCOMM_CTRL_SURFACE_DATA_PL_FIELD ) * surface_num_of );
        
        if( queue_ok == true )
        {
            tx_msg_ready = false;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Servo-node control command task.
///
/// This function manages forwarding of Control Surface Commands to the 
/// Servo-node CAN network.
////////////////////////////////////////////////////////////////////////////////
static void SNodeCtrlCmdTask( void )
{
    const FMUCOMM_RX_PKT*              eth_ctrl_cmd_p;
    const FMUCOMM_CTRL_SURFACE_CMD_PL* eth_ctrl_cmd_pl_p;
    
    CAN_TX_SERVO_CMD_U can_ctrl_cmd;
    
    uint8_t snodes_max;
    uint8_t snodes_idx;
    
    eth_ctrl_cmd_p = FMUCommGet( FMUCOMM_TYPE_CTRL_SURFACE_CMD );
    
    // Control Surface Command received ?
    if( eth_ctrl_cmd_p->valid == true )
    {
        // Typecast received packet to controller command type to access packet
        // content.
        eth_ctrl_cmd_pl_p = (FMUCOMM_CTRL_SURFACE_CMD_PL*) eth_ctrl_cmd_p->pl_p;
        
        // Determine number of nodes commanded
        snodes_max = (uint8_t) ( eth_ctrl_cmd_p->wrap.length / 
                                 sizeof( FMUCOMM_CTRL_SURFACE_CMD_PL_FIELD ) );
        
        // Loop through each node's information.
        for( snodes_idx = 0; snodes_idx < snodes_max; snodes_idx++ )
        {
            // Construct the CAN message to send with the received Ethernet
            // data.
            can_ctrl_cmd.cmd_type = eth_ctrl_cmd_pl_p->ctrlSurface[ snodes_idx ].cmdType;
            can_ctrl_cmd.cmd_pwm  = eth_ctrl_cmd_pl_p->ctrlSurface[ snodes_idx ].cmdPwm;
            can_ctrl_cmd.cmd_pos  = eth_ctrl_cmd_pl_p->ctrlSurface[ snodes_idx ].cmdPos;
            
            // Forward the command to the Servo-node.
            CANTxSet( CAN_TX_MSG_SERVO_CMD, 
                      eth_ctrl_cmd_pl_p->ctrlSurface[ snodes_idx ].surfaceID,
                      &can_ctrl_cmd.data_u32[ 0 ] );
        }
    } 
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Servo-node receive CAN messages.
///
/// Read up to 'SNODE_CTRL_DATA_MSG_MAX' number of CAN messages into module
/// data.
////////////////////////////////////////////////////////////////////////////////
static void SNodeCANRx( void )
{
    CAN_RX_MSG_SNODE_SERVO_STATUS_U servo_status_msg;
    CAN_RX_MSG_SNODE_VSENSE_DATA_U  vsense_data_msg;
    CAN_RX_MSG_SNODE_STATUS_U       node_status_msg;
    CAN_RX_MSG_SNODE_VERSION_U      node_version_msg;
            
    bool    rx_valid;
    uint8_t msg_idx = 0;
    uint8_t src_id;
    
    // Read Servo Status CAN message into module data while message or available
    // or 'SNODE_CTRL_DATA_MSG_MAX' message are read for the function's 
    // execution.
    rx_valid = true;
    for( ;
         ( msg_idx < SNODE_CTRL_DATA_MSG_MAX ) && ( rx_valid == true );
         msg_idx++ )
    {
        rx_valid = CANRxGet( CAN_RX_MSG_SNODE_SERVO_STATUS,
                             &src_id,
                             &servo_status_msg.data_u32[ 0 ] );

        // Data received ?
        if( rx_valid == true )
        {
            // Buffer for Source ID is not valid ?
            if( snode_rx_data_id_p[ src_id ] == NULL )
            {
                // Attempt to setup the buffer in module data.
                SNodeBufSetup( src_id );
            }
            
            // Buffer for Source ID is valid ?
            //
            // Note: if buffer for Source ID is still invalid, the setup of
            // buffer failed - the received data will be ignored.
            //
            if( snode_rx_data_id_p[ src_id ] != NULL )
            {
                snode_rx_data_id_p[ src_id ]->rx_time_us   = CoreTime32usGet();
                snode_rx_data_id_p[ src_id ]->servo_status = servo_status_msg;
            }
        }
    }

    // Read VSENSE Data CAN message into module data while message or available
    // or 'SNODE_CTRL_DATA_MSG_MAX' message are read for the function's 
    // execution.
    rx_valid = true;
    for( ;
         ( msg_idx < SNODE_CTRL_DATA_MSG_MAX ) && ( rx_valid == true );
         msg_idx++ )
    {
        rx_valid = CANRxGet( CAN_RX_MSG_SNODE_VSENSE_DATA,
                             &src_id,
                             &vsense_data_msg.data_u32[ 0 ] );

        // Data received ?
        if( rx_valid == true )
        {
            // Buffer for Source ID is not valid ?
            if( snode_rx_data_id_p[ src_id ] == NULL )
            {
                // Attempt to setup the buffer in module data.
                SNodeBufSetup( src_id );
            }
            
            // Buffer for Source ID is valid ?
            //
            // Note: if buffer for Source ID is still invalid, the setup of
            // buffer failed - the received data will be ignored.
            //
            if( snode_rx_data_id_p[ src_id ] != NULL )
            {
                snode_rx_data_id_p[ src_id ]->rx_time_us  = CoreTime32usGet();
                snode_rx_data_id_p[ src_id ]->vsense_data = vsense_data_msg;
            }
        }
    }
    
    // Read Node Status CAN message into module data while message or available
    // or 'SNODE_CTRL_DATA_MSG_MAX' message are read for the function's 
    // execution.
    rx_valid = true;
    for( ;
         ( msg_idx < SNODE_CTRL_DATA_MSG_MAX ) && ( rx_valid == true );
         msg_idx++ )
    {
        rx_valid = CANRxGet( CAN_RX_MSG_SNODE_STATUS,
                             &src_id,
                             &node_status_msg.data_u32[ 0 ] );

        // Data received ?
        if( rx_valid == true )
        {
            // Buffer for Source ID is not valid ?
            if( snode_rx_data_id_p[ src_id ] == NULL )
            {
                // Attempt to setup the buffer in module data.
                SNodeBufSetup( src_id );
            }
            
            // Buffer for Source ID is valid ?
            //
            // Note: if buffer for Source ID is still invalid, the setup of
            // buffer failed - the received data will be ignored.
            //
            if( snode_rx_data_id_p[ src_id ] != NULL )
            {
                snode_rx_data_id_p[ src_id ]->rx_time_us  = CoreTime32usGet();
                snode_rx_data_id_p[ src_id ]->node_status = node_status_msg;
            }
        }
    }
    
    // Read Node Version CAN message into module data while message or available
    // or 'SNODE_CTRL_DATA_MSG_MAX' message are read for the function's 
    // execution.
    rx_valid = true;
    for( ;
         ( msg_idx < SNODE_CTRL_DATA_MSG_MAX ) && ( rx_valid == true );
         msg_idx++ )
    {
        rx_valid = CANRxGet( CAN_RX_MSG_SNODE_VERSION,
                             &src_id,
                             &node_version_msg.data_u32[ 0 ] );

        // Data received ?
        if( rx_valid == true )
        {
            // Buffer for Source ID is not valid ?
            if( snode_rx_data_id_p[ src_id ] == NULL )
            {
                // Attempt to setup the buffer in module data.
                SNodeBufSetup( src_id );
            }
            
            // Buffer for Source ID is valid ?
            //
            // Note: if buffer for Source ID is still invalid, the setup of
            // buffer failed - the received data will be ignored.
            //
            if( snode_rx_data_id_p[ src_id ] != NULL )
            {
                snode_rx_data_id_p[ src_id ]->rx_time_us   = CoreTime32usGet();
                snode_rx_data_id_p[ src_id ]->node_version = node_version_msg;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Servo-node CAN data timeout.
///
/// Invalidate CAN data after 'SNODE_CTRL_DATA_TIMEOUT_US' has elapsed.
////////////////////////////////////////////////////////////////////////////////
static void SNodeCANTimeout( void )
{
    uint32_t psnt_time_us;
    uint8_t  id_idx;
    
    psnt_time_us = CoreTime32usGet();

    for( id_idx = 0;
         id_idx < 128;
         id_idx++ )
    {
        // Buffer for Source ID is valid ?
        if( snode_rx_data_id_p[ id_idx ] != NULL )
        {
            // Source ID hasn't received any data for the timeout ?
            if( psnt_time_us - snode_rx_data_id_p[ id_idx ]->rx_time_us > 
                    SNODE_CTRL_DATA_TIMEOUT_US )
            {
                // Identify buffer as being unused.
                snode_rx_data_id_p[ id_idx ]->valid = false;
                
                // Remove reference to buffer to identify Source ID buffer
                // as not established.
                snode_rx_data_id_p[ id_idx ] = NULL;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Servo-node control data build.
///
/// @param  eth_ctrl_data_msg
///             Pointer to store the construction Control Surface Data.
///
/// @return Number of Servo-node fields within the Control Surface Data 
///         packet.
///
/// Build the Control Surface Data Ethernet packet.
////////////////////////////////////////////////////////////////////////////////
static uint8_t SNodeCtrlDataBuild( FMUCOMM_CTRL_SURFACE_DATA_PL* eth_ctrl_data_msg )
{
    uint8_t surface_idx = 0;
    uint8_t id_idx;
    
    for( id_idx = 0;
         id_idx < 128;
         id_idx++ )
    {
        // Module data for Servo-Node ID is valid ?
        if( snode_rx_data_id_p[ id_idx ] != NULL )
        {
            // Copy the control surface data into the Ethernet packet.
            eth_ctrl_data_msg->ctrlSurface[ surface_idx ].surfaceID     = id_idx;
            eth_ctrl_data_msg->ctrlSurface[ surface_idx ].cmdTypeEcho   = snode_rx_data_id_p[ id_idx ]->servo_status.cmd_type_echo;
            eth_ctrl_data_msg->ctrlSurface[ surface_idx ].actPwm        = snode_rx_data_id_p[ id_idx ]->servo_status.pwm_act;
            eth_ctrl_data_msg->ctrlSurface[ surface_idx ].inputVoltage  = snode_rx_data_id_p[ id_idx ]->servo_status.servo_voltage;
            eth_ctrl_data_msg->ctrlSurface[ surface_idx ].inputCurrent  = snode_rx_data_id_p[ id_idx ]->servo_status.servo_current;
            eth_ctrl_data_msg->ctrlSurface[ surface_idx ].vsense1Cor    = snode_rx_data_id_p[ id_idx ]->vsense_data.vsense1_cor;
            eth_ctrl_data_msg->ctrlSurface[ surface_idx ].vsense2Cor    = snode_rx_data_id_p[ id_idx ]->vsense_data.vsense1_cor;;

            surface_idx++;
        }
    }
    
    return surface_idx;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Servo-node buffer setup.
///
/// @param  node_id
///             Servo-node ID element to setup within the module data buffer.
///
/// If the module data buffer is not already full, initialize the module data
/// buffer and module data ID buffer for reception of CAN data for the
/// supplied Servo-node ID.
////////////////////////////////////////////////////////////////////////////////
static void SNodeBufSetup( uint8_t node_id )
{
    uint8_t buf_idx;
            
    // Find first empty element in the buffer
    for( buf_idx = 0;
         buf_idx < SNODE_RX_BUF_SIZE;
         buf_idx++ )
    {
        if( snode_rx_data_buf[ buf_idx ].valid == false )
        {
            break;
        }
    }
    
    // Empty element in buffer found ?
    if( buf_idx < SNODE_RX_BUF_SIZE )
    {
        // Setup point to empty buffer element for the CAN node ID.
        snode_rx_data_id_p[ node_id ] = &snode_rx_data_buf[ buf_idx ];
        
        // Identify buffer as being used.
        snode_rx_data_buf[ buf_idx ].valid = true;
    }
}