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
#include "rc.h"

// *****************************************************************************
// ************************** Macros *******************************************
// *****************************************************************************

// The number of Servo-Nodes available in the module data buffer.
#define SNODE_RX_BUF_SIZE 10

// Maximum number of CAN message read on each 'Task' execution.
#define SNODE_CTRL_DATA_MSG_MAX 4

// Number of micro-seconds after which an Servo-Node is invalidated, when
// fresh CAN data is not received.
#define SNODE_CTRL_DATA_TIMEOUT_US 1000000

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

typedef struct
{
    SNODE_ID_WRITE_STATUS status;         // Status/state of configuration write operation.
    uint8_t               curID;          // Current Node ID.
    uint8_t               newID;          // New/Updated Node ID.
    
} SNODE_WRITE_ID_DATA;

typedef struct
{
    SNODE_CAL_WRITE_STATUS  status;         // Status/state of configuration write operation.
    uint8_t                 id;             // CAN Node ID
    uint8_t                 cfg_sel;
    SNODE_CFG_VAL           cfg_val;
    
} SNODE_WRITE_CAL_DATA;

typedef struct
{
    SNODE_CAL_READ_STATUS status;         // Status/state of configuration write operation.
    uint8_t               id;             // CAN Node ID
    uint8_t               cfg_sel;
    SNODE_CFG_VAL         cfg_val;
    
} SNODE_READ_CAL_DATA;


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

SNODE_WRITE_ID_DATA snode_write_id_data = 
{
    .status = SNODE_ID_WRITE_SUCCESS,
};

SNODE_WRITE_CAL_DATA snode_write_cal_data = 
{
    .status = SNODE_CAL_WRITE_SUCCESS,
};

SNODE_READ_CAL_DATA snode_read_cal_data = 
{
    .status = SNODE_CAL_READ_SUCCESS,
};

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

static void SNodeCtrlDataTask( void );
static void SNodeCtrlCmdTask(  void );
static void SNodeCalWriteTask( void );
static void SNodeIDWriteTask(  void );
static void SNodeCalReadTask(  void );

static void SNodeCANRx( void );
static void SNodeCANTimeout( void );
static uint8_t SNodeCtrlDataBuild( FMUCOMM_CTRL_SURFACE_DATA_PL* eth_ctrl_data_msg );
static void SNodeBufSetup( uint8_t node_id );

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

void SNodeTask( void )
{
    // Host <-> FMU <-> Servo-Node communication tasks.
    SNodeCtrlDataTask();
    SNodeCtrlCmdTask();
    
    // Web <-> FMU <-> Servo-Node communication tasks.
    SNodeIDWriteTask();
    SNodeCalWriteTask();
    SNodeCalReadTask();
}

// Setup for a Servo-Node calibration writing process.
void SNodeCalWriteSet( uint8_t node_id, SNODE_CFG_VAL* cfg_val_p )
{
    snode_write_cal_data.status  = SNODE_CAL_WRITE_IN_PROG;
    snode_write_cal_data.id      = node_id;
    snode_write_cal_data.cfg_sel = 1;
    snode_write_cal_data.cfg_val = *cfg_val_p;
}

// Setup for a Servo-Node calibration reading process.
void SNodeCalReadSet( uint8_t node_id )
{
    snode_read_cal_data.status  = SNODE_CAL_READ_IN_PROG;
    snode_read_cal_data.id      = node_id;
    snode_read_cal_data.cfg_sel = 1;
}

// Setup for a Servo-Node ID writing process.
void SNodeIDWriteSet( uint8_t node_id_cur, uint8_t node_id_new )
{
    snode_write_id_data.status = SNODE_ID_WRITE_IN_PROG;
    snode_write_id_data.curID  = node_id_cur;
    snode_write_id_data.newID  = node_id_new;
}

// Get the results of the Servo-Node ID writing process.
SNODE_ID_WRITE_STATUS SNodeIDWriteStatusGet( void )
{
    return snode_write_id_data.status;
}

// Get the results of the Servo-Node calibration writing process.
SNODE_CAL_WRITE_STATUS SNodeCalWriteStatusGet( void )
{
    return snode_write_cal_data.status;
}

// Get the results of the Servo-Node calibration reading process.
SNODE_CAL_READ_STATUS SNodeCalReadStatusGet( void )
{
    return snode_read_cal_data.status;
}

// construct the string directly for returning received data.
// NOTE: calibration read values are packed as comma
// delimited string.
//
void SNodeCalReadStrGet( char* str_in )
{
    uint8_t coeff_idx;
    
    char* init_str = "";
    char* del_str  = ",";
    
    char elem_str[20];
    
    // Initialize the input string.
    strcpy(str_in, init_str);
    
    // Append string with PWM coefficients.
    for( coeff_idx = 0;
         coeff_idx < 6;
         coeff_idx++ )
    {
        if( snode_read_cal_data.status == SNODE_CAL_READ_SUCCESS )
        {
            itoa(elem_str, snode_read_cal_data.cfg_val.pwm_coeff[ coeff_idx ], 10);
            strcat(str_in, elem_str);
        }

        // Add comma delimiter.
        strcat(str_in, del_str); 
    }
    
    // Append string with VSENSE1 coefficients.
    for( coeff_idx = 0;
         coeff_idx < 6;
         coeff_idx++ )
    {
        if( snode_read_cal_data.status == SNODE_CAL_READ_SUCCESS )
        {
            itoa(elem_str, snode_read_cal_data.cfg_val.vsense1_coeff[ coeff_idx ], 10);
            strcat(str_in, elem_str);
        }

        // Add comma delimiter.
        strcat(str_in, del_str); 
    }
    
    // Append string with VSENSE2 coefficients.
    for( coeff_idx = 0;
         coeff_idx < 6;
         coeff_idx++ )
    {
        if( snode_read_cal_data.status == SNODE_CAL_READ_SUCCESS )
        {
            itoa(elem_str, snode_read_cal_data.cfg_val.vsense2_coeff[ coeff_idx ], 10);
            strcat(str_in, elem_str);
        }

        // Add comma delimiter.
        strcat(str_in, del_str); 
    }
}

// construct the string directly for returning received data.
// NOTE: calibration read values are packed as comma
// delimited string.
//
void SNodeRxDataStrGet( char* str_in )
{
    uint8_t surface_idx = 0;
    uint8_t id_idx;
    
    char* del_str = ",";
    char elem_str[20];
    
    // Determine the number of Servo-Nodes on the network.
    for( id_idx = 0;
         id_idx < 128;
         id_idx++ )
    {
        // Module data for Servo-Node ID is valid ?
        if( snode_rx_data_id_p[ id_idx ] != NULL )
        {
            surface_idx++;
        }
    }
    
    // Add number of CAN Servo-Nodes as first element.
    utoa(elem_str, (uint32_t) surface_idx, 10);
    strcpy(str_in, elem_str);

    // Add comma delimiter.
    strcat(str_in, del_str);
    
    // Loop through the Servo-nodes on the network, populating their data.
    for( id_idx = 0;
         id_idx < 128;
         id_idx++ )
    {
        // Module data for Servo-Node ID is valid ?
        if( snode_rx_data_id_p[ id_idx ] != NULL )
        {
            // Add 'ID'.
            utoa(elem_str, id_idx, 10);
            strcat(str_in, elem_str);
            
            // Add comma delimiter.
            strcat(str_in, del_str); 
            
            // Add 'actPwm'.
            utoa(elem_str, snode_rx_data_id_p[ id_idx ]->servo_status.pwm_act, 10);
            strcat(str_in, elem_str);
            
            // Add comma delimiter.
            strcat(str_in, del_str); 
            
            // Add 'servoVoltage'.
            utoa(elem_str, snode_rx_data_id_p[ id_idx ]->servo_status.servo_voltage, 10);
            strcat(str_in, elem_str);
            
            // Add comma delimiter.
            strcat(str_in, del_str); 
            
            // Add 'vsense1Cor'.
            itoa(elem_str, snode_rx_data_id_p[ id_idx ]->vsense_data.vsense1_cor, 10);
            strcat(str_in, elem_str);
            
            // Add comma delimiter.
            strcat(str_in, del_str); 
            
            // Add 'vsense2Cor'.
            itoa(elem_str, snode_rx_data_id_p[ id_idx ]->vsense_data.vsense2_cor, 10);
            strcat(str_in, elem_str);
            
            // Add comma delimiter.
            strcat(str_in, del_str); 
        }
    }
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
    
    static FMUCOMM_CTRL_SURFACE_DATA_PL eth_ctrl_data_msg;
    
    static uint32_t prev_tx_time   = 0;
    static bool     tx_msg_ready   = false;
    static uint8_t  surface_num_of = 0;

    // Required time has elapsed since last Control Surface Data transmission ?
    if( CoreTime32usGet() - prev_tx_time > 10000 )
    {
        // Increment tx-time by fixed transmission period time to eliminate
        // drift.
        prev_tx_time += 10000;
        
        // Transmission period is still already elapsed ?
        //
        // Note: This could occur if processing inhibited this function's 
        // execution for an extended amount of time.
        //
        if( CoreTime32usGet() - prev_tx_time > 10000 )
        {
            // Update tx-time to the current time.  Single or multiple
            // periods have elapsed.  Setting tx-time to the current time
            // prevents repeated identifications of the period having elapsed.
            prev_tx_time = CoreTime32usGet();
        }
        
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
        bool queue_ok;
        
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
    
    bool rc_ctrl;
    uint16_t rc_servo_val[ 10 ];
    
    rc_ctrl = RCCtrlGet();
    
    // RC commanded control to be performed ?
    if( rc_ctrl == true )
    {
        RCServoGet( &rc_servo_val[ 0 ] );
        
        snodes_max = 10;
        // Loop through each node's information.
        for( snodes_idx = 0; snodes_idx < snodes_max; snodes_idx++ )
        {
            // Construct the CAN message to send with the received RC
            // data.
            can_ctrl_cmd.cmd_type = 0; // PWM control
            can_ctrl_cmd.cmd_pwm  = rc_servo_val[ snodes_idx ];
            can_ctrl_cmd.cmd_pos  = 0; // N/A

            // Forward the command to the Servo-node.
            CANTxSet( CAN_TX_MSG_SERVO_CMD, 
                      snodes_idx + 2,
                      &can_ctrl_cmd.data_u32[ 0 ] );
        }
    }
    else // Host commanded control is to be performed.
    {
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
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Servo-node write identification task.
///
/// Once the new ID has been received, this function writes the ID out to the 
/// Servo-node CAN network.
////////////////////////////////////////////////////////////////////////////////
static void SNodeIDWriteTask( void )
{
    static enum
    {            
        SM_IDLE,
        SM_WRITE_REQ,     
        SM_DELAY,    
        SM_READ_RESP,
                
    } taskState = SM_IDLE;
    
    switch(taskState)
    {
        case SM_IDLE:
        {
            // Calibration write operation to be performed ?
            if( snode_write_id_data.status == SNODE_ID_WRITE_IN_PROG )
            {
                // Kickoff the writing process.
                taskState = SM_WRITE_REQ;
            }
            
            break;
        }
        case SM_WRITE_REQ:
        {
            CAN_TX_WRITE_REQ_U write_req;
            
            // Build the CAN message to transmit.
            write_req.cfg_sel    = 0;                           // Node ID selection.
            write_req.cfg_val_u8 = snode_write_id_data.newID;   // New node ID value.
            
            // Queue the CAN message for transmission.
            CANTxSet( CAN_TX_MSG_CFG_WRITE_REQ, 
                      snode_write_id_data.curID, 
                      &write_req.data_u32[0] );
            
            taskState++;
            
            break;
        }
        // Wait at least 100ms for the CAN node to process the message
        // and sent the response back.
        case SM_DELAY:
        {
            static uint32_t delayStart = 0;
            
            // Initialize delay state on first evaluation.
            if( delayStart == 0 )
            {
                delayStart = CoreTime32usGet();
            }
            
            // Required time (i.e. 100ms) has elapsed ?
            if( CoreTime32usGet() - delayStart > 100000 )
            {
                taskState++;
                
                // Reset the start time for evaluation on next delay.
                delayStart = 0;
            }
            
            break;
        }
        case SM_READ_RESP:
        {
            CAN_RX_WRITE_RESP_U write_resp;
            bool                rx_valid;
            uint8_t             rx_node_id;
            
            // Default the ID write operation status to failed.
            snode_write_id_data.status = SNODE_ID_WRITE_FAIL;
            
            // Read the response back.
            rx_valid = CANRxGet( CAN_RX_MSG_CFG_WRITE_RESP, 
                                 &rx_node_id,
                                 &write_resp.data_u32[0] );
            
            // Message received/valid ?
            if( rx_valid == true )
            {
                // - Message's node ID matches that expected ?
                // - Updated value matches that selected ?
                // - Response identifies success ?
                // 
                if( ( snode_write_id_data.newID == rx_node_id ) &&
                    ( write_resp.cfg_sel        == 0          ) &&
                    ( write_resp.fault_status   == 0          ) )
                {
                    snode_write_id_data.status = SNODE_ID_WRITE_SUCCESS;
                }
            }
            
            // ID operation completed, return to the idle state.
            taskState = SM_IDLE;
            
            break;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Servo-node write calibration task.
///
/// Once calibration data has been received, this function writes the data
/// out to the Servo-node CAN network.
////////////////////////////////////////////////////////////////////////////////
static void SNodeCalWriteTask( void )
{
    static enum
    {            
        SM_IDLE,
        SM_WRITE_REQ,     
        SM_DELAY,    
        SM_WRITE_RESP,
        SM_DONE_EVAL,
                
    } taskState = SM_IDLE;
    
    switch(taskState)
    {
        case SM_IDLE:
        {
            // Calibration write operation to be performed ?
            if( snode_write_cal_data.status == SNODE_CAL_WRITE_IN_PROG )
            {
                // Kickoff the writing process.
                taskState = SM_WRITE_REQ;
            }
            
            break;
        }
        case SM_WRITE_REQ:
        {
            CAN_TX_WRITE_REQ_U write_req;
            
            // Build the CAN message to transmit.
            write_req.cfg_sel = snode_write_cal_data.cfg_sel;
            
            // PWM coefficient value to be transmitted ?
            if( snode_write_cal_data.cfg_sel <= 6 )
            {
                write_req.cfg_val_i32 = snode_write_cal_data.cfg_val.pwm_coeff[ snode_write_cal_data.cfg_sel - 1 ];
            }
            // VSENSE1 coefficient value to be transmitted ?
            else
            if(snode_write_cal_data.cfg_sel <= 12 )
            {
                write_req.cfg_val_i32 = snode_write_cal_data.cfg_val.vsense1_coeff[ snode_write_cal_data.cfg_sel - 7 ];
            }
            // VSENSE2 coefficient value to be transmitted.
            else
            {
                write_req.cfg_val_i32 = snode_write_cal_data.cfg_val.vsense2_coeff[ snode_write_cal_data.cfg_sel - 13 ];
            }
            
            // Queue the CAN message for transmission.
            CANTxSet( CAN_TX_MSG_CFG_WRITE_REQ, 
                      snode_write_cal_data.id,
                      &write_req.data_u32[0] );
            
            taskState++;
            
            break;
        }
        // Wait at least 100ms for the CAN node to process the message
        // and sent the response back.
        case SM_DELAY:
        {
            static uint32_t delayStart = 0;
            
            // Initialize delay state on first evaluation.
            if( delayStart == 0 )
            {
                delayStart = CoreTime32usGet();
            }
            
            // Required time (i.e. 100ms) has elapsed ?
            if( CoreTime32usGet() - delayStart > 100000 )
            {
                taskState++;
                
                // Reset the start time for evaluation on next delay.
                delayStart = 0;
            }
            
            break;
        }
        case SM_WRITE_RESP:
        {
            CAN_RX_WRITE_RESP_U write_resp;
            bool                rx_valid;
            uint8_t             rx_node_id;
            
            // Read the response back.
            rx_valid = CANRxGet( CAN_RX_MSG_CFG_WRITE_RESP, 
                                 &rx_node_id,
                                 &write_resp.data_u32[0] );
            
            // - Response received/valid ?
            // - Response node ID matches that expected ?
            // - Updated value matches that selected ?
            // - Response identifies success ?
            // 
            if( ( rx_valid                  == true                         ) &&
                ( snode_write_cal_data.id   == rx_node_id                   ) &&
                ( write_resp.cfg_sel        == snode_write_cal_data.cfg_sel ) &&
                ( write_resp.fault_status   == 0                            ) )
            {
                // Update configuration selection to program the next
                // parameter (or identify completion of write process).
                snode_write_cal_data.cfg_sel++;
            }
            else
            {
                // Expected response not received - fail the write operation.
                snode_write_cal_data.status = SNODE_CAL_WRITE_FAIL;
            }
            
            taskState++;
            
            break;
        }
        case SM_DONE_EVAL:
        {
            // Failure with Calibration write operation ?
            if( snode_write_cal_data.status == SNODE_CAL_WRITE_FAIL )
            {
                taskState = SM_IDLE;
            }
            else
            {
                // Calibration is complete ?
                if( snode_write_cal_data.cfg_sel > 18 )
                {
                    // Identify success since all calibration values have been
                    // successfully written.
                    snode_write_cal_data.status = SNODE_CAL_WRITE_SUCCESS;
                    
                    taskState = SM_IDLE;
                }
                else
                {
                    // Still more calibration values to write - continue
                    // with the next request/response sequence.
                    taskState = SM_WRITE_REQ;
                }
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Servo-node read calibration task.
///
/// Once a read operation has been commanded, this function reads the data
/// from to the Servo-node CAN network.
////////////////////////////////////////////////////////////////////////////////
static void SNodeCalReadTask( void )
{
    static enum
    {            
        SM_IDLE,
        SM_READ_REQ,     
        SM_DELAY,    
        SM_READ_RESP,
        SM_DONE_EVAL,
                
    } taskState = SM_IDLE;
    
    switch(taskState)
    {
        case SM_IDLE:
        {
            // Calibration read operation to be performed ?
            if( snode_read_cal_data.status == SNODE_CAL_READ_IN_PROG )
            {
                // Kickoff the writing process.
                taskState = SM_READ_REQ;
            }
            
            break;
        }
        case SM_READ_REQ:
        {
            CAN_TX_READ_REQ_U read_req;
            
            // Build the CAN message to transmit.
            read_req.cfg_sel = snode_read_cal_data.cfg_sel;
            
            // Queue the CAN message for transmission.
            CANTxSet( CAN_TX_MSG_CFG_READ_REQ, 
                      snode_read_cal_data.id,
                      &read_req.data_u32[0] );
            
            taskState++;
            
            break;
        }
        // Wait at least 100ms for the CAN node to process the message
        // and sent the response back.
        case SM_DELAY:
        {
            static uint32_t delayStart = 0;
            
            // Initialize delay state on first evaluation.
            if( delayStart == 0 )
            {
                delayStart = CoreTime32usGet();
            }
            
            // Required time (i.e. 100ms) has elapsed ?
            if( CoreTime32usGet() - delayStart > 100000 )
            {
                taskState++;
                
                // Reset the start time for evaluation on next delay.
                delayStart = 0;
            }
            
            break;
        }
        case SM_READ_RESP:
        {
            CAN_RX_READ_RESP_U read_resp;
            bool               rx_valid;
            uint8_t            rx_node_id;
            
            // Read the response back.
            rx_valid = CANRxGet( CAN_RX_MSG_CFG_READ_RESP, 
                                 &rx_node_id,
                                 &read_resp.data_u32[0] );
            
            // Save the read value to module data.
            if( snode_read_cal_data.cfg_sel <= 6 )  // PWM
            {
                snode_read_cal_data.cfg_val.pwm_coeff[ snode_read_cal_data.cfg_sel - 1 ] = read_resp.cfg_val_i32;
            }
            else
            if(snode_read_cal_data.cfg_sel <= 12 )  // VSENSE1
            {
                snode_read_cal_data.cfg_val.vsense1_coeff[ snode_read_cal_data.cfg_sel - 7 ] = read_resp.cfg_val_i32;
            }
            else    // VSENSE2
            {
                snode_read_cal_data.cfg_val.vsense2_coeff[ snode_read_cal_data.cfg_sel - 13 ] = read_resp.cfg_val_i32;
            }
            
            // - Response received/valid ?
            // - Response node ID matches that expected ?
            // - Updated value matches that selected ?
            // 
            if( ( rx_valid                == true                        ) &&
                ( snode_read_cal_data.id  == rx_node_id                  ) &&
                ( read_resp.cfg_sel       == snode_read_cal_data.cfg_sel ) )
            {
                // Update configuration selection to program the next
                // parameter (or identify completion of read process).
                snode_read_cal_data.cfg_sel++;
            }
            else
            {
                // Expected response not received - fail the read operation.
                snode_read_cal_data.status = SNODE_CAL_READ_FAIL;
            }
            
            taskState++;
            
            break;
        }
        case SM_DONE_EVAL:
        {
            // Failure with Calibration read operation ?
            if( snode_read_cal_data.status == SNODE_CAL_READ_FAIL )
            {
                taskState = SM_IDLE;
            }
            else
            {
                // Calibration is complete ?
                if( snode_read_cal_data.cfg_sel > 18 )
                {
                    // Identify success since all calibration values have been
                    // successfully read.
                    snode_read_cal_data.status = SNODE_CAL_READ_SUCCESS;
                    
                    taskState = SM_IDLE;
                }
                else
                {
                    // Still more calibration values to read - continue
                    // with the next request/response sequence.
                    taskState = SM_READ_REQ;
                }
            }
            
            break;
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
            eth_ctrl_data_msg->ctrlSurface[ surface_idx ].vsense2Cor    = snode_rx_data_id_p[ id_idx ]->vsense_data.vsense1_cor;

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