////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Servo-node management.
////////////////////////////////////////////////////////////////////////////////

#ifndef SNODE_H_
#define SNODE_H_

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
// ************************** Macros *******************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

typedef enum
{
    SNODE_CAL_WRITE_SUCCESS,
    SNODE_CAL_WRITE_IN_PROG,
    SNODE_CAL_WRITE_FAIL,
            
} SNODE_CAL_WRITE_STATUS;

typedef enum
{
    SNODE_CAL_READ_SUCCESS,
    SNODE_CAL_READ_IN_PROG,
    SNODE_CAL_READ_FAIL,
            
} SNODE_CAL_READ_STATUS;

typedef enum
{
    SNODE_ID_WRITE_SUCCESS,
    SNODE_ID_WRITE_IN_PROG,
    SNODE_ID_WRITE_FAIL,
            
} SNODE_ID_WRITE_STATUS;

typedef struct 
{
    int32_t pwm_coeff[ 6 ];
    int32_t vsense1_coeff[ 6 ];
    int32_t vsense2_coeff[ 6 ];
    
} SNODE_CFG_VAL;

// *****************************************************************************
// ************************** Declarations *************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

////////////////////////////////////////////////////////////////////////////////
/// @brief  Perform the Servo-Node Task.
///
/// Manage Ethernet and CAN data for communication to/from the Servo-node
/// network.
////////////////////////////////////////////////////////////////////////////////
void SNodeTask( void );

////////////////////////////////////////////////////////////////////////////////
/// @brief  Set Servo-Node calibration write.
///
/// @param  node_id
///             The node to be written.
/// @param  cfg_val_p
///            The calibration values to write.
///
/// This function sets up a Servo-Node calibration writing process.
////////////////////////////////////////////////////////////////////////////////
void SNodeCalWriteSet( uint8_t node_id, SNODE_CFG_VAL* cfg_val_p );

////////////////////////////////////////////////////////////////////////////////
/// @brief  Set Servo-Node calibration read.
///
/// @param  node_id
///             The node to be read.
///
/// This function sets up a Servo-Node calibration reading process.
////////////////////////////////////////////////////////////////////////////////
void SNodeCalReadSet( uint8_t node_id );

////////////////////////////////////////////////////////////////////////////////
/// @brief  Set Servo-Node ID write.
///
/// @param  node_id_cur
///             The current value of the node ID to be written.
/// @param  node_id_new
///             The new ID value to update the node with.
///
/// This function sets up a Servo-Node ID writing process.
////////////////////////////////////////////////////////////////////////////////
void SNodeIDWriteSet( uint8_t node_id_cur, uint8_t node_id_new );

////////////////////////////////////////////////////////////////////////////////
/// @brief  Get Servo-Node calibration write status.
///
/// @return Status of the Servo-Node calibration writing process.
///
/// This function returns the status of the Servo-Node calibration writing 
/// process.  If the process does not get completed in a required time, then 
/// the status will be returned as \e FAIL.
////////////////////////////////////////////////////////////////////////////////
SNODE_CAL_WRITE_STATUS SNodeCalWriteStatusGet( void );

////////////////////////////////////////////////////////////////////////////////
/// @brief  Get Servo-Node calibration read status.
///
/// @return Status of the Servo-Node calibration read process.
///
/// This function returns the status of the Servo-Node calibration read 
/// process.  If the process does not get completed in a required time, then 
/// the status will be returned as \e FAIL.
////////////////////////////////////////////////////////////////////////////////
SNODE_CAL_READ_STATUS  SNodeCalReadStatusGet(  void );

////////////////////////////////////////////////////////////////////////////////
/// @brief  Get Servo-Node ID write status.
///
/// @return Status of the Servo-Node ID write process.
///
/// This function returns the status of the Servo-Node ID write process.  If
/// the process does not get completed in a required time, then the status
/// will be returned as \e FAIL.
////////////////////////////////////////////////////////////////////////////////
SNODE_ID_WRITE_STATUS  SNodeIDWriteStatusGet(  void );

////////////////////////////////////////////////////////////////////////////////
/// @brief  Get Servo-Node calibration read string.
///
/// @param  str_in
///             Buffer to save the calibration string.
///
/// This function returns string encoding of the read Servo-Node calibration
/// values.  Values are returned as a comma delimited string with calibration
/// value in order: pwm[0:6], vsense1[0:6], vsense2[0:6].
////////////////////////////////////////////////////////////////////////////////
void SNodeCalReadStrGet( char* str_in );

////////////////////////////////////////////////////////////////////////////////
/// @brief  Get Servo-Node received data string.
///
/// @param  str_in
///             Buffer to save the received Servo-Node data.
///
/// @note   This function provides a means of identifying the CAN Servo-Nodes
///         currently connected to the network.
///
/// This function returns string encoding of received Servo-Node data values.
/// Values are returned as a comma delimited string with data in order:
/// ID, PWM, ServoVolt, Vsense1, Vsense2.  The order of CAN node data is ordered
/// with an ascending ID value.
////////////////////////////////////////////////////////////////////////////////
void SNodeRxDataStrGet(  char* str_in );

#endif // SNODE_H_