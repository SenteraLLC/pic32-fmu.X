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

void SNodeCalWriteSet( uint8_t node_id, SNODE_CFG_VAL* cfg_val_p );
void SNodeCalReadSet(  uint8_t node_id );
void SNodeIDWriteSet( uint8_t node_id_cur, uint8_t node_id_new );

SNODE_CAL_WRITE_STATUS SNodeCalWriteStatusGet( void );
SNODE_CAL_READ_STATUS  SNodeCalReadStatusGet(  void );
SNODE_ID_WRITE_STATUS  SNodeIDWriteStatusGet(  void );

void SNodeCalReadStrGet( char* str_in );
void SNodeRxDataStrGet(  char* str_in );


#endif // SNODE_H_