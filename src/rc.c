////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Radio Control (RC) module.
////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

#include "rc.h"
#include "sbus.h"
#include "fmucomm.h"
#include "coretime.h"

// *****************************************************************************
// ************************** Macros *******************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

/// SBUS threshold value for detection of the RC-switch being active.
///
/// @note For a 3-state switch, S.Bus values for different states are:
///       (INACTIVE=172, MID=922, ACTIVE=1811).  A value between the MID and 
///       ACTIVE points is selected for the comparison.
///
#define RC_SBUS_SWITCH_ACT 1500

// Values received over S.Bus interface are in the range [172:1811]. These 
// values are mapped to the standard servo range [988:2012].
//
// Values are scaled by 1E4 to provide resolution in intermediate calculation.
// Final division removes this scaling.
//
#define RC_SERVO_MUL    6248U  ///< Servo PWM conversion multiplication factor.
#define RC_SERVO_ADD 8805400U  ///< Servo PWM conversion addition factor.
#define RC_SERVO_DIV   10000U  ///< Servo PWM conversion division factor.

/// The amount of time (in milliseconds) between successive transmission of
/// RC Ethernet data.
#define RC_SEND_TX_PERIOD 10000     // 10ms.

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

/// RC control flag used to identify if RC control is commanded.
static bool rc_ctrl = false;

/// RC commanded servo values.
static uint16_t rc_servo_val[ 10 ];

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

static void RCDecode( void );
static void RCSend( void );

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

void RCTask( void )
{
    RCDecode();
    RCSend();
}

bool RCCtrlGet( void )
{
    return rc_ctrl;
}

void RCServoGet( uint16_t servo_val[ 10 ] )
{
    memcpy( &servo_val[ 0 ], &rc_servo_val[ 0 ], sizeof( rc_servo_val ) );
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************

////////////////////////////////////////////////////////////////////////////////
/// @brief  Decode S.Bus data values into control values.
///
/// @note   The RC-Switch is required to be connected to S.Bus channel 1.
///
/// @note   The Servos are required to be connected to S.Bus channels 2-11.
///
/// This function determines the RC-switch state and servo PWM values based
/// on S.Bus data.
////////////////////////////////////////////////////////////////////////////////
static void RCDecode( void )
{
    uint16_t   ch_list[ SBUS_CH_MAX ];
    SBUS_CH_E  sbus_ch_idx;
    uint8_t    servo_idx = 0;
    
    // Get SBus channel data.
    SBusListGet( &ch_list[ 0 ] );
    
    // Decode RC-switch data.
    if( ch_list[ SBUS_CH1 ] > RC_SBUS_SWITCH_ACT )
    {
        rc_ctrl = true;
    }
    else
    {
        rc_ctrl = false;
    }
    
    // Decode Servo data.
    for( sbus_ch_idx = SBUS_CH2;
         sbus_ch_idx < SBUS_CH12;
         sbus_ch_idx++ )
    {
        // Convert the S.Bus values into Servo PWM values.
        rc_servo_val[ servo_idx ] = 
                (uint16_t) ( ( ch_list[ sbus_ch_idx ] * RC_SERVO_MUL + RC_SERVO_ADD ) / RC_SERVO_DIV );
        
        servo_idx++;
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief  Periodically send RC Ethernet data.
///
/// This function checks if the required time has elapsed since transmission
/// of the previous Ethernet packet.  If enough time has elapsed, a new
/// Ethernet packet is constructed and queued for transmission.
////////////////////////////////////////////////////////////////////////////////
static void RCSend( void )
{
    static FMUCOMM_CH_DATA_PL ch_data_pl;
    
    static uint32_t prev_tx_time = 0;
    static bool     tx_msg_ready = false;
    
    bool queue_ok;
    
    // Required time has elapsed since last Control Surface Data transmission ?
    if( CoreTime32usGet() - prev_tx_time > RC_SEND_TX_PERIOD )
    {
        // Increment tx-time by fixed transmission period time to eliminate
        // drift.
        prev_tx_time += RC_SEND_TX_PERIOD;
        
        // Transmission period is still already elapsed ?
        //
        // Note: This could occur if processing inhibited this function's 
        // execution for an extended amount of time.
        //
        if( CoreTime32usGet() - prev_tx_time > RC_SEND_TX_PERIOD )
        {
            // Update tx-time to the current time.  Single or multiple
            // periods have elapsed.  Setting tx-time to the current time
            // prevents repeated identifications of the period having elapsed.
            prev_tx_time = CoreTime32usGet();
        }
        
        // Get SBus channel data.
        SBusListGet( &ch_data_pl.chVal[ 0 ] );
        
        tx_msg_ready = true;
    }
    
    if( tx_msg_ready == true )
    {
        queue_ok = FMUCommSet( FMUCOMM_TYPE_RC_DATA,
                               (uint8_t*) &ch_data_pl, 
                               sizeof( ch_data_pl ) );
        
        if( queue_ok == true )
        {
            tx_msg_ready = false;
        }
    }
}