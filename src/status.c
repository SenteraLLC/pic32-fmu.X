////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Annunciate FMU Status.
////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

#include "status.h"
#include "fmucomm.h"
#include "coretime.h"
#include "adc.h"
#include "emc1412.h"

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

#define STATUS_FW_VERSION 0
#define STATUS_HW_VERSION 0
#define STATUS_SERIAL_NUM 0

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

static void StatusLEDTask( void );
static void StatusPktTask( void );

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

void StatusTask( void )
{
    StatusLEDTask();
    
    StatusPktTask();
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************

// Blink the heartbeat LED at a 0.5Hz rate.
static void StatusLEDTask( void )
{
    static uint32_t prevBlinkTime = 0;
    
    // 1.0s elapsed since last blink ?
    if( CoreTime32usGet() - prevBlinkTime > 1000000 )
    {
        // Toggle the LED state.
        LATFbits.LATF3 ^= 1;
        
        // Latch the execution time for evaluation of next blink.
        prevBlinkTime = CoreTime32usGet();
    }
    
    return;
}

// Transmit the FMU Heartbeat packet at a 1Hz rate.
static void StatusPktTask( void )
{
    static enum 
    {
        SM_TX,
        SM_DELAY,
            
    } statusPktState = SM_TX;
    
    switch (statusPktState)
    {
        case SM_TX:
        {
            FMUCOMM_FMU_HEARTBEAT_PL fmu_heartbeat_pl;
            bool setSuccess;
            
            // Populate FMU Heartbeat Packet information.
            fmu_heartbeat_pl.fwVersion    = STATUS_FW_VERSION;
            fmu_heartbeat_pl.hwVersion    = STATUS_HW_VERSION;
            fmu_heartbeat_pl.serialNum    = STATUS_SERIAL_NUM;
            fmu_heartbeat_pl.msUptime     = CoreTime32msGet();
            fmu_heartbeat_pl.inputVoltage = ADCVbattGet();
            fmu_heartbeat_pl.boardTemp    = EMC1412IntTempGet();
            
            // Queue message for transmission.
            setSuccess = FMUCommSet( FMUCOMM_TYPE_FMU_HEARTBEAT,
                                     (uint8_t*) &fmu_heartbeat_pl,
                                     sizeof( fmu_heartbeat_pl ) );
            
            // Message successfully queued for transfer ?
            //
            // If queuing for transfer unsuccessful, the state machine is not
            // updated and queuing will be attempted again.
            //
            if( setSuccess == true )
            {
                statusPktState++;
            }
            
            break;
        }
        case SM_DELAY:
        {
            static uint32_t prevExeTime = 0;
            
            // Required time has elapsed ?
            //
            // Delay required amount of time so that a cycle of the Task
            // Execution is performed every ~1s. This is to accomplish an
            // FMU Heartbeat annunciation on LAN at 1Hz.
            //
            if( CoreTime32usGet() - prevExeTime > 1000000 )
            {
                // Perform another task cycle.
                statusPktState = SM_TX;
                
                // Latch the execution time for evaluation on next delay.
                prevExeTime = CoreTime32usGet();
            }
            
            break;
        }
    }

    return;
}


