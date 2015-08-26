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

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

/// The node type - value of '0' identifies node as a FMU node.
static const uint8_t  status_node_type  = 0;

static const uint8_t  status_fw_rev_ver = 0;    ///< Firmware revision version number.
static const uint8_t  status_fw_min_ver = 0;    ///< Firmware minor version number.
static const uint8_t  status_fw_maj_ver = 0;    ///< Firmware major version number.

static const uint8_t  status_hw_rev_ver = 0;    ///< Hardware revision version number.
static const uint8_t  status_hw_min_ver = 0;    ///< Hardware minor version number.
static const uint8_t  status_hw_maj_ver = 0;    ///< Hardware major version number.

/// The serial number - set during manufacturing.
static const uint32_t status_serial_num = 0;

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
        LATFINV = _LATF_LATF3_MASK;
        
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
            fmu_heartbeat_pl.fwVersionRev = status_fw_rev_ver;
            fmu_heartbeat_pl.fwVersionMin = status_fw_min_ver;       
            fmu_heartbeat_pl.fwVersionMaj = status_fw_maj_ver;    
                    
            fmu_heartbeat_pl.hwVersionRev = status_hw_rev_ver;
            fmu_heartbeat_pl.hwVersionMin = status_hw_min_ver;       
            fmu_heartbeat_pl.hwVersionMaj = status_hw_maj_ver;  
            
            fmu_heartbeat_pl.serialNum    = status_serial_num;
            
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
            if( CoreTime32usGet() - prevExeTime > 1000000 )
            {
                // Increment by fixed transmission period time to eliminate
                // drift.
                prevExeTime += 1000000;

                // Still identified that require time has elapsed ?
                //
                // Note: This could occur if processing inhibited this function's 
                // execution for an extended amount of time.
                //
                if( CoreTime32usGet() - prevExeTime > 1000000 )
                {
                    // Update to the current time.  Single or multiple timeouts
                    // have elapsed.  Updating to the current time prevents
                    // repeated identifications of timeout having elapsed.
                    prevExeTime = CoreTime32usGet();
                }

                // Perform another task cycle.
                statusPktState = SM_TX;
            }
            
            break;
        }
    }

    return;
}


