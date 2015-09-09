////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Monitor Host status and annunciate FMU status.
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
#include "can.h"

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

// The node type of the FMU. 0 = FMU, 1 = Servo-Node.
#define STATUS_FMU_NODE_TYPE 0

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

/// Structure defining space for storage of a serial number.
///
/// @note The tool used for programming the serial number has a minimum flash
///       size (0x200) for preserving flash memory (i.e. a flash row size).  
///       The serial number is spare padded to this minimum required size to
///       interface with the tool.
///
typedef struct
{
    uint32_t val;
    
    uint8_t spare[ 508 ];
    
} STATUS_SERIAL_NUM_S;

/// The node type - value of '0' identifies node as a FMU node.
static const uint8_t  status_node_type  = 0;

static const uint8_t  status_fw_rev_ver = 1;    ///< Firmware revision version number.
static const uint8_t  status_fw_min_ver = 0;    ///< Firmware minor version number.
static const uint8_t  status_fw_maj_ver = 0;    ///< Firmware major version number.

static const uint8_t  status_hw_rev_ver = 0;    ///< Hardware revision version number.
static const uint8_t  status_hw_min_ver = 0;    ///< Hardware minor version number.
static const uint8_t  status_hw_maj_ver = 1;    ///< Hardware major version number.

/// The serial number - set during initial programming.
///
/// @note The serial number is set to the starting address of Program
///       memory.
///
static const STATUS_SERIAL_NUM_S __attribute__((space(prog), address(0x9D000000))) status_serial_num;

/// Data from Host Heartbeat Ethernet message.
static FMUCOMM_HOST_HEARTBEAT_PL status_host_hb_data;

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

static void StatusLEDTask( void );
static void StatusFMUHbTask( void );
static void StatusFMUNodeTask( void );
static void StatusHostHbTask( void );

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

void StatusTask( void )
{
    StatusLEDTask();
    StatusFMUHbTask();
    StatusFMUNodeTask();
    StatusHostHbTask();
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
static void StatusFMUHbTask( void )
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
            
            fmu_heartbeat_pl.serialNum    = status_serial_num.val;
            
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

// Periodically send the Node Status and Node Version CAN messages.
static void StatusFMUNodeTask( void )
{
    static uint32_t prevExeTime = 0;
    
    CAN_TX_NODE_STATUS_U node_status_msg;
    CAN_TX_NODE_VER_U    node_ver_msg;
    
    // Required time has elapsed ?
    if( CoreTime32usGet() - prevExeTime > 500000 )
    {
        // Increment by fixed transmission period time to eliminate
        // drift.
        prevExeTime += 500000;

        // Still identified that require time has elapsed ?
        //
        // Note: This could occur if processing inhibited this function's 
        // execution for an extended amount of time.
        //
        if( CoreTime32usGet() - prevExeTime > 500000 )
        {
            // Update to the current time.  Single or multiple timeouts
            // have elapsed.  Updating to the current time prevents
            // repeated identifications of timeout having elapsed.
            prevExeTime = CoreTime32usGet();
        }

        
        // NODE STATUS MESSAGE -------------------------------------------------
        
        // Build the message.
        //
        // Note: RCON register truncated to 16-bits since upper 16-bits is all
        // spares.
        node_status_msg.reset_detail = (uint16_t) RCON;
        
        if( RCONbits.POR )
        {
            node_status_msg.reset_condition = 1;
        }
        else
        if( RCONbits.BOR )
        {
            node_status_msg.reset_condition = 2;
        }
        else
        if( RCONbits.SWR )
        {
            node_status_msg.reset_condition = 3;
        }
        else
        {
            node_status_msg.reset_condition = 4;
        }
        
        // Reset the RCON register BOR and POR bits so that the reset condition
        // can be detected on the next reset.
        RCONCLR = _RCON_BOR_MASK;
        RCONCLR = _RCON_POR_MASK;
        
        // Queue message for transmission.
        //
        // Note: destination ID set as zero b/c it is not applicable for the
        // node status message which is a broadcast message.
        //
        CANTxSet( CAN_TX_MSG_NODE_STATUS, 
                  0,
                  &node_status_msg.data_u32[ 0 ] );
        
        
        // NODE VERSION MESSAGE ------------------------------------------------
        
        // Build the message.
        node_ver_msg.node_type  = STATUS_FMU_NODE_TYPE;
        node_ver_msg.rev_ver    = status_fw_rev_ver;
        node_ver_msg.min_ver    = status_fw_min_ver;
        node_ver_msg.maj_ver    = status_fw_maj_ver;
        node_ver_msg.serial_num = status_serial_num.val;
        
        // Queue message for transmission.
        //
        // Note: destination ID set as zero b/c it is not applicable for the
        // node version message which is a broadcast message.
        //
        CANTxSet( CAN_TX_MSG_NODE_VER, 
                  0,
                  &node_ver_msg.data_u32[ 0 ] );
    }
}

// Read the Host heartbeat message into module data.
static void StatusHostHbTask( void )
{
    const FMUCOMM_RX_PKT*            host_hb_pkt;
          FMUCOMM_HOST_HEARTBEAT_PL* host_hb_pl;
          
    host_hb_pkt = FMUCommGet( FMUCOMM_TYPE_HOST_HEARTBEAT );
    
    // valid Host Heartbeat message received ?
    if( host_hb_pkt->valid == true )
    {
        host_hb_pl = (FMUCOMM_HOST_HEARTBEAT_PL*) host_hb_pkt->pl_p;
        
        // Copy message payload into module data.
        status_host_hb_data = *host_hb_pl;
    }
}
