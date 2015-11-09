////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Vector Navigation (VN-100) driver.
////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

#include "vn100.h"
#include "coretime.h"
#include "spi.h"
#include "fmucomm.h"
#include "util.h"

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

// The type of IMU being read. '0' identifies a VN-100.
#define VN100_IMU_TYPE  0U

// Size (in bytes) of the buffer used to holding request and receive packets.
// The size is chosen to be larger than any packet size required for transfer.
// Size includes the Header, Payload, and Footer (e.g. CRC) fields.
#define VN100_XFER_BUF_SIZE 100

// VN-100 command values for a Read and Write register operation.
#define VN100_READ_REG_CMD  1
#define VN100_WRITE_REG_CMD 2

// VN100 SPI Read Register
//  (Request)
//      - Header:        4 bytes
//      - Payload:       0 bytes
//      - CRC:           2 bytes
//  (Response)
//      - Header:        4 bytes
//      - Payload:       Variable size based on read register
//      - Serial Status: 2 bytes
//      - Serial Count:  2 bytes
//      - CRC:           2 bytes
//
// VN100 SPI Write Register
//  (Request)
//      - Header:        4 bytes
//      - Payload:       Variable size
//      - CRC:           2 bytes
//  (Response)
//      - Header:        4 bytes
//      - Payload:       Variable size (same as request payload)
//      - Serial Status: 2 bytes
//      - Serial Count:  2 bytes
//      - CRC:           2 bytes
// 
#define VN100_SPI_HEADER_SIZE 4
#define VN100_SPI_CRC_SIZE    2

#define mVN100_SPI_PKT_SIZE_NOCRC(x) ( VN100_SPI_HEADER_SIZE    + (x)                )

#define mVN100_SPI_CRC_OFFSET(x)     ( VN100_SPI_HEADER_SIZE    + (x)                )
#define mVN100_SPI_PKT_SIZE(x)       ( mVN100_SPI_CRC_OFFSET(x) + VN100_SPI_CRC_SIZE )


typedef union
{
    struct
    {
        uint16_t AttitudeQuality         : 2;    // bits  0 -  1, Provides an indication of the quality of the attitude solution.
        uint16_t GyroSaturation          : 1;    // bits       2, At least one gyro axis is currently saturated.
        uint16_t GyroSaturationRecovery  : 1;    // bits       3, Filter is in the process of recovering from a gyro saturation event.
        uint16_t MagDisturbance          : 2;    // bits  4 -  5, A magnetic DC disturbance has been detected. (0 ? No magnetic disturbance, 1 to 3 ? Magnetic disturbance is present)
        uint16_t MagSaturation           : 1;    // bits       6, At least one magnetometer axis is currently saturated.
        uint16_t AccDisturbance          : 2;    // bits  7 -  8, A strong acceleration disturbance has been detected. (0 ? No acceleration disturbance, 1 to 3 ? Acceleration disturbance has been detected)
        uint16_t AccSaturation           : 1;    // bits       9, At least one accelerometer axis is currently saturated.
        uint16_t                         : 1;    // bits      10, (Reserved)
        uint16_t KnownMagDisturbance     : 1;    // bits      11, A known magnetic disturbance has been reported by the user and the magnetometer is currently tuned out.
        uint16_t KnownAccelDisturbance   : 1;    // bits      12, A known acceleration disturbance has been reported by the user and the accelerometer is currently tuned out.
        uint16_t                         : 3;    // bits 13 - 15, (Reserved)
    };
    
    uint16_t valU16;
    
} VN100_VPE_STATUS;

typedef struct
{
    uint8_t     cmdID;
    uint8_t     arg;
    
    uint8_t*    reqPl;
    uint16_t    reqPlLen;
    
    uint8_t*    respPl;
    uint16_t    respPlLen;
    
    bool        crcComp;
    bool        commSuccess;
    
} VN100_COMM_PKT;

// Communication Protocol Payload when writing uninitialized VN100.
typedef struct __attribute__ ((packed))
{
    uint8_t  serialCount;
    uint8_t  serialStatus;
    uint8_t  spiCount;
    uint8_t  spiStatus;
    uint8_t  serialChecksum;
    uint8_t  spiChecksum;
    uint8_t  errorMode;
    
}VN100_COMM_PROTO_WRITE_PL;

// Communication Protocol Payload when reading initialized VN100.
typedef struct __attribute__ ((packed))
{
    uint8_t          serialCount;
    uint8_t          serialStatus;
    uint8_t          spiCount;
    uint8_t          spiStatus;
    uint8_t          serialChecksum;
    uint8_t          spiChecksum;
    uint8_t          errorMode;
    
    VN100_VPE_STATUS vpeStatus;         // Vector Processing Engine Status.
    uint32_t         syncInTime;        // Time since last SyncIn trigger.
    
}VN100_COMM_PROTO_READ_PL;

// IMU Measurement Payload.
typedef struct __attribute__ ((packed))
{
    float            magX;              // Uncompensated magnetic X-axis.
    float            magY;              // Uncompensated magnetic Y-axis.
    float            magZ;              // Uncompensated magnetic Z-axis.
    float            accelX;            // Uncompensated acceleration X-axis.
    float            accelY;            // Uncompensated acceleration Y-axis.
    float            accelZ;            // Uncompensated acceleration Z-axis.
    float            gyroX;             // Uncompensated angular rate X-axis.
    float            gyroY;             // Uncompensated angular rate Y-axis.
    float            gyroZ;             // Uncompensated angular rate Z-axis.
    float            temp;              // IMU temperature.
    float            pressure;          // Barometric pressure.
    
    VN100_VPE_STATUS vpeStatus;         // Vector Processing Engine Status.
    uint32_t         syncInTime;        // Time since last SyncIn trigger.
    
} VN100_IMU_MEAS_PL;

// YPR Measurement Payload.
typedef struct __attribute__ ((packed))
{
    float            yaw;               
    float            pitch;             
    float            roll;    
    
    VN100_VPE_STATUS vpeStatus;
    uint32_t         syncInTime;
    
} VN100_YPR_MEAS_PL;

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

FMUCOMM_IMU_DATA_PL vn100_lan_imu_data;
    
//
// Communication Protocol Control Data -----------------------------------------
//

static       VN100_COMM_PROTO_READ_PL  vn100_comm_read_proto_pl;
static const VN100_COMM_PROTO_WRITE_PL vn100_comm_write_proto_pl =
{
    0,    // serialCount    - N/A, SPI used. 
    0,    // serialStatus   - N/A, SPI used.  
    2,    // spiCount       - Append SPI messages with SyncIn Time.
    1,    // spiStatus      - Append SPI messages with VPE status.
    0,    // serialChecksum - N/A, SPI used.
    3,    // spiChecksum    - Append SPI messages with a 16-bit CRC.
    0,    // errorMode      - N/A, SPI used.
};

static VN100_COMM_PKT vn100_comm_read_proto_pkt = 
{
    .cmdID       = VN100_READ_REG_CMD,
    .arg         = 30,
    
    .reqPl       = NULL,
    .reqPlLen    = 0,
    
    .respPl      = (uint8_t*) &vn100_comm_read_proto_pl,
    .respPlLen   = sizeof( vn100_comm_read_proto_pl ),
    
    .crcComp     = true,        // CRC computed since this is a check to see if VN100 already configured.
    .commSuccess = false,
};

static VN100_COMM_PKT vn100_comm_write_proto_pkt = 
{
    .cmdID       = VN100_WRITE_REG_CMD,
    .arg         = 30,
    
    .reqPl       = (uint8_t*) &vn100_comm_write_proto_pl,
    .reqPlLen    = sizeof( vn100_comm_write_proto_pl ),
    
    .respPl      = NULL,                                  // Don't care about response field.
    .respPlLen   = sizeof( vn100_comm_write_proto_pl ),   // Still need to read response field.
    
    .crcComp     = false,       // CRC not computed since VN100 not yet configured.
    .commSuccess = false,
};

//
// IMU Measurement Data --------------------------------------------------------
//

static VN100_IMU_MEAS_PL vn100_imu_meas_pl;

static VN100_COMM_PKT vn100_imu_meas_pkt = 
{
    .cmdID      = VN100_READ_REG_CMD,
    .arg        = 54,
    
    .reqPl      = NULL,
    .reqPlLen   = 0,
            
    .respPl     = (uint8_t*) &vn100_imu_meas_pl,
    .respPlLen  = sizeof( vn100_imu_meas_pl ),   
    
    .crcComp     = true,
    .commSuccess = false,
};

//
// YPR Measurement Data --------------------------------------------------------
//

static VN100_YPR_MEAS_PL vn100_ypr_meas_pl;

static VN100_COMM_PKT vn100_ypr_meas_pkt = 
{
    .cmdID      = VN100_READ_REG_CMD,
    .arg        = 8,
    
    .reqPl      = NULL,
    .reqPlLen   = 0,
            
    .respPl     = (uint8_t*) &vn100_ypr_meas_pl,
    .respPlLen  = sizeof( vn100_ypr_meas_pl ),   
    
    .crcComp     = true,
    .commSuccess = false,
};

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

static void VN100LANBuildData( void );
static bool VN100Comm( VN100_COMM_PKT* pkt );

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

void VN100Task( void )
{
    // Note: The chosen method of startup is to detect if the VN100 is 
    // configured for operation.  If the device is not correctly configured,
    // then the software writes the configuration to the peripheral.  If the 
    // device is already configured, then normal operation is immediately
    // performed.
    //
    // An alternative method could be performed in which the VN100 is always
    // reset on startup so that its state is known.  This method has the major
    // disadvantage of resetting the Kalman filter.  If the PIC32 experiences
    // a WDT reset during flight, immediate response of the VN100 is necessary
    // to achieve the best potential of maintaining flight.
    //
    static enum
    {
        SM_DETECT,          // Detect if the VN-100 is already configured correctly.
        SM_INIT,            // Initialize the VN-100 peripheral.
        SM_SPI_IMU_GET,     // Get IMU data - i.e. mag, accel, gyro, temp, pressure.
        SM_SPI_YPR_GET,     // Get YPR data - i.e. yaw, pitch, roll.
        SM_LAN_DATA_BUILD,  // Build LAN/Ethernet data for transfer.
        SM_LAN_DATA_SEND,   // Sent the constructed LAN/Ethernet data.
        SM_DELAY,           // Delay required time for periodic transfer of data.
                
    } vn100TaskState = SM_DETECT;

    switch (vn100TaskState)
    {
        case SM_DETECT:
        {
            // Read the Communication Protocol Control register until
            // communication is completed.
            if( VN100Comm( &vn100_comm_read_proto_pkt ) == true )
            {
                // Register was read successfully and matches expected value ?
                if( ( vn100_comm_read_proto_pkt.commSuccess   == true                                     ) &&
                    ( vn100_comm_read_proto_pl.serialCount    == vn100_comm_write_proto_pl.serialCount    ) &&
                    ( vn100_comm_read_proto_pl.serialStatus   == vn100_comm_write_proto_pl.serialStatus   ) &&
                    ( vn100_comm_read_proto_pl.spiCount       == vn100_comm_write_proto_pl.spiCount       ) &&
                    ( vn100_comm_read_proto_pl.spiStatus      == vn100_comm_write_proto_pl.spiStatus      ) &&
                    ( vn100_comm_read_proto_pl.serialChecksum == vn100_comm_write_proto_pl.serialChecksum ) &&
                    ( vn100_comm_read_proto_pl.spiChecksum    == vn100_comm_write_proto_pl.spiChecksum    ) &&
                    ( vn100_comm_read_proto_pl.errorMode      == vn100_comm_write_proto_pl.errorMode      ) )
                {
                    // Configuration of the VN100 is already complete;
                    // continue immediately with normal operation.
                    vn100TaskState = SM_SPI_IMU_GET;
                }
                else
                {
                    // VN100 is not yet configured; perform peripheral
                    // initialization.
                    vn100TaskState = SM_INIT;
                }
            }
            break;
        }
        case SM_INIT:
        {
            // Write the Communication Protocol Control register until
            // communication is completed.
            if( VN100Comm( &vn100_comm_write_proto_pkt ) == true )
            {
                // Register was successfully written ?
                //
                // Note: if not successfully written, the state machine will
                // retain and current state and the register will attempt to
                // be re-written.
                //
                if( vn100_comm_write_proto_pkt.commSuccess == true )
                {
                    vn100TaskState++;
                }
            }
            break;
        }
        case SM_SPI_IMU_GET:
        {
            // Read IMU measurement data until communication is completed.
            if( VN100Comm( &vn100_imu_meas_pkt ) == true )
            {
                vn100TaskState++;
            }
            break;
        }
        case SM_SPI_YPR_GET:
        {
            // Read YPR measurement data until communication is completed.
            if( VN100Comm( &vn100_ypr_meas_pkt ) == true )
            {
                vn100TaskState++;
            }
            break;
        }
        case SM_LAN_DATA_BUILD:
        {
            // Build the data field in the LAN/Ethernet packet.
            VN100LANBuildData();
            
            vn100TaskState++;
            
            break;
        }
        case SM_LAN_DATA_SEND:
        {
            bool pktSuccess;
            
            // Queue packet for transmission.
            pktSuccess = FMUCommSet( FMUCOMM_TYPE_IMU_DATA,
                                     (uint8_t*) &vn100_lan_imu_data,
                                     sizeof( vn100_lan_imu_data ) );
            
            // Packed successfully queued for transmission ?
            //
            // Note: if unsuccessfully current machine state will be retained
            // and packet will attempt to be re-queued.
            //
            if( pktSuccess == true )
            {
                vn100TaskState++;
            }
            
            break;
        }
        case SM_DELAY:
        {
            static uint32_t prevExeTime = 0;
            
            // Required time has elapsed ?
            //
            // Delay required amount of time so that a cycle of the Task
            // Execution is performed every ~10ms. This is to accomplish an
            // IMU data annunciation on LAN at 100Hz.
            //
            // Note: The VN-100 has data available from the IMU subsystem at
            // a 800Hz rate, and data available from the NavFilter Subsystem
            // (i.e. attitude data) at a 400Hz rate. Therefore, fresh data will
            // always be available at the 100Hz annunciation rate.
            //
            if( CoreTime32usGet() - prevExeTime > 10000 )
            {
                // Increment by fixed transmission period time to eliminate
                // drift.
                prevExeTime += 10000;

                // Still identified that require time has elapsed ?
                //
                // Note: This could occur if processing inhibited this function's 
                // execution for an extended amount of time.
                //
                if( CoreTime32usGet() - prevExeTime > 10000 )
                {
                    // Update to the current time.  Single or multiple timeouts
                    // have elapsed.  Updating to the current time prevents
                    // repeated identifications of timeout having elapsed.
                    prevExeTime = CoreTime32usGet();
                }

                // Perform another task cycle.
                vn100TaskState = SM_SPI_IMU_GET;
            }
            
            break;
        }
    }
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************

////////////////////////////////////////////////////////////////////////////////
/// @brief  Build VN100 Ethernet data.
///
/// This function builds an \e IMU_DATA Ethernet packet for transmission.
////////////////////////////////////////////////////////////////////////////////
static void VN100LANBuildData( void )
{
    vn100_lan_imu_data.fmuTime       = CoreTime64usGet();
    vn100_lan_imu_data.imuType       = VN100_IMU_TYPE;
    
    vn100_lan_imu_data.imuTimeSyncIn = vn100_imu_meas_pl.syncInTime;
    vn100_lan_imu_data.magX          = vn100_imu_meas_pl.magX;
    vn100_lan_imu_data.magY          = vn100_imu_meas_pl.magY;
    vn100_lan_imu_data.magZ          = vn100_imu_meas_pl.magZ;
    vn100_lan_imu_data.accelX        = vn100_imu_meas_pl.accelX;
    vn100_lan_imu_data.accelY        = vn100_imu_meas_pl.accelY;
    vn100_lan_imu_data.accelZ        = vn100_imu_meas_pl.accelZ;
    vn100_lan_imu_data.gyroX         = vn100_imu_meas_pl.gyroX;
    vn100_lan_imu_data.gyroY         = vn100_imu_meas_pl.gyroY;
    vn100_lan_imu_data.gyroZ         = vn100_imu_meas_pl.gyroZ;
    vn100_lan_imu_data.temp          = vn100_imu_meas_pl.temp;
    vn100_lan_imu_data.pressure      = vn100_imu_meas_pl.pressure;
    
    vn100_lan_imu_data.attTimeSyncIn = vn100_ypr_meas_pl.syncInTime;
    vn100_lan_imu_data.yaw           = vn100_ypr_meas_pl.yaw;
    vn100_lan_imu_data.pitch         = vn100_ypr_meas_pl.pitch;
    vn100_lan_imu_data.roll          = vn100_ypr_meas_pl.roll;
    
    // Default all data as valid; data set invalid if issue exists.
    vn100_lan_imu_data.imuValid.mag    = 1;
    vn100_lan_imu_data.imuValid.accel  = 1;
    vn100_lan_imu_data.imuValid.gyro   = 1;
    vn100_lan_imu_data.imuValid.temp   = 1;
    vn100_lan_imu_data.imuValid.press  = 1;
    vn100_lan_imu_data.imuValid.att    = 1;
    
    // IMU Measurement data reception unsuccessfully ?
    if( vn100_imu_meas_pkt.commSuccess == false )
    {
        vn100_lan_imu_data.imuValid.mag    = 0;
        vn100_lan_imu_data.imuValid.accel  = 0;
        vn100_lan_imu_data.imuValid.gyro   = 0;
        vn100_lan_imu_data.imuValid.temp   = 0;
        vn100_lan_imu_data.imuValid.press  = 0;
    }
    else
    {
        // Issue with magnetometer data ?
        if( ( vn100_imu_meas_pl.vpeStatus.MagDisturbance        != 0 ) ||
            ( vn100_imu_meas_pl.vpeStatus.MagSaturation         != 0 ) ||
            ( vn100_imu_meas_pl.vpeStatus.KnownAccelDisturbance != 0 ) )
        {
            vn100_lan_imu_data.imuValid.mag = 0;
        }

        // Issue with accelerometer data ?
        if( ( vn100_imu_meas_pl.vpeStatus.AccDisturbance        != 0 ) ||
            ( vn100_imu_meas_pl.vpeStatus.AccSaturation         != 0 ) ||
            ( vn100_imu_meas_pl.vpeStatus.KnownAccelDisturbance != 0 ) )
        {
            vn100_lan_imu_data.imuValid.accel = 0;
        }

        // Issue with gyroscope data ?
        if( ( vn100_imu_meas_pl.vpeStatus.GyroSaturation         != 0 ) ||
            ( vn100_imu_meas_pl.vpeStatus.GyroSaturationRecovery != 0 ) )
        {
            vn100_lan_imu_data.imuValid.gyro = 0;
        }
    }
    
    // YPR Measurement data reception unsuccessfully ?
    if( vn100_ypr_meas_pkt.commSuccess == false )
    {
        vn100_lan_imu_data.imuValid.att = 0;
    }
    else
    {
        // Attitude quality is not Excellent or Good ?
        if( vn100_ypr_meas_pl.vpeStatus.AttitudeQuality > 1 )
        {
            vn100_lan_imu_data.imuValid.att = 0;
        }
    }
}

//==============================================================================

////////////////////////////////////////////////////////////////////////////////
/// @brief  VN100 communication transfer.
///
/// @param  pkt
///             Control and communication data for the transfer.
///
/// @return Identification of communication transfer being completed.
///             true  - transfer is complete.
///             flase - transfer is in progress.
///
/// @note   Calling function should set up elements:
///             - arg
///             - cmdID
///             - reqPl
///             - reqPlLen
///             - respPlLen
///             - respPl (pointer)
///             - crcComp
///         This function will populate the following elements based upon the 
///         response data received.
///             - respPl (value if not NULL)
///             - respStat
///             - respCnt
///             - commDone
///
/// This function preform a request and response communication transfer with 
/// the VN100 to read and set peripheral registers.
////////////////////////////////////////////////////////////////////////////////
static bool VN100Comm( VN100_COMM_PKT* pkt )
{
    static uint8_t vn100XferBuf[ VN100_XFER_BUF_SIZE ];
    
    static const SPI_PORT VN100_SPI_PORT = SPI_PORT_SPI2;
    static       SPI_XFER vn100spi;
                 bool     commDone       = false;
    
    static enum
    {
        SM_REQUEST_START,
        SM_REQUEST_FINISH,
        SM_WAIT,
        SM_RESPONSE_START,
        SM_RESPONSE_FINISH,
                
    } commState = SM_REQUEST_START;
    
    // Default transfer not to successful.
    pkt->commSuccess = false;
    
    // Execute the state machine.
    switch( commState )
    {
        case SM_REQUEST_START:
        {
            uint16_t calcCRC;
            
            // Build header field.
            vn100XferBuf[ 0 ] = pkt->cmdID;
            vn100XferBuf[ 1 ] = pkt->arg;
            vn100XferBuf[ 2 ] = 0;
            vn100XferBuf[ 3 ] = 0;
            
            // Copy payload to transfer buffer after the header field.
            memcpy( &vn100XferBuf[ VN100_SPI_HEADER_SIZE ], pkt->reqPl, pkt->reqPlLen );
            
            // CRC is to be computed ?
            if( pkt->crcComp == true )
            {
                // Calculate CRC of packet.
                calcCRC = utilCRC16( (void*) &vn100XferBuf[ 0 ], 
                                     mVN100_SPI_CRC_OFFSET( pkt->reqPlLen ),
                                     0 );

                // Append calculated CRC to packet after the payload.
                //
                // Note: Endianness of CRC needs to be swapped.
                //
                vn100XferBuf[ mVN100_SPI_CRC_OFFSET( pkt->reqPlLen ) + 0 ] = (uint8_t) ( ( calcCRC >> 8 ) & 0x00FF );
                vn100XferBuf[ mVN100_SPI_CRC_OFFSET( pkt->reqPlLen ) + 1 ] = (uint8_t) ( ( calcCRC >> 0 ) & 0x00FF );
                
                // Set the VN100 SPI transfer length
                vn100spi.length = mVN100_SPI_PKT_SIZE( pkt->reqPlLen );
            }
            else
            {
                vn100spi.length = mVN100_SPI_PKT_SIZE_NOCRC( pkt->reqPlLen );
            }

            // Set up VN100 packet for SPI transfer.
            vn100spi.port   = VN100_SPI_PORT;
            vn100spi.rxBuf  = NULL;
            vn100spi.txBuf  = &vn100XferBuf[ 0 ];
            
            // Message successfully queued for transfer ?
            if( SPIXfer( &vn100spi ) == 0 )
            {
                commState++;
            }
            break;
        }
        case SM_REQUEST_FINISH:
        {
            // SPI transfer of Request Packet is complete ?
            if (vn100spi.xferDone != 0)
            {
                commState++;
            }
            else
            {
                break;
            }
            // No break - continue immediately with next state if transfer done.
        }
        case SM_WAIT:
        {
            static uint32_t startTime = 0;
            
            // Setup start-time at beginning of delay.
            if( startTime == 0 )
            {
                startTime = CoreTime32usGet();
            }
            
            // Required time has elapsed ?
            // 
            // Note: Must delay at least 50us before issuing the response packet
            // to satisfy VN100 operation. Buffer is added to wait no less
            // than 100us.
            // 
            if( CoreTime32usGet() - startTime > 100 )
            {
                commState++;
                
                // Clear the start time for evaluation on next delay.
                startTime = 0;
            }
            else
            {
                break;
            }
            // No break - continue immediately with next state if delay elapsed.
        }
        case SM_RESPONSE_START:
        {
            // Set up VN100 packet for SPI transfer.
            vn100spi.port   = VN100_SPI_PORT;
            vn100spi.rxBuf  = &vn100XferBuf[ 0 ];
            vn100spi.txBuf  = NULL;
            
            // CRC is to be computed ?
            if( pkt->crcComp == true )
            {
                vn100spi.length = mVN100_SPI_PKT_SIZE( pkt->respPlLen );
            }
            else
            {
                vn100spi.length = mVN100_SPI_PKT_SIZE_NOCRC( pkt->respPlLen );
            }

            // Message successfully queued for transfer ?
            if( SPIXfer( &vn100spi ) == 0 )
            {
                commState++;
            }
            break;
        }
        case SM_RESPONSE_FINISH:
        {
            // SPI transfer of Response Packet is complete ?
            if( vn100spi.xferDone != 0 )
            {
                uint16_t calcCRC = 0;
                
                // CRC is to be computed ?
                if( pkt->crcComp == true )
                {
                    // Calculate received data CRC.
                    calcCRC = utilCRC16( (void*) &vn100XferBuf[ 0 ], 
                                         mVN100_SPI_PKT_SIZE( pkt->respPlLen ),
                                         0 );
                }
                
                // All of the following data is correct ?
                //  - Argument matches that transmitted.
                //  - command ID matches that transmitted.
                //  - Error ID = 0, i.e. no errors occurred.
                //  - Data is valid, i.e. received CRC is correct.
                //
                if( ( vn100XferBuf[ 1 ] == pkt->cmdID ) &&
                    ( vn100XferBuf[ 2 ] == pkt->arg   ) &&
                    ( vn100XferBuf[ 3 ] == 0          ) &&
                    ( calcCRC           == 0          ) )
                {
                    // Identify the transfer as successful.
                    pkt->commSuccess = true;
                    
                    // Response payload pointer to valid buffer ?
                    if( pkt->respPl != NULL )
                    {
                        // Populate the received response payload.
                        memcpy( pkt->respPl,
                                &vn100XferBuf[ VN100_SPI_HEADER_SIZE ],
                                pkt->respPlLen );
                    }
                }
                
                // Identify the communication transfer as complete.
                commDone = true;
                
                // Re-start state machine.
                commState = SM_REQUEST_START;
            }
            break;
        }
    }
    
    return commDone;
}