/*******************************************************************************
/
/   Filename:   vn100.c
/
*******************************************************************************/

#include <xc.h>
#include "vn100.h"
#include "coretime.h"
#include "spi.h"
#include "stdtypes.h"
#include "fmucomm.h"
#include "stdbool.h"
#include "util.h"

#define VN100_MAX_PAYLOAD_SIZE  100     // Maximum payload size is 100 bytes.

// The type of IMU being read. '0' identifies a VN-100.
#define VN100_IMU_TYPE  0U

// VN-100 Register IDs

typedef enum {
    VN100_REG_TAG       = 0,    // User Tag Register
    VN100_REG_MODEL     = 1,    // Model Number Register
    VN100_REG_HWREV     = 2,    // Hardware Revision Register
    VN100_REG_SN        = 3,    // Serial Number Register
    VN100_REG_FWVER     = 4,    // Firmware Version Register
    VN100_REG_SBAUD     = 5,    // Serial Baud Rate Register
    VN100_REG_ADOR      = 6,    // Async Data Output Type Register
    VN100_REG_ADOF      = 7,    // Async Data Output Frequency Register
    VN100_REG_YPR       = 8,    // Yaw Pitch Roll
    VN100_REG_QTN       = 9,    // Attitude Quaternion
    VN100_REG_QTM       = 10,
    VN100_REG_QTA       = 11,
    VN100_REG_QTR       = 12,
    VN100_REG_QMA       = 13,
    VN100_REG_QAR       = 14,
    VN100_REG_QMR       = 15,   // Quat, Mag, Accel, and Ang Rates
    VN100_REG_DCM       = 16,
    VN100_REG_MAG       = 17,   // Magnetic Measurements
    VN100_REG_ACC       = 18,   // Acceleration Measurements
    VN100_REG_GYR       = 19,   // Angular Rate Measurements
    VN100_REG_MAR       = 20,   // Mag, Accel, and Ang Rates
    VN100_REG_REF       = 21,   // Magnetic and Gravity Reference Vectors
    VN100_REG_SIG       = 22,
    VN100_REG_HSI       = 23,   // Magnetometer Compensation
    VN100_REG_ATP       = 24,
    VN100_REG_ACT       = 25,   // Accelerometer Compensation
    VN100_REG_RFR       = 26,   // Reference Frame Rotation
    VN100_REG_YMR       = 27,   // Yaw, Pitch, Roll, Mag, Accel, Ang Rates
    VN100_REG_ACG       = 28,
    VN100_REG_PROT      = 30,   // Communication Protocol Control
    VN100_REG_SYNC      = 32,   // Synchronization Control
    VN100_REG_STAT      = 33,   // Synchronization Status
    VN100_REG_VPE       = 35,   // Vector Processing Engine Basic Control
    VN100_REG_VPEMBT    = 36,   // VPE Magnetometer Basic Tuning
    VN100_REG_VPEABT    = 38,   // VPE Accelerometer Basic Tuning
    VN100_REG_MCC       = 44,   // Magnetometer Calibration Control
    VN100_REG_CMC       = 47,   // Calculated Magnetometer Calibration
    VN100_REG_VCM       = 50,   // Velocity Compensation Measurement
    VN100_REG_VCC       = 51,   // Velocity Compensation Control
    VN100_REG_VCS       = 52,   // Velocity Compensation Status
    VN100_REG_IMU       = 54,   // IMU Measurements
    VN100_REG_BOR1      = 75,   // Binary Output Register 1
    VN100_REG_BOR2      = 76,   // Binary Output Register 2
    VN100_REG_BOR3      = 77,   // Binary Output Register 3
    VN100_REG_DTDV      = 80,   // Delta Theta and Delta Velocity
    VN100_REG_DTDVC     = 82,   // Delta Theta and Delta Velocity Config
    VN100_REG_RVC       = 83,   // Reference Vector Configuration
    VN100_REG_GCMP      = 84,   // Gyro Compensation
    VN100_REG_IMUF      = 85,   // IMU Filtering Configuration
    VN100_REG_YPRTBAAR  = 239,  // Y/P/R, True Body Accel, Ang Rates
    VN100_REG_YPRTIAAR  = 240,  // Y/P/R, True Inertial Accel, Ang Rates
    VN100_REG_RAW       = 251,
    VN100_REG_CMV       = 252,
    VN100_REG_STV       = 253,
    VN100_REG_COV       = 254,
    VN100_REG_CAL       = 255,
} __attribute__ ((packed)) VN100_REG_E;


// VN-100 Command IDs

typedef enum {
    VN100_CMD_READ_REG                  = 0x01,
    VN100_CMD_WRITE_REG                 = 0x02,
    VN100_CMD_WRITE_SETTINGS            = 0x03,
    VN100_CMD_RESTORE_FACTORY_SETTINGS  = 0x04,
    VN100_CMD_TARE                      = 0x05,
    VN100_CMD_RESET                     = 0x06,
    VN100_CMD_FLASH_FIRMWARE            = 0x07,
    VN100_CMD_SET_REF_FRAME             = 0x08,
    VN100_CMD_HARDWARE_IN_LOOP          = 0x09,
    VN100_CMD_GET_FLASH_CNT             = 0x0A,
    VN100_CMD_CALIBRATE                 = 0x0B,
} __attribute__ ((packed)) VN100_CMD_E;


// VN-100 Error IDs

typedef enum {
    VN100_ERR_NONE                      = 0,
    VN100_ERR_HARD_FAULT_EXCEPTION      = 1,
    VN100_ERR_INPUT_BUFFER_OVERFLOW     = 2,
    VN100_ERR_INVALID_CHECKSUM          = 3,
    VN100_ERR_INVALID_COMMAND           = 4,
    VN100_ERR_NOT_ENOUGH_PARAMS         = 5,
    VN100_ERR_TOO_MANY_PARAMS           = 6,
    VN100_ERR_INVALID_PARAM             = 7,
    VN100_ERR_INVALID_REG               = 8,
    VN100_ERR_UNAUTHORIZED_ACCESS       = 9,
    VN100_ERR_WATCHDOG_RESET            = 10,
    VN100_ERR_OUTPUT_BUFFER_OVERFLOW    = 11,
    VN100_ERR_INSUFFICIENT_BANDWIDTH    = 12,
    VN100_ERR_LIST_OVERFLOW             = 255,
} __attribute__ ((packed)) VN100_ERR_E;

typedef struct __attribute__ ((packed)) {
    VN100_CMD_E     cmdID : 8;      // Command ID.
    VN100_REG_E     regID : 8;      // Register ID.
    uint8_t         zero_0;         // Zero byte. (0x00)
    uint8_t         zero_1;         // Zero byte. (0x00)
} VN100_SPI_REQUEST_HEADER;


typedef struct __attribute__ ((packed)) {
    uint8_t         zero_0;         // Zero byte. (0x00)
    VN100_CMD_E     cmdID : 8;      // Command ID.
    VN100_REG_E     regID : 8;      // Register ID.
    VN100_ERR_E     errID : 8;      // Error ID.
} VN100_SPI_RESPONSE_HEADER;


typedef union {
    VN100_SPI_REQUEST_HEADER    request;        // Request header format.
    VN100_SPI_RESPONSE_HEADER   response;       // Response header format.
    UINT_32                     raw;            // Raw header data.
} VN100_SPI_HEADER;


typedef struct __attribute__ ((packed)) {
    VN100_SPI_HEADER    header;                             // Packet header.
    uint8_t             payload[VN100_MAX_PAYLOAD_SIZE];    // Packet payload.
//    uint16_t            crc16;                              // Packet checksum.
} VN100_SPI_PKT;

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

// IMU Measurements (ID 54)
static struct
{
    float            magX;               // Uncompensated magnetic X-axis.
    float            magY;               // Uncompensated magnetic Y-axis.
    float            magZ;               // Uncompensated magnetic Z-axis.
    float            accelX;             // Uncompensated acceleration X-axis.
    float            accelY;             // Uncompensated acceleration Y-axis.
    float            accelZ;             // Uncompensated acceleration Z-axis.
    float            gyroX;              // Uncompensated angular rate X-axis.
    float            gyroY;              // Uncompensated angular rate Y-axis.
    float            gyroZ;              // Uncompensated angular rate Z-axis.
    float            temp;               // IMU temperature.
    float            pressure;           // Barometric pressure.
    uint32_t         syncInTime;
    VN100_VPE_STATUS vpeStatus;
    uint16_t         crc16;
} imuMeasurements;

// IMU Measurements (ID 54)
static struct
{
    float            yaw;               
    float            pitch;             
    float            roll;    
    uint32_t         syncInTime;
    VN100_VPE_STATUS vpeStatus;
    uint16_t         crc16;
} yprMeasurements;

FMUCOMM_IMU_DATA imuData;

// Local function prototypes. ==================================================

static int VN100Init();
void VN100PktBuildData( void );
static unsigned int VN100RegSizeGet(VN100_REG_E reg);
static VN100_SPI_PKT* VN100ReadReg(VN100_REG_E reg);
static VN100_SPI_PKT* VN100WriteReg(VN100_REG_E reg, void* regData);

//==============================================================================


void VN100Task()
{
    static enum
    {
        SM_INIT,
        SM_GET_IMU,
        SM_GET_YPR,
        SM_BUILD_DATA,
        SM_PKT_SEND,
        SM_ADD_DELAY,
    } vn100TaskState = SM_INIT;

    VN100_SPI_PKT* pkt_p;

    switch (vn100TaskState)
    {
        case SM_INIT:
        {
            // Perform initialization until complete.
            if ( VN100Init() == 0 )
            {
                vn100TaskState++;
            }
            break;
        }
        case SM_GET_IMU:
        {
            pkt_p = VN100ReadReg( VN100_REG_IMU );
            
            // Reading of register is complete ?
            if( pkt_p != 0 )
            {
                // Error did not occur with register read?  If error did occur, 
                // the register will attempt to be re-read.
                if( pkt_p->header.response.errID == VN100_ERR_NONE )
                {
                    // Copy the read data to module data.
                    memcpy( &imuMeasurements,
                            pkt_p->payload,
                            sizeof( imuMeasurements ) );

                    vn100TaskState++;
                }
            }
            break;
        }
        case SM_GET_YPR:
        {
            pkt_p = VN100ReadReg( VN100_REG_YPR );
            
            // Reading of register is complete ?
            if( pkt_p != 0 )
            {
                // Error did not occur with register read?  If error did occur, 
                // the register will attempt to be re-read.
                if( pkt_p->header.response.errID == VN100_ERR_NONE )
                {
                    // Copy the read data to module data.
                    memcpy( &yprMeasurements,
                            pkt_p->payload,
                            sizeof( yprMeasurements ) );

                    vn100TaskState++;
                }
            }
            break;
        }
        case SM_BUILD_DATA:
        {
            // Build the data field in the ethernet packet.
            VN100PktBuildData();
            vn100TaskState++;
            
            break;
        }
        case SM_PKT_SEND:
        {
            bool pktSuccess;
            
            // Queue packet for transmission.
            pktSuccess = FMUCommSet( FMUCOMM_TYPE_IMU_DATA,
                                     (uint8_t*) &imuData,
                                     sizeof( imuData ) );
            
            // Packed successfully queued for transmission ?
            if( pktSuccess == true )
            {
                vn100TaskState++;
            }
            
            break;
        }
        case SM_ADD_DELAY:
        {
            static uint32_t startTime = 0;
            
            // Setup start-time at beginning of delay.
            if( startTime == 0 )
            {
                startTime = CoreTime32usGet();
            }
            
            // Required time has elapsed ?
            //  - Delay 2.5ms so new IMU data will be available (800Hz rate)
            //    from the IMU subsystem, and new attitude data will be
            //    available (400Hz rate) from the NavFilter Subsystem.
            // 
            if( CoreTime32usGet() - startTime > 2500 )
            {
                // Perform another task cycle.
                vn100TaskState = SM_GET_IMU;
                
                // Clear the start time for evaluation on next delay.
                startTime = 0;
            }
            
            break;
        }
    }
}


//==============================================================================

static int VN100Init()
{
    // Note: Synchronization Control register is not set (i.e. left as default)
    // since the VN100 is operated with GPS synchronized sampling.  Since
    // the VN100 annunciates elapsed time since the GPS trigger and the 
    // MCU also monitors this trigger, precise sampling time can be determined.
    
    const struct
    {
        uint8_t  serialCount;
        uint8_t  serialStatus;
        uint8_t  spiCount;
        uint8_t  spiStatus;
        uint8_t  serialChecksum;
        uint8_t  spiChecksum;
        uint8_t  errorMode;
    } reg_prot_val = {
        0,    // serialCount    - N/A, SPI used. 
        0,    // serialStatus   - N/A, SPI used.  
        2,    // spiCount       - Append SPI messages with SyncIn Time.
        1,    // spiStatus      - Append SPI messages with VPE status.
        0,    // serialChecksum - N/A, SPI used.
        0,    // spiChecksum    - Append SPI messages with a 16-bit CRC.            // DEBUG: Updated from '3' for testing
        0,    // errorMode      - N/A, SPI used.
    };
    
    VN100_SPI_PKT* pkt_p;
    int retVal = 1;
    
    pkt_p = VN100WriteReg( VN100_REG_PROT, (void*) &reg_prot_val );
    
    // Writing of register is complete ?
    if( pkt_p != 0 )
    {
        // Error did not occur with register update?  If error did occur, the
        // register will attempt to be re-written.
        if( pkt_p->header.response.errID == VN100_ERR_NONE )
        {
            // Identify initialization as successful.
            retVal = 0;
        }
    }
            
    return retVal;
}


//==============================================================================

void VN100PktBuildData( void )
{
    uint16_t calcCRC;
    
    imuData.fmuTime = CoreTime64usGet();
    imuData.imuType = VN100_IMU_TYPE;
    
    imuData.imuTimeSyncIn = imuMeasurements.syncInTime;
    imuData.magX          = imuMeasurements.magX;
    imuData.magY          = imuMeasurements.magY;
    imuData.magZ          = imuMeasurements.magZ;
    imuData.accelX        = imuMeasurements.accelX;
    imuData.accelY        = imuMeasurements.accelY;
    imuData.accelZ        = imuMeasurements.accelZ;
    imuData.gyroX         = imuMeasurements.gyroX;
    imuData.gyroY         = imuMeasurements.gyroY;
    imuData.gyroZ         = imuMeasurements.gyroZ;
    imuData.temp          = imuMeasurements.temp;
    imuData.pressure      = imuMeasurements.pressure;
    
    imuData.attTimeSyncIn = yprMeasurements.syncInTime;
    imuData.yaw           = yprMeasurements.yaw;
    imuData.pitch         = yprMeasurements.pitch;
    imuData.roll          = yprMeasurements.roll;
    
    // Default all data as valid; data set invalid if issue exists.
    imuData.imuValid.mag    = 1;
    imuData.imuValid.accel  = 1;
    imuData.imuValid.gyro   = 1;
    imuData.imuValid.temp   = 1;
    imuData.imuValid.press  = 1;
    imuData.imuValid.att    = 1;
    
    // Calculate the CRC of IMU measurement data.
    calcCRC = utilCRC16( (void*) &imuMeasurements, sizeof( imuMeasurements ), 0 );
    
    // Calculate CRC does not match that sent ?
    if( calcCRC != imuMeasurements.crc16 )
    {
        // invalidate IMU measurement data
        imuData.imuValid.mag    = 0;
        imuData.imuValid.accel  = 0;
        imuData.imuValid.gyro   = 0;
        imuData.imuValid.temp   = 0;
        imuData.imuValid.press  = 0;
    }
    
    // Issue with magnetometer data ?
    if( ( imuMeasurements.vpeStatus.MagDisturbance        != 0 ) ||
        ( imuMeasurements.vpeStatus.MagSaturation         != 0 ) ||
        ( imuMeasurements.vpeStatus.KnownAccelDisturbance != 0 ) )
    {
        imuData.imuValid.mag = 0;
    }
    
    // Issue with accelerometer data ?
    if( ( imuMeasurements.vpeStatus.AccDisturbance        != 0 ) ||
        ( imuMeasurements.vpeStatus.AccSaturation         != 0 ) ||
        ( imuMeasurements.vpeStatus.KnownAccelDisturbance != 0 ) )
    {
        imuData.imuValid.accel = 0;
    }
    
    // Issue with gyroscope data ?
    if( ( imuMeasurements.vpeStatus.GyroSaturation         != 0 ) ||
        ( imuMeasurements.vpeStatus.GyroSaturationRecovery != 0 ) )
    {
        imuData.imuValid.gyro = 0;
    }
   
    // Calculate the CRC of YPR measurement data.
    calcCRC = utilCRC16( (void*) &yprMeasurements,  sizeof( yprMeasurements ), 0 );
    
    // Calculate CRC does not match that sent ?
    if( calcCRC != yprMeasurements.crc16 )
    {
        // invalidate YPR measurement data
        imuData.imuValid.att = 0;
    }
    
    // Attitude quality is not Excellent or Good ?
    if( yprMeasurements.vpeStatus.AttitudeQuality > 1 )
    {
        imuData.imuValid.att = 0;
    }
}

// For a Write Register transaction, the request packed payload contains
// the value to write to the selected register.  The response payload contains
// the value written to the register, but only the error code in the header
// is needed to be inspected to verify if the write was successful.
//
static VN100_SPI_PKT* VN100WriteReg(VN100_REG_E reg, void* regData)
{
    VN100_SPI_PKT* retVal = 0;

    static VN100_SPI_PKT pkt;
    
    static SPI_XFER vn100spi = {
        .port = SPI_PORT_SPI2,
    };

    static enum
    {
        SM_REQUEST_START,
        SM_REQUEST_FINISH,
        SM_WAIT,
        SM_RESPONSE_START,
        SM_RESPONSE_FINISH,
    } readRegState = SM_REQUEST_START;

    static uint32_t requestTime;

    switch (readRegState)
    {
        case SM_REQUEST_START:
        {
            // Populate the VN100 Request Packet.
            pkt.header.request.cmdID = VN100_CMD_WRITE_REG;
            pkt.header.request.regID = reg;
            pkt.header.request.zero_0 = 0;
            pkt.header.request.zero_1 = 0;
            memcpy(pkt.payload, regData, VN100RegSizeGet(reg));
            
            // Set up VN100 packet for SPI transfer.
            vn100spi.rxBuf = 0;
            vn100spi.txBuf = (uint8_t*)&pkt;
            vn100spi.length =
                    sizeof(VN100_SPI_HEADER) + VN100RegSizeGet(reg);

            // Message successfully queued for transfer ?
            if (SPIXfer(&vn100spi) == 0)
            {
                readRegState = SM_REQUEST_FINISH;
            }
            break;
        }
        case SM_REQUEST_FINISH:
        {
            // SPI transfer of Request Packet is complete ?
            if (vn100spi.xferDone != 0)
            {
                requestTime = CoreTime32usGet();
                readRegState = SM_WAIT;
            }
            break;
        }
        case SM_WAIT:
        {
            // Per VN-100 user manual, wait at least 50us before
            // issuing the response packet.
            if ((CoreTime32usGet() - requestTime) < 100)
            {
                break;
            }
            else
            {
                readRegState = SM_RESPONSE_START;
            }
            // No break.
        }
        case SM_RESPONSE_START:
        {
            // Set up VN100 packet for SPI transfer of Response Packet.
            vn100spi.rxBuf = (uint8_t*)&pkt;
            vn100spi.txBuf = 0;
            vn100spi.length = 
                    sizeof(VN100_SPI_HEADER) + VN100RegSizeGet(reg);

            // Message successfully queued for transfer ?
            if (SPIXfer(&vn100spi) == 0)
            {
                readRegState = SM_RESPONSE_FINISH;
            }
            break;
        }
        case SM_RESPONSE_FINISH:
        {
            // SPI transfer of Response Packet is complete ?
            if (vn100spi.xferDone != 0)
            {
                // Re-start state machine and return Response Packet alias.
                readRegState = SM_REQUEST_START;
                retVal = &pkt;
            }
            break;
        }
    }
    return retVal;
}


//==============================================================================

static VN100_SPI_PKT* VN100ReadReg(VN100_REG_E reg)
{
    VN100_SPI_PKT* retVal = 0;

    static VN100_SPI_PKT pkt;
    
    static SPI_XFER vn100spi = {
        .port = SPI_PORT_SPI2,
    };

    static enum
    {
        SM_REQUEST_START,
        SM_REQUEST_FINISH,
        SM_WAIT,
        SM_RESPONSE_START,
        SM_RESPONSE_FINISH,
    } readRegState = SM_REQUEST_START;

    static uint32_t requestTime;

    switch (readRegState)
    {
        case SM_REQUEST_START:
        {
            // Populate the VN100 packet.
            pkt.header.request.cmdID = VN100_CMD_READ_REG;
            pkt.header.request.regID = reg;
            pkt.header.request.zero_0 = 0;
            pkt.header.request.zero_1 = 0;

            // Set up VN100 packet for SPI transfer of Request Packet.
            vn100spi.rxBuf = 0;
            vn100spi.txBuf = (uint8_t*)&pkt;
            vn100spi.length = sizeof(pkt.header.request);

            // Message successfully queued for transfer ?
            if (SPIXfer(&vn100spi) == 0)
            {
                readRegState = SM_REQUEST_FINISH;
            }
            break;
        }
        case SM_REQUEST_FINISH:
        {
            // SPI transfer of Request Packet is complete ?
            if (vn100spi.xferDone != 0)
            {
                requestTime = CoreTime32usGet();
                readRegState = SM_WAIT;
            }
            break;
        }
        case SM_WAIT:
        {
            // Per VN-100 user manual, wait at least 50us before
            // issuing the response packet.
            if ((CoreTime32usGet() - requestTime) < 100)
            {
                break;
            }
            else
            {
                readRegState = SM_RESPONSE_START;
            }
            // No break.
        }
        case SM_RESPONSE_START:
        {
            // Set up VN100 packet for SPI transfer of Response Packet.
            vn100spi.rxBuf = (uint8_t*)&pkt;
            vn100spi.txBuf = 0;
            vn100spi.length = 
                    VN100RegSizeGet(reg) + 4;   // Add 4-byte response header.

            // Message successfully queued for transfer ?
            if (SPIXfer(&vn100spi) == 0)
            {
                readRegState = SM_RESPONSE_FINISH;
            }
            break;
        }
        case SM_RESPONSE_FINISH:
        {
            // SPI transfer of Response Packet is complete ?
            if (vn100spi.xferDone != 0)
            {
                // Re-start state machine and return Response Packet alias.
                readRegState = SM_REQUEST_START;
                retVal = &pkt;
            }
            break;
        }
    }
    return retVal;
}


//==============================================================================

static unsigned int VN100RegSizeGet(VN100_REG_E reg)
{
    int size = 0;

    switch (reg)
    {
        case VN100_REG_TAG:         // ID: 0
            size = 20;
            break;
        case VN100_REG_MODEL:       // ID: 1
            size = 24;
            break;
        case VN100_REG_HWREV:       // ID: 2
            size = 4;
            break;
        case VN100_REG_SN:          // ID: 3
            size = 4;
            break;
        case VN100_REG_FWVER:       // ID: 4
            size = 4;
            break;
        case VN100_REG_SBAUD:       // ID: 5
            size = 4;
            break;
        case VN100_REG_ADOR:        // ID: 6
            size = 4;
            break;
        case VN100_REG_ADOF:        // ID: 7
            size = 4;
            break;
        case VN100_REG_YPR:         // ID: 8
            size = 12;
            break;
        case VN100_REG_QTN:         // ID: 9
            size = 16;
            break;
        case VN100_REG_QTM:         // ID: 10
            break;
        case VN100_REG_QTA:         // ID: 11
            break;
        case VN100_REG_QTR:         // ID: 12
            break;
        case VN100_REG_QMA:         // ID: 13
            break;
        case VN100_REG_QAR:         // ID: 14
            break;
        case VN100_REG_QMR:         // ID: 15
            size = 52;
            break;
        case VN100_REG_DCM:         // ID: 16
            break;
        case VN100_REG_MAG:         // ID: 17
            size = 12;
            break;
        case VN100_REG_ACC:         // ID: 18
            size = 12;
            break;
        case VN100_REG_GYR:         // ID: 19
            size = 12;
            break;
        case VN100_REG_MAR:         // ID: 20
            size = 36;
            break;
        case VN100_REG_REF:         // ID: 21
            size = 24;
            break;
        case VN100_REG_SIG:         // ID: 22
            break;
        case VN100_REG_HSI:         // ID: 23
            size = 48;
            break;
        case VN100_REG_ATP:         // ID: 24
            break;
        case VN100_REG_ACT:         // ID: 25
            size = 48;
            break;
        case VN100_REG_RFR:         // ID: 26
            size = 36;
            break;
        case VN100_REG_YMR:         // ID: 27
            size = 48;
            break;
        case VN100_REG_ACG:         // ID: 28
            break;
        case VN100_REG_PROT:        // ID: 30
            size = 7;
            break;
        case VN100_REG_SYNC:        // ID: 32
            size = 20;
            break;
        case VN100_REG_STAT:        // ID: 33
            size = 12;
            break;
        case VN100_REG_VPE:         // ID: 35
            size = 4;
            break;
        case VN100_REG_VPEMBT:      // ID: 36
            size = 36;
            break;
        case VN100_REG_VPEABT:      // ID: 38
            size = 36;
            break;
        case VN100_REG_MCC:         // ID: 44
            size = 4;
            break;
        case VN100_REG_CMC:         // ID: 47
            size = 48;
            break;
        case VN100_REG_VCM:         // ID: 50
            size = 12;
            break;
        case VN100_REG_VCC:         // ID: 51
            size = 8;
            break;
        case VN100_REG_VCS:         // ID: 52
            size = 8;
            break;
        case VN100_REG_IMU:         // ID: 54
            size = 44;
            break;
        case VN100_REG_BOR1:        // ID: 75
            size = 22;
            break;
        case VN100_REG_BOR2:        // ID: 76
            size = 22;
            break;
        case VN100_REG_BOR3:        // ID: 77
            size = 22;
            break;
        case VN100_REG_DTDV:        // ID: 80
            size = 28;
            break;
        case VN100_REG_DTDVC:       // ID: 82
            size = 6;
            break;
        case VN100_REG_RVC:         // ID: 83
            size = 32;
            break;
        case VN100_REG_GCMP:        // ID: 84
            size = 48;
            break;
        case VN100_REG_IMUF:        // ID: 85
            size = 15;
            break;
        case VN100_REG_YPRTBAAR:    // ID: 239
            size = 36;
            break;
        case VN100_REG_YPRTIAAR:    // ID: 240
            size = 36;
            break;
        case VN100_REG_RAW:         // ID: 251
            break;
        case VN100_REG_CMV:         // ID: 252
            break;
        case VN100_REG_STV:         // ID: 253
            break;
        case VN100_REG_COV:         // ID: 254
            break;
        case VN100_REG_CAL:         // ID: 255
            break;
        default:
            break;
    }

    return size;
}


//==============================================================================

