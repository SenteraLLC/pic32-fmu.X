////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Analog to Digital Converter (ADC) driver. 
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

//typedef struct __attribute__ ((packed)) {
//    VN100_CMD_E     cmdID : 8;      // Command ID.
//    VN100_REG_E     regID : 8;      // Register ID.
//    uint8_t         zero_0;         // Zero byte. (0x00)
//    uint8_t         zero_1;         // Zero byte. (0x00)
//} VN100_SPI_REQUEST_HEADER;
//
//
//typedef struct __attribute__ ((packed)) {
//    uint8_t         zero_0;         // Zero byte. (0x00)
//    VN100_CMD_E     cmdID : 8;      // Command ID.
//    VN100_REG_E     regID : 8;      // Register ID.
//    VN100_ERR_E     errID : 8;      // Error ID.
//} VN100_SPI_RESPONSE_HEADER;
//
//
//typedef union {
//    VN100_SPI_REQUEST_HEADER    request;        // Request header format.
//    VN100_SPI_RESPONSE_HEADER   response;       // Response header format.
//    UINT_32                     raw;            // Raw header data.
//} VN100_SPI_HEADER;
//
//typedef struct __attribute__ ((packed))
//{
//    uint16_t crc16;                             // Packet checksum.
//} VN100_SPI_FOOTER;
//
//typedef struct __attribute__ ((packed)) {
//    VN100_SPI_HEADER    header;                             // Packet header.
//    uint8_t             payload[VN100_MAX_PAYLOAD_SIZE];    // Packet payload.
//    VN100_SPI_FOOTER    footer;                             // Packet footer.
//} VN100_SPI_PKT;



#define VN100_XFER_BUF_SIZE 100

// VN100 SPI Read Register
//  (Request)
//      - Header:        4 bytes
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
//      - Payload:       Variable size based on read register
//      - Serial Status: 2 bytes
//      - Serial Count:  2 bytes
//      - CRC:           2 bytes
// 
#define VN100_SPI_HEADER_SIZE 4
#define VN100_SPI_STATUS_SIZE 2
#define VN100_SPI_COUNT_SIZE  2
#define VN100_SPI_CRC_SIZE    2

#define mVN100_SPI_REQ_CRC_OFFSET(x)     ( VN100_SPI_HEADER_SIZE            + (x)                   )
#define mVN100_SPI_REQ_PKT_SIZE(x)       ( mVN100_SPI_REQ_CRC_OFFSET(x)     + VN100_SPI_CRC_SIZE    )

#define mVN100_SPI_RESP_STATUS_OFFSET(x) ( VN100_SPI_HEADER_SIZE            + (x)                   )
#define mVN100_SPI_RESP_COUNT_OFFSET(x)  ( mVN100_SPI_RESP_STATUS_OFFSET(x) + VN100_SPI_STATUS_SIZE )
#define mVN100_SPI_RESP_CRC_OFFSET(x)    ( mVN100_SPI_RESP_COUNT_OFFSET(x)  + VN100_SPI_COUNT_SIZE  )
#define mVN100_SPI_RESP_PKT_SIZE(x)      ( mVN100_SPI_RESP_CRC_OFFSET(x)    + VN100_SPI_CRC_SIZE    )


// Accessed VN100 registers
typedef enum
{
    VN100_REG_PROT,     // Communication Protocol Control
    VN100_REG_YPR,      // Yaw Pitch Roll
    VN100_REG_IMU,      // IMU Measurements
    
    VN100_REG_TOTAL     // Total number of registers accessed.
    
} VN100_REG_E;

typedef struct
{
    uint8_t  regID;
    uint16_t regLen;
    
} VN100_REG_CFG;

//typedef struct __attribute__ ((packed))
//{
//    VN100_CMD_E     cmdID : 8;      // Command ID.
//    VN100_REG_E     regID : 8;      // Register ID.
//    uint8_t         zero_0;         // Zero byte. (0x00)
//    uint8_t         zero_1;         // Zero byte. (0x00)
//    
//    uint8_t*        payload;        // Packet payload.
//    uint16_t        payloadLen;     // Payload length.
//    
//} VN100_SPI_REQ_PKT;
//
//typedef struct __attribute__ ((packed))
//{
//    uint8_t         zero_0;         // Zero byte. (0x00)
//    VN100_CMD_E     cmdID : 8;      // Command ID.
//    VN100_REG_E     regID : 8;      // Register ID.
//    VN100_ERR_E     errID : 8;      // Error ID.
//    
//    uint8_t*        payload;        // Packet payload.
//    uint16_t        payloadLen;     // Payload length.
//    
//    bool            crc16_fail;     // 16-bit CRC failure/mismatch.
//    
//} VN100_SPI_RESP_PKT;

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

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************


static const VN100_REG_CFG vn100_reg_cfg[ VN100_REG_TOTAL ] = 
{
    { 30 ,  7 },       // VN100_REG_PROT
    {  8 , 12 },       // VN100_REG_YPR
    { 54 , 44 },       // VN100_REG_IMU
};

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
    VN100_VPE_STATUS vpeStatus;
    uint32_t         syncInTime;
    
} imuMeasurements;

// IMU Measurements (ID 54)
static struct
{
    float            yaw;               
    float            pitch;             
    float            roll;    
    VN100_VPE_STATUS vpeStatus;
    uint32_t         syncInTime;
    
} yprMeasurements;

FMUCOMM_IMU_DATA imuData;

    
static uint8_t vn100XferBuf[ VN100_XFER_BUF_SIZE ];

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

static int VN100Init();
void VN100PktBuildData( void );
static unsigned int VN100RegSizeGet(VN100_REG_E reg);

static bool VN100ReadReg(  VN100_REG_E reg, void* regData );
static bool VN100WriteReg( VN100_REG_E reg, void* regData );

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

void VN100Task()
{
    static enum
    {
        SM_INIT,
        SM_SPI_IMU_GET,
        SM_SPI_YPR_GET,
        SM_LAN_DATA_BUILD,
        SM_LAN_DATA_SEND,
        SM_DELAY,
                
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
        case SM_SPI_IMU_GET:
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
        case SM_SPI_YPR_GET:
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
        case SM_LAN_DATA_BUILD:
        {
            // Build the data field in the ethernet packet.
            VN100LANBuildData();
            vn100TaskState++;
            
            break;
        }
        case SM_LAN_DATA_SEND:
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
        case SM_DELAY:
        {
            static uint64_t prevExeTime = 0;
            
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
            if( CoreTime64usGet() - prevExeTime > 10000 )
            {
                // Perform another task cycle.
                vn100TaskState = SM_SPI_IMU_GET;
                
                // Latch the execution time for evaluation on next delay.
                prevExeTime = CoreTime64usGet();
            }
            
            break;
        }
    }
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************

// Note: Synchronization Control register is not set (i.e. left as default)
// since the VN100 is operated with GPS synchronized sampling.  Since
// the VN100 annunciates elapsed time since the GPS trigger and the 
// MCU also monitors this trigger, precise sampling time can be determined.
// 
static bool VN100Init()
{
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
        3,    // spiChecksum    - Append SPI messages with a 16-bit CRC.
        0,    // errorMode      - N/A, SPI used.
    };
            
    return VN100WriteReg( VN100_REG_PROT, (void*) &reg_prot_val );
}


//==============================================================================

void VN100LANBuildData( void )
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
static bool VN100WriteReg( VN100_REG_E reg, void* regData )
{
    static const uint8_t  VN100_CMD_WRITE_REG = 0x02;
    static       SPI_XFER vn100spi;
                 bool     writeSuccess = false;
    
    static enum
    {
        SM_REQUEST_START,
        SM_REQUEST_FINISH,
        SM_WAIT,
        SM_RESPONSE_START,
        SM_RESPONSE_FINISH,
                
    } writeRegState = SM_REQUEST_START;
    
    switch( writeRegState )
    {
        case SM_REQUEST_START:
        {
            uint16_t regLen = vn100_reg_cfg[ reg ].regLen;
            uint16_t calcCRC;
            
            // Build header field.
            vn100XferBuf[ 0 ] = VN100_CMD_WRITE_REG;
            vn100XferBuf[ 1 ] = vn100_reg_cfg[ reg ].regID;
            vn100XferBuf[ 2 ] = 0;
            vn100XferBuf[ 3 ] = 0;
            
            // Copy payload to transfer buffer.
            memcpy( &vn100XferBuf[ VN100_SPI_HEADER_SIZE ], regData, regLen );
            
            // Calculate CRC of packet.
            calcCRC = utilCRC16( (void*) &vn100XferBuf[ 0 ], 
                                 VN100_SPI_HEADER_SIZE + regLen,
                                 0 );
            
            // Append calculated CRC to packet.
            memcpy( &vn100XferBuf[ VN100_SPI_HEADER_SIZE + regLen ], calcCRC, VN100_SPI_CRC_SIZE );

            // Set up VN100 packet for SPI transfer.
            vn100spi.port   = VN100_SPI_PORT;
            vn100spi.rxBuf  = NULL;
            vn100spi.txBuf  = &vn100XferBuf[ 0 ];
            vn100spi.length = VN100_SPI_HEADER_SIZE + regLen + VN100_SPI_CRC_SIZE;

            // Message successfully queued for transfer ?
            if( SPIXfer( &vn100spi ) == 0 )
            {
                writeRegState++;
            }
            break;
        }
        case SM_REQUEST_FINISH:
        {
            // SPI transfer of Request Packet is complete ?
            if (vn100spi.xferDone != 0)
            {
                writeRegState++;
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
                writeRegState++;
                
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
            uint16_t regLen = vn100_reg_cfg[ reg ].regLen;
            
            // Set up VN100 packet for SPI transfer.
            vn100spi.port   = VN100_SPI_PORT;
            vn100spi.rxBuf  = &vn100XferBuf[ 0 ];
            vn100spi.txBuf  = NULL;
            vn100spi.length = VN100_SPI_HEADER_SIZE + 
                              regLen +
                              VN100_SPI_VPE_SIZE + 
                              VN100_SPI_CRC_SIZE;

            // Message successfully queued for transfer ?
            if( SPIXfer( &vn100spi ) == 0 )
            {
                writeRegState++;
            }
            break;
        }
        case SM_RESPONSE_FINISH:
        {
            // SPI transfer of Response Packet is complete ?
            if( vn100spi.xferDone != 0 )
            {
                // Calculate received data CRC.
                uint16_t calcCRC = utilCRC16( (void*) &vn100XferBuf[ 0 ], 
                                              VN100_SPI_HEADER_SIZE + regLen + VN100_SPI_VPE_SIZE + VN100_SPI_CRC_SIZE,
                                              0 );
                
                // All of the following data is correct ?
                //  - Command matches that transmitted.
                //  - Register ID matches that transmitted.
                //  - Error ID = 0, i.e. no errors occurred.
                //  - Data is valid, i.e. CRC is correct.
                //
                if( ( vn100XferBuf[ 1 ] == VN100_CMD_WRITE_REG        ) &&
                    ( vn100XferBuf[ 2 ] == vn100_reg_cfg[ reg ].regID ) &&
                    ( vn100XferBuf[ 3 ] == 0                          ) &&
                    ( calcCRC           == 0                          ) )
                {
                    writeSuccess = true;
                }
                
                // Re-start state machine.
                writeRegState = SM_REQUEST_START;
            }
            break;
        }
    }
    return writeSuccess;
}


//==============================================================================

static bool VN100ReadReg( VN100_REG_E reg )
{
    static const uint8_t  VN100_CMD_READ_REG = 0x01;
    static       SPI_XFER vn100spi;
                 bool     readSuccess = false;

    static enum
    {
        SM_REQUEST_START,
        SM_REQUEST_FINISH,
        SM_WAIT,
        SM_RESPONSE_START,
        SM_RESPONSE_FINISH,
                
    } readRegState = SM_REQUEST_START;

    switch( readRegState )
    {
        case SM_REQUEST_START:
        {
            uint16_t regLen = vn100_reg_cfg[ reg ].regLen;
            uint16_t calcCRC;
            
            // Build header field.
            vn100XferBuf[ 0 ] = VN100_CMD_READ_REG;
            vn100XferBuf[ 1 ] = vn100_reg_cfg[ reg ].regID;
            vn100XferBuf[ 2 ] = 0;
            vn100XferBuf[ 3 ] = 0;
            
            // Calculate CRC of packet.
            calcCRC = utilCRC16( (void*) &vn100XferBuf[ 0 ], 
                                 VN100_SPI_HEADER_SIZE,
                                 0 );
            
            // Append calculated CRC to packet.
            memcpy( &vn100XferBuf[ VN100_SPI_HEADER_SIZE ], calcCRC, VN100_SPI_CRC_SIZE );

            // Set up VN100 packet for SPI transfer.
            vn100spi.port   = VN100_SPI_PORT;
            vn100spi.rxBuf  = NULL;
            vn100spi.txBuf  = &vn100XferBuf[ 0 ];
            vn100spi.length = VN100_SPI_HEADER_SIZE + regLen + VN100_SPI_CRC_SIZE;

            // Message successfully queued for transfer ?
            if( SPIXfer( &vn100spi ) == 0 )
            {
                writeRegState++;
            }
            break;
            
            
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

// Calling function should set up elements:
//  - arg
//  - cmdID
//  - reqPl
//  - reqPlLen
//  - respPlLen
// This function will populate the following elements based upon the response
// data received.
//  - respPl
//  - respStat
//  - respCnt
//
// Additionally the function returns a Boolean flag to identify successful
// execution of the transfer.
//
typedef struct
{
    uint8_t     arg;
    uint8_t     cmdID;
    
    uint8_t*    reqPl;
    uint16_t    reqPlLen;
    
    uint8_t*    respPl;
    uint16_t    respPlLen;
    uint16_t    respStat;
    uint16_t    respCnt;
    
} VN100_COMM_PKT;

static bool VN100Comm( VN100_COMM_PKT* pkt )
{   
    static const SPI_PORT VN100_SPI_PORT      = SPI_PORT_SPI2;
    static const uint8_t  VN100_CMD_WRITE_REG = 0x02;
    static       SPI_XFER vn100spi;
                 bool     successFlag         = false;
    
    static enum
    {
        SM_REQUEST_START,
        SM_REQUEST_FINISH,
        SM_WAIT,
        SM_RESPONSE_START,
        SM_RESPONSE_FINISH,
                
    } writeRegState = SM_REQUEST_START;
    
    switch( writeRegState )
    {
        case SM_REQUEST_START:
        {
            uint16_t calcCRC;
            
            // Build header field.
            vn100XferBuf[ 0 ] = pkt->arg;
            vn100XferBuf[ 1 ] = pkt->cmdID;
            vn100XferBuf[ 2 ] = 0;
            vn100XferBuf[ 3 ] = 0;
            
            // Copy payload to transfer buffer.
            memcpy( &vn100XferBuf[ VN100_SPI_HEADER_SIZE ], pkt->reqPl, pkt->reqPlLen );
            
            // Calculate CRC of packet.
            calcCRC = utilCRC16( (void*) &vn100XferBuf[ 0 ], 
                                 mVN100_SPI_REQ_CRC_OFFSET( pkt->reqPlLen ),
                                 0 );
            
            // Append calculated CRC to packet.
            memcpy( &vn100XferBuf[ mVN100_SPI_REQ_CRC_OFFSET( pkt->reqPlLen ) ],
                    calcCRC,
                    VN100_SPI_CRC_SIZE );

            // Set up VN100 packet for SPI transfer.
            vn100spi.port   = VN100_SPI_PORT;
            vn100spi.rxBuf  = NULL;
            vn100spi.txBuf  = &vn100XferBuf[ 0 ];
            vn100spi.length = mVN100_SPI_REQ_PKT_SIZE( pkt->reqPlLen );

            // Message successfully queued for transfer ?
            if( SPIXfer( &vn100spi ) == 0 )
            {
                writeRegState++;
            }
            break;
        }
        case SM_REQUEST_FINISH:
        {
            // SPI transfer of Request Packet is complete ?
            if (vn100spi.xferDone != 0)
            {
                writeRegState++;
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
                writeRegState++;
                
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
            vn100spi.length = mVN100_SPI_RESP_PKT_SIZE( pkt->respPlLen );

            // Message successfully queued for transfer ?
            if( SPIXfer( &vn100spi ) == 0 )
            {
                writeRegState++;
            }
            break;
        }
        case SM_RESPONSE_FINISH:
        {
            // SPI transfer of Response Packet is complete ?
            if( vn100spi.xferDone != 0 )
            {
                // Calculate received data CRC.
                uint16_t calcCRC = utilCRC16( (void*) &vn100XferBuf[ 0 ], 
                                              mVN100_SPI_RESP_PKT_SIZE( pkt->respPlLen ),
                                              0 );
                
                // All of the following data is correct ?
                //  - Argument matches that transmitted.
                //  - command ID matches that transmitted.
                //  - Error ID = 0, i.e. no errors occurred.
                //  - Data is valid, i.e. received CRC is correct.
                //
                if( ( vn100XferBuf[ 1 ] == pkt->arg   ) &&
                    ( vn100XferBuf[ 2 ] == pkt->cmdID ) &&
                    ( vn100XferBuf[ 3 ] == 0          ) &&
                    ( calcCRC           == 0          ) )
                {
                    // Identify the transfer as successful.
                    successFlag = true;
                    
                    // Populate the received response payload.
                    memcpy( pkt->respPl,
                            &vn100XferBuf[ VN100_SPI_HEADER_SIZE ],
                            pkt->respPlLen );
                    
                    // Populate the received Serial Status.
                    memcpy( pkt->respStat,
                            &vn100XferBuf[ mVN100_SPI_RESP_STATUS_OFFSET( pkt->respPlLen ) ],
                            VN100_SPI_STATUS_SIZE );
                    
                    // Populate the received Serial Count.
                    memcpy( pkt->respCnt,
                            &vn100XferBuf[ mVN100_SPI_RESP_COUNT_OFFSET( pkt->respPlLen ) ],
                            VN100_SPI_COUNT_SIZE );
                }
                
                // Re-start state machine.
                writeRegState = SM_REQUEST_START;
            }
            break;
        }
    }
    
    return successFlag;
}