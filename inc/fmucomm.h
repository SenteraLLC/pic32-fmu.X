////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Flight Management Unit (FMU) Ethernet Communication.
////////////////////////////////////////////////////////////////////////////////

#ifndef FMUCOMM_H_
#define	FMUCOMM_H_

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

#include "sbus.h"

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

#define FMUCOMM_GPS_DATA_LEN_C 800U

// Received (Host -> FMU) messages.
typedef enum
{
    FMUCOMM_TYPE_HOST_HEARTBEAT,
    FMUCOMM_TYPE_CTRL_SURFACE_CMD,
    FMUCOMM_TYPE_GPS_CMD,
    FMUCOMM_TYPE_HOST_EXCEPTION,
    
    FMUCOMM_RX_TYPE_MAX,
            
} FMUCOMM_RX_TYPE_E;

// Transmitted (FMU -> Host) messages.
typedef enum
{   
    FMUCOMM_TYPE_FMU_HEARTBEAT,     
    FMUCOMM_TYPE_IMU_DATA,          
    FMUCOMM_TYPE_GPS_DATA,                  
    FMUCOMM_TYPE_CTRL_SURFACE_DATA, 
    FMUCOMM_TYPE_RC_DATA,       
    FMUCOMM_TYPE_FMU_EXCEPTION,   
            
    FMUCOMM_TX_TYPE_MAX,
            
} FMUCOMM_TX_TYPE_E;

// Communication protocol wrapper content.  Messages are wrapped with a 
// header (header0, header1, header2, type, and length) and footer (CRC) field.
typedef struct
{
    uint8_t   header0;
    uint8_t   header1;
    uint8_t   header2;
    uint8_t   type;
    
    union
    {
        struct
        {
            uint8_t lengthLsb;
            uint8_t lengthMsb;
        };
        
        uint16_t length;
    };
    
    union
    {
        struct
        {
            uint8_t crcLsb;
            uint8_t crcMsb;
        };
        
        uint16_t crc;
    };
    
} FMUCOMM_PKT_WRAP;

// Definition of received packet.
typedef struct
{
    // Flag identifying if the received packet is valid.
    //  true  = packet is valid
    //  false = packet is invalid; no received for the execution cycle
    bool valid;
    
    FMUCOMM_PKT_WRAP wrap;
    
    uint8_t* pl_p;
    
} FMUCOMM_RX_PKT;

//
// Received (HOST -> FMU) Message Payload Definition ---------------------------
//

typedef struct __attribute__ ((packed))
{
    union
    {
        struct __attribute__ ((packed))
        {
            uint8_t	 fwVersionRev;  // Firmware revision version ID.
            uint8_t	 fwVersionMin;  // Firmware minor version ID.
            uint8_t	 fwVersionMaj;  // Firmware major version ID.
            uint8_t	 hwVersionRev;  // Hardware revision version ID.
            uint8_t	 hwVersionMin;  // Hardware minor version ID.
            uint8_t	 hwVersionMaj;  // Hardware major version ID.
            uint32_t serialNum;     // Serial number.
            uint32_t msUptime;      // System uptime in milliseconds.
        };
        
        uint8_t pl_u8[ 14 ];
    };
    
} FMUCOMM_HOST_HEARTBEAT_PL;

typedef struct __attribute__ ((packed))
{
    uint8_t  surfaceID;
    uint8_t  cmdType;
    uint16_t cmdPwm;
    uint16_t cmdPos;
    
} FMUCOMM_CTRL_SURFACE_CMD_PL_FIELD;

typedef struct __attribute__ ((packed))
{
    union
    {
        FMUCOMM_CTRL_SURFACE_CMD_PL_FIELD ctrlSurface[ 10 ];
        
        uint8_t pl_u8[ 60 ];
    };
            
} FMUCOMM_CTRL_SURFACE_CMD_PL;

typedef struct __attribute__ ((packed))
{    
    union
    {
        uint8_t GPSData[ 1024 ];
        
        uint8_t pl_u8[ 1024 ];
    };
    
} FMUCOMM_HOST_GPS_CMD_PL;

typedef struct __attribute__ ((packed))
{
    union
    {
        uint8_t debugData[ 1024 ];
        
        uint8_t pl_u8[ 1024 ];
    };
    
} FMUCOMM_HOST_EXCEPTION_PL;

//
// Transmitted (FMU -> HOST) Message Payload Definition ------------------------
//

typedef struct __attribute__ ((packed))
{
    uint8_t	 fwVersionRev;  // Firmware revision version ID.
    uint8_t	 fwVersionMin;  // Firmware minor version ID.
    uint8_t	 fwVersionMaj;  // Firmware major version ID.
    uint8_t	 hwVersionRev;  // Hardware revision version ID.
    uint8_t	 hwVersionMin;  // Hardware minor version ID.
    uint8_t	 hwVersionMaj;  // Hardware major version ID.
    uint32_t serialNum;     // Serial number.
    uint32_t msUptime;      // System uptime in milliseconds.
    uint16_t inputVoltage;  // Input voltage in millivolts.
    int16_t  boardTemp;     // Board temperature in 1e2 degrees Celsius.
} FMUCOMM_FMU_HEARTBEAT_PL;

typedef struct __attribute__ ((packed)) 
{
    uint64_t fmuTime;       // FMU timestamp in microseconds.
    uint16_t imuType;       // IMU type: 0 = VN-100, 1 = MPU-9150
    
    struct __attribute__ ((packed))
    {
        uint16_t mag    :  1;   // bits       0
        uint16_t accel  :  1;   // bits       1
        uint16_t gyro   :  1;   // bits       2
        uint16_t temp   :  1;   // bits       3
        uint16_t press  :  1;   // bits       4
        uint16_t att    :  1;   // bits       5
        uint16_t        : 10;   // bits  6 - 15 (reserved)
        
    } imuValid;             // Set as 1 if data is valid, 0 otherwise.

    uint32_t imuTimeSyncIn; // Elapsed time between last IMU sync pulse trigger
                            // and sampling of IMU measurements.
    float    magX;          // Uncompensated magnetic X-axis in Gauss.
    float    magY;          // Uncompensated magnetic Y-axis in Gauss.
    float    magZ;          // Uncompensated magnetic Z-axis in Gauss.
    float    accelX;        // Uncompensated acceleration X-axis in m/s/s.
    float    accelY;        // Uncompensated acceleration Y-axis in m/s/s.
    float    accelZ;        // Uncompensated acceleration Z-axis in m/s/s.
    float    gyroX;         // Uncompensated angular rate X-axis in rad/s.
    float    gyroY;         // Uncompensated angular rate Y-axis in rad/s.
    float    gyroZ;         // Uncompensated angular rate Z-axis in rad/s.
    float    temp;          // IMU temperature in degrees Celsius.
    float    pressure;      // Barometric pressure in kPa.
    
    uint32_t attTimeSyncIn; // Elapsed time between last IMU sync pulse trigger
                            // and sampling of attitude measurements.
    float    yaw;           // Estimated yaw attitude in degrees.
    float    pitch;         // Estimated pitch attitude in degrees.
    float    roll;          // Estimated roll attitude in degrees.
} FMUCOMM_IMU_DATA_PL;

typedef struct __attribute__ ((packed))
{
    uint64_t fmuTime;                          // FMU timestamp in nanosecones.
    uint16_t gpsType;                          // GPS type: 0 = Novatel OEMStar, 1 = U-blox
    uint8_t  gpsData[FMUCOMM_GPS_DATA_LEN_C];  // GPS receiver data.
} FMUCOMM_GPS_DATA_PL;

typedef struct __attribute__ ((packed))
{
    uint8_t  surfaceID;     // Control surface ID.
    uint16_t cmdTypeEcho;   // Echo of the command type used.
    uint16_t actPwm;        // Actual surface position.
    uint16_t inputVoltage;  // Input voltage in millivolts.
    uint16_t inputCurrent;  // Input current in milliamps.
    int16_t  vsense1Cor;    // The calibration corrected value of V_SENSE1.
    int16_t  vsense2Cor;    // The calibration corrected value of V_SENSE2.
} FMUCOMM_CTRL_SURFACE_DATA_PL_FIELD;

typedef struct __attribute__ ((packed))
{
    FMUCOMM_CTRL_SURFACE_DATA_PL_FIELD ctrlSurface[ 10 ];
            
} FMUCOMM_CTRL_SURFACE_DATA_PL;

typedef struct __attribute__ ((packed))
{
    uint16_t chVal[ SBUS_CH_MAX ];
    
} FMUCOMM_CH_DATA_PL;

typedef struct __attribute__ ((packed))
{
    uint8_t debugData[1024];
} FMUCOMM_FMU_EXCEPTION_PL;

// *****************************************************************************
// ************************** Declarations *************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

// Service the FMUComm module.  Communication channels are setup, and received
// Ethernet data is processed.
void FMUCommTask();

// Queue Ethernet data for transmission.
bool FMUCommSet( FMUCOMM_TX_TYPE_E pktType, uint8_t* pl_p, uint16_t plLen );

// Get received Ethernet data.
const FMUCOMM_RX_PKT* FMUCommGet( FMUCOMM_RX_TYPE_E pktType );

#endif	// FMUCOMM_H_

