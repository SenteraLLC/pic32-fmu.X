/*******************************************************************************
/
/   Filename:   fmucomm.h
/
*******************************************************************************/

#ifndef FMUCOMM_H
#define	FMUCOMM_H

#include "stdtypes.h"
#include "stdbool.h"

// Typedefs ====================================================================

typedef enum
{
    FMUCOMM_TYPE_HOST_HEARTBEAT,      // Host to FMU
    FMUCOMM_TYPE_CTRL_SURFACE_CMD,    // Host to FMU
    FMUCOMM_TYPE_HOST_EXCEPTION,      // Host to FMU
    FMUCOMM_TYPE_FMU_HEARTBEAT,       // FMU to Host
    FMUCOMM_TYPE_IMU_DATA,            // FMU to Host
    FMUCOMM_TYPE_GPS_DATA,            // FMU to Host
    FMUCOMM_TYPE_AIR_DATA,            // FMU to Host
    FMUCOMM_TYPE_CTRL_SURFACE_DATA,   // FMU to Host
    FMUCOMM_TYPE_FMU_EXCEPTION,       // FMU to Host
} FMUCOMM_TYPE_E;

typedef struct
{
    uint32_t fwVersion;     // Firmware version ID.
    uint32_t hwVersion;     // Hardware version ID.
    uint32_t serialNum;     // Serial number.
    uint32_t msUptime;      // System uptime in milliseconds.
} FMUCOMM_HOST_HEARTBEAT_PKT;

typedef struct
{
    uint8_t surfaceID;      // Control surface ID.
    int16_t position;       // Control surface position in 1e3 radians.
    //... (repeated for additional surfaces)
} FMUCOMM_CTRL_SURFACE_CMD_PKT;

typedef struct
{
    uint8_t debugData[1024];
} FMUCOMM_HOST_EXCEPTION_PKT;

typedef struct
{
    uint32_t fwVersion;     // Firmware version ID.
    uint32_t hwVersion;     // Hardware version ID.
    uint32_t serialNum;     // Serial number.
    uint32_t msUptime;      // System uptime in milliseconds.
    uint16_t inputVoltage;  // Input voltage in millivolts.
    int16_t  boardTemp;     // Board temperature in 1e2 degrees Celsius.
} FMUCOMM_FMU_HEARTBEAT_PKT;

typedef struct __attribute__ ((packed)) 
{
    uint64_t fmuTime;       // FMU timestamp in microseconds.
    uint16_t imuType;       // IMU type: 0 = VN-100, 1 = MPU-9150
    
    struct
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
} FMUCOMM_IMU_DATA;

typedef struct
{
    uint64_t fmuTime;       // FMU timestamp in nanosecones.
    uint16_t gpsType;       // GPS type: 0 = Novatel OEMStar, 1 = U-blox
    uint8_t  gpsData[1500]; // GPS receiver data.
} FMUCOMM_GPS_DATA_PKT;

typedef struct
{
    uint8_t airDataID;      // Air data ID.
    float   staticPress;    // Static pressure in kPa.
    float   dynamicPress;   // Dynamic pressure in kPa.
    float   temperature;    // Temperature in degrees Celsius.
} FMUCOMM_AIR_DATA_PKT;

typedef struct
{
    uint8_t  surfaceID;     // Control surface ID.
    int16_t  cmdPosition;   // Commanded surface position.
    int16_t  actPosition;   // Actual surface position.
    uint16_t inputVoltage;  // Input voltage in millivolts.
    uint16_t inputCurrent;  // Input current in milliamps.
} FMUCOMM_CTRL_SURFACE_DATA_PKT;

typedef struct
{
    uint8_t debugData[1024];
} FMUCOMM_FMU_EXCEPTION_PKT;


// Function Prototypes =========================================================

void FMUCommTask();

bool FMUCommSet( FMUCOMM_TYPE_E pktType, uint8_t* pl_p, uint16_t plLen );

#endif	/* FMUCOMM_H */

