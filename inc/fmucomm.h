/*******************************************************************************
/
/   Filename:   fmucomm.h
/
*******************************************************************************/

#ifndef FMUCOMM_H
#define	FMUCOMM_H



// Typedefs ====================================================================

typedef enum
{
    FMUCOMM_TYPE_HOST_HEARTBEAT     = 0x00,     // Host to FMU
    FMUCOMM_TYPE_CTRL_SURFACE_CMD   = 0x01,     // Host to FMU
    FMUCOMM_TYPE_HOST_EXCEPTION     = 0x7F,     // Host to FMU
    FMUCOMM_TYPE_FMU_HEARTBEAT      = 0x80,     // FMU to Host
    FMUCOMM_TYPE_IMU_DATA           = 0x81,     // FMU to Host
    FMUCOMM_TYPE_GPS_DATA           = 0x82,     // FMU to Host
    FMUCOMM_TYPE_AIR_DATA           = 0x83,     // FMU to Host
    FMUCOMM_TYPE_CTRL_SURFACE_DATA  = 0x84,     // FMU to Host
    FMUCOMM_TYPE_FMU_EXCEPTION      = 0xFF,     // FMU to Host
} FMUCOMM_TYPE_E;

typedef struct
{
    uint8_t  header0;
    uint8_t  header1;
    uint8_t  header2;
    FMUCOMM_TYPE_E type;
    uint16_t length;
    uint8_t  payload;
    uint16_t crc;
};

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

typedef struct
{
    uint64_t fmuTime;       // FMU timestamp in nanoseconds.
    uint16_t imuType;       // IMU type: 0 = VN-100, 1 = MPU-9150
    uint16_t imuValid;      // Set as 1 if data is valid, 0 otherwise.
                            //   Bit 0: Mag
                            //   Bit 1: Accel
                            //   Bit 2: Gyro
                            //   Bit 3: Temp
                            //   Bit 4: Press
                            //   Bit 5: AttitudeBit 2: Gyro
                            //   Bit 6-15: Reserved
    uint64_t timeStartup;   // Time since IMU startup.
    uint64_t timeSyncIn;    // Time since last IMU sync pulse trigger.
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
    float    yaw;           // Estimated yaw attitude in degrees.
    float    pitch;         // Estimated pitch attitude in degrees.
    float    roll;          // Estimated roll attitude in degrees.
} FMUCOMM_IMU_DATA_PKT;

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




#endif	/* FMUCOMM_H */

