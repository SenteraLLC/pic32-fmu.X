/*******************************************************************************
/
/   Filename:   vn100.h
/
*******************************************************************************/


#ifndef VN100_H
#define	VN100_H

#include "stdtypes.h"


//==============================================================================

#define VN100_MAX_PAYLOAD_SIZE  100     // Maximum payload size is 100 bytes.

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


//==============================================================================

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
} VN100_SPI_PKT;


//==============================================================================

// Model Number Register (ID 1)

typedef struct __attribute__ ((packed)) {
    UINT_8  productName[24];    // Product name.  Max 24 characters.
} VN100_MODEL_NUMBER;

// Hardware Revision (ID 2)

typedef struct __attribute__ ((packed)) {
    UINT_32 revision;           // Hardware revision.
} VN100_HARDWARE_REVISION;

// Serial Number Register (ID 3)

typedef struct __attribute__ ((packed)) {
    UINT_32 serialNum;          // Serial number (32-bit unsigned integer).
} VN100_SERIAL_NUMBER;

// Firmware Version Register (ID 4)

typedef struct __attribute__ ((packed)) {
    UINT_8   majorVersion;      // Major release version of firmware.
    UINT_8   minorVersion;      // Minor release version of firmware.
    UINT_8   featureVersion;    // Feature release version of firmware.
    UINT_8   hotFix;            // Hot fix number. Numbers above 100 are
                                //   reserved for custom firmware version.
} VN100_FIRMWARE_VERSION;

// Communication Protocol Control (ID 30)

typedef struct __attribute__ ((packed)) {
    UINT_8  serialCount;        // Append counter or time to serial messages.
    UINT_8  serialStatus;       // Append the status to serial messages.
    UINT_8  spiCount;           // Append counter to SPI messages.
    UINT_8  spiStatus;          // Append status to SPI messages.
    UINT_8  serialChecksum;     // Choose serial checksum type.
    UINT_8  spiChecksum;        // Choose SPI checksum type.
    UINT_8  errorMode;          // Choose action when errors are generated.
} VN100_COMM_PROTO_CTRL;

// IMU Measurements (ID 54)

typedef struct __attribute__ ((packed)) {
    FP_32   magX;               // Uncompensated magnetic X-axis.
    FP_32   magY;               // Uncompensated magnetic Y-axis.
    FP_32   magZ;               // Uncompensated magnetic Z-axis.
    FP_32   accelX;             // Uncompensated acceleration X-axis.
    FP_32   accelY;             // Uncompensated acceleration Y-axis.
    FP_32   accelZ;             // Uncompensated acceleration Z-axis.
    FP_32   gyroX;              // Uncompensated angular rate X-axis.
    FP_32   gyroY;              // Uncompensated angular rate Y-axis.
    FP_32   gyroZ;              // Uncompensated angular rate Z-axis.
    FP_32   temp;               // IMU temperature.
    FP_32   pressure;           // Barometric pressure.
} VN100_IMU_MEASUREMENTS;


// Function Prototypes =========================================================

void VN100Task();



#endif	/* VN100_H */

