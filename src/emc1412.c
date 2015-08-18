////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief EMC1412 Temperature Sensor driver.
////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

#include "emc1412.h"
#include "i2c.h"
#include "coretime.h"

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

#define EMC1412_I2C_ADDR    (0x4Cu)     // 7-bit I2C slave address.
#define EMC1412_I2C_FREQ    (10000)     // 10kHz

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

static int32_t intTemp = 0;     // Internal diode temperature (deg C x 1E3).
static int32_t extTemp = 0;     // External diode temperature (deg C x 1E3).

static volatile uint8_t prodIDReg = 0;           // Address 0xFD data.
static volatile uint8_t statusReg = 0;           // Address 0x02 data.
static volatile uint8_t intDiodeHiByteReg = 0;   // Address 0x00 data.
static volatile uint8_t intDiodeLoByteReg = 0;   // Address 0x29 data.
static volatile uint8_t extDiodeHiByteReg = 0;   // Address 0x01 data.
static volatile uint8_t extDiodeLoByteReg = 0;   // Address 0x10 data.

typedef struct {
    uint8_t regAddr;
    void*   writePtr;
} EMC1412_REG;

static const EMC1412_REG emc1412RegValues[6] =
{
    {.regAddr = 0xFD, .writePtr = (uint8_t*)&prodIDReg},
    {.regAddr = 0x02, .writePtr = (uint8_t*)&statusReg},
    {.regAddr = 0x00, .writePtr = (uint8_t*)&intDiodeHiByteReg},
    {.regAddr = 0x29, .writePtr = (uint8_t*)&intDiodeLoByteReg},
    {.regAddr = 0x01, .writePtr = (uint8_t*)&extDiodeHiByteReg},
    {.regAddr = 0x10, .writePtr = (uint8_t*)&extDiodeLoByteReg},
};

static const uint8_t configData = 0b00000100;   // Bit 2 = 1
                                                // Temperature range
                                                // -65'C to + 191.875'C

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

void EMC1412Task( void )
{
    static enum
    {
        EMC1412_CONFIG,
        EMC1412_CONFIG_ACK,
        EMC1412_DELAY,
        EMC1412_INIT_RX,
        EMC1412_RX,
        EMC1412_RESULT,
                
    } EMC1412TaskState = EMC1412_CONFIG;
    
    static I2C_DRV_STATUS status;

    static I2C_TRANSFER xfer =
    {
        .dev        = EMC1412_I2C_ADDR,
        .freq       = EMC1412_I2C_FREQ,
        .rw         = 0,
        .addr       = 0,
        .numBytes   = 0,
        .buf        = 0,
        .status     = &status,
    };

    static uint16_t emc1412RegIndex = 0;

    switch( EMC1412TaskState )
    {
        case EMC1412_CONFIG:
        {
            xfer.rw       = 0;                     // Write data transfer.
            xfer.addr     = 0x03;                  // Configuration register.
            xfer.numBytes = 1;                     // One byte transfer.
            xfer.buf      = (uint8_t*)&configData; // Register configuration.

            // Try to kick off I2C write. Kickoff successful ?
            if( I2CXfer( &xfer ) == 1 )
            {
                EMC1412TaskState++;
            }
            
            break;
        }
        case EMC1412_CONFIG_ACK:
        {
            // I2C write of configuration done ?
            if (*xfer.status == I2C_DRV_STATUS_DONE)
            {
                EMC1412TaskState++;
            }
            // Error occurred with I2C write of configuration ?
            else if (*xfer.status == I2C_DRV_STATUS_ERROR)
            {
                // Retry configuration.
                EMC1412TaskState = EMC1412_CONFIG;  
            }
            
            break;
        }
        case EMC1412_DELAY:
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
                EMC1412TaskState++;
                
                // Latch the execution time for evaluation on next delay.
                prevExeTime = CoreTime32usGet();
            }
            
            break;
        }
        case EMC1412_INIT_RX:
        {
            // Configure I2C xfer struct to read single byte registers.
            xfer.rw       = 1;
            xfer.addr     = emc1412RegValues[ emc1412RegIndex ].regAddr;
            xfer.numBytes = 1;
            xfer.buf      = emc1412RegValues[ emc1412RegIndex ].writePtr;
            
            EMC1412TaskState++;
            
            break;
        }
        case EMC1412_RX:
        {
            // Try to kick off I2C read. Kickoff successful ?
            if( I2CXfer( &xfer ) == 1 )
            {
                EMC1412TaskState++;
            }
            
            break;
        }
        case EMC1412_RESULT:
        {
            // I2C read finished ?
            if( *xfer.status == I2C_DRV_STATUS_DONE )
            {
                emc1412RegIndex++;
                
                // All EMC1412 registers read for Task execution cycle ?
                if( emc1412RegIndex >= ( sizeof( emc1412RegValues ) / 
                                         sizeof( EMC1412_REG      ) ) )
                {
                    // Verify successful transfer of constant product ID
                    // register, then update the temperature measurements.
                    if( prodIDReg == 0x20 )
                    {
                        intTemp = ( ( ( intDiodeHiByteReg << 3 ) |
                                      ( intDiodeLoByteReg >> 5 ) ) * 125 ) - 64000;
                        
                        extTemp = ( ( ( extDiodeHiByteReg << 3 ) |
                                      ( extDiodeLoByteReg >> 5 ) ) * 125 ) - 64000;
                    }

                    // Setup state machine for next Task execution cycle.
                    emc1412RegIndex  = 0;
                    EMC1412TaskState = EMC1412_DELAY;
                }
                // Additional EMC1412 registers to read for the Task execution
                // cycle.
                else
                {
                    // Perform read sequence of next register.
                    EMC1412TaskState = EMC1412_INIT_RX;
                }
            }
            // Error with I2C read operation ?
            else if (*xfer.status == I2C_DRV_STATUS_ERROR)
            {
                // Restart register read sequence.
                emc1412RegIndex  = 0;
                EMC1412TaskState = EMC1412_INIT_RX;
            }
            
            break;
        }
    }
    return;
}

int16_t EMC1412IntTempGet( void )
{
    return ( intTemp / 10 );
}

int16_t EMC1412ExtTempGet( void )
{
    return( extTemp / 10 );
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************







