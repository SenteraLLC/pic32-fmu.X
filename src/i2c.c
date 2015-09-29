////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Inter-Integrated Circuit (I2C) driver.
////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

#include <plib.h>

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

#include "system.h"
#include "i2c.h"
#include "ts.h"

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

// A write to a LAT register latches data to corresponding port I/O pins.
//  Those I/O port pin(s) configured as outputs are updated.

// A read from LAT register reads the data held in PORT data latch, not
//  from the port I/O pins.

#define mGetI2CClk()        (PORTBbits.RB3)     // Get Clock.
#define mGetI2CData()       (PORTBbits.RB4)     // Get Data.

#define mGetI2CClkLat()     (LATBbits.LATB3)    // Get state of Clock latch.
#define mGetI2CDataLat()    (LATBbits.LATB4)    // Get state of Data latch.

#define mSetI2CClkHigh()    do {LATBSET = _LATB_LATB3_MASK;} while(0)
#define mSetI2CClkLow()     do {LATBCLR = _LATB_LATB3_MASK;} while(0)

#define mSetI2CDataHigh()   do {LATBSET = _LATB_LATB4_MASK;} while(0)
#define mSetI2CDataLow()    do {LATBCLR = _LATB_LATB4_MASK;} while(0)

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

static volatile I2C_TRANSFER *ptr;
static volatile int error;              // 0 = normal, 1 = error condition

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

int16_t I2CXfer(I2C_TRANSFER *xfer)
{
    if ((xfer->buf == 0)  || (xfer->status == 0))
    {
        // Null pointer error.
        return -1;
    }

    // Set I2C driver status as waiting for bus.
    *xfer->status = I2C_DRV_STATUS_WAIT;

    // Take control of software I2C bus if it's available. ----------

    asm("di");              // Entering critical section: disable interrupts.
    if (IEC0bits.T4IE == 1)
    {
        asm("ei");          // Exiting critical section: enable interrupts.
        return 0;           // Software I2C bus is busy.
    }

    ptr = xfer;
    
    // Configure the software I2C timer for desired clock frequency.
    // 4 interrupts per clock cycle.
    if ((GetPeripheralClock() % (xfer->freq * 4)) != 0)
    {
        // No perfect frequency divisor possible.
        // Round up to next period number to be conservative.
        WritePeriod4((GetPeripheralClock() / (xfer->freq * 4)) + 1);
    }
    else
    {
        // Perfect frequency match.
        WritePeriod4(GetPeripheralClock() / (xfer->freq * 4));
    }

    // Reset error.
    error = 0;

    // Set I2C driver status as busy.
    *xfer->status = I2C_DRV_STATUS_BUSY;

    TMR4 = 0;
    mT4ClearIntFlag();
    mT4IntEnable(1);        // Initiate software I2C transfer.
    asm("ei");              // Exiting critical section: enable interrupts.

    return 1;
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************

////////////////////////////////////////////////////////////////////////////////
/// @brief  I2C bit-bang processing task.
///
/// The I2C data is bit-banged on the communication channel.
////////////////////////////////////////////////////////////////////////////////
void __ISR (_TIMER_4_VECTOR, IPL6SOFT) Timer4ISR(void)
{
    static enum {
        START       = 0,    // <-- Start of I2C transfer.
        ADDRESS_W   = 1,
        CHECK_ACK1  = 2,
        REGISTER_W  = 3,
        CHECK_ACK2  = 4,

        RESTART     = 5,    // <-- Beginning of read sequence.
        ADDRESS_R   = 6,
        CHECK_ACK3  = 7,
        READ_BYTE   = 8,
        WRITE_ACK   = 9,
        NACK        = 10,

        WRITE_BYTE  = 12,   // <-- Beginning of write sequence.
        CHECK_ACK4  = 13,

        STOP        = 11    // <-- End of I2C transfer.
    } i2cState = START;
    
    static int iState = 0;
    static int ack = 0;
    static uint8_t value = 0;
    static uint8_t i2cBytesRead = 0;
    static uint8_t i2cBytesWritten = 0;

ts_start(I2C_T4_ISR);
    
    // Delay in case of clock stretching.
    if (mGetI2CClkLat() && !mGetI2CClk())
    {
        mT4ClearIntFlag();
        return;
    }

    // Each case has discrete states (iState):
    //  (iState % 4) == 0: (Clock Low) Toggle Data
    //  (iState % 4) == 1: Set Clock High
    //  (iState % 4) == 2: (Clock High) Commit Data
    //  (iState % 4) == 3: Set Clock Low

    switch (i2cState)
    {
        //-----------------------------------------------------------
        case START:
        case RESTART:
        {
            // Start condition: Data transitions high-to-low 
            //                  while clock is high.
            switch (iState)
            {
                case 0:
                {
                    mSetI2CDataHigh();
                    iState++;
                    break;
                }
                case 1:
                {
                    mSetI2CClkHigh();
                    iState++;
                    break;
                }
                case 2:
                {
                    mSetI2CDataLow();
                    iState++;
                    break;
                }
                case 3:
                {
                    mSetI2CClkLow();
                    iState = 0;
                    i2cState++;
                    break;
                }
            }
            break;
        }
        //-----------------------------------------------------------
        // Write 8 bits.
        case ADDRESS_W:
        case REGISTER_W:
        case ADDRESS_R:
        {
            if ((iState % 4) == 0)
            {
                // Load the proper value to transmit.
                if (iState == 0)
                {
                    if (i2cState == ADDRESS_W)
                    {
                        value = (ptr->dev << 1);
                    }
                    else if (i2cState == REGISTER_W)
                    {
                        value = ptr->addr;
                    }
                    else if (i2cState == ADDRESS_R)
                    {
                        value = (ptr->dev << 1) | 0x01;
                    }
                }

                // Set data line appropriately.
                if ((value >> (8 - (iState / 4) - 1)) & 0x0001)
                {
                    mSetI2CDataHigh();
                }
                else
                {
                    mSetI2CDataLow();
                }
            }
            else if ((iState % 4) == 1)
            {
                // Clock high.
                mSetI2CClkHigh();
            }
            else if ((iState % 4) == 2)
            {
                // Freeze data.
            }
            else if ((iState % 4) == 3)
            {
                // Clock low.
                mSetI2CClkLow();
                if (iState == 31)
                {
                    // Ready to check ACK.
                    iState = 0;
                    i2cState++;
                    break;
                }
            }
            iState++;
            break;
        }
        //-----------------------------------------------------------
        // Check for ACK.
        case CHECK_ACK1:
        case CHECK_ACK2:
        case CHECK_ACK3:
        case CHECK_ACK4:
        {
            switch (iState)
            {
                case 0:
                {
                    mSetI2CDataHigh();
                    iState++;
                    break;
                }
                case 1:
                {
                    mSetI2CClkHigh();
                    iState++;
                    break;
                }
                case 2:
                {
                    if (mGetI2CData())
                    {
                        ack = 0;
                    }
                    else
                    {
                        ack = 1;
                    }
                    iState++;
                    break;
                }
                case 3:
                {
                    mSetI2CClkLow();
                    iState = 0;
                    if (!ack)
                    {
                        i2cState = STOP;
                        error = 1;
                    }
                    else
                    {
                        if ((ptr->rw == 0) && (i2cState != CHECK_ACK1))
                        {
                            if (i2cBytesWritten < ptr->numBytes)
                            {
                                i2cState = WRITE_BYTE;
                            }
                            else
                            {
                                i2cState = STOP;
                            }
                        }
                        else
                        {
                            i2cState++;
                        }
                    }
                    break;
                }
            }
            break;
        }
        //-----------------------------------------------------------
        case READ_BYTE:
        {
            if ((iState % 4) == 0)
            {
                if (iState == 0)
                {
                    value = 0;
                    mSetI2CDataHigh();
                }
            }
            else if ((iState % 4) == 1)
            {
                // Clock high.
                mSetI2CClkHigh();
            }
            else if ((iState % 4) == 2)
            {
                // Read data.
                value |= (mGetI2CData() << (8 - (iState / 4) - 1));
            }
            else if ((iState % 4) == 3)
            {
                // Clock low.
                mSetI2CClkLow();
                if (iState == 31)
                {
                    // Transfer value.
                    ptr->buf[i2cBytesRead] = value;
                    i2cBytesRead++;
                 
                    // Ready to ACK unless done reading bytes.
                    if (i2cBytesRead >= ptr->numBytes)
                    {
                        i2cState = NACK;
                        i2cBytesRead = 0;
                    }
                    else
                    {
                        i2cState = WRITE_ACK;
                    }

                    iState = 0;
                    break;
                }
            }
            iState++;
            break;
        }
        //-----------------------------------------------------------
        case WRITE_ACK:
        {
            switch (iState)
            {
                case 0:
                {
                    mSetI2CDataLow();
                    iState++;
                    break;
                }
                case 1:
                {
                    mSetI2CClkHigh();
                    iState++;
                    break;
                }
                case 2:
                {
                    // Freeze data.
                    iState++;
                    break;
                }
                case 3:
                {
                    mSetI2CClkLow();
                    iState = 0;
                    i2cState = READ_BYTE;
                    break;
                }
            }
            break;
        }
        //-----------------------------------------------------------
        case NACK:
        {
            switch (iState)
            {
                case 0:
                {
                    mSetI2CDataHigh();
                    iState++;
                    break;
                }
                case 1:
                {
                    mSetI2CClkHigh();
                    iState++;
                    break;
                }
                case 2:
                {
                    if (mGetI2CData())
                    {
                        ack = 0;
                    }
                    else
                    {
                        ack = 1;
                    }
                    iState++;
                    break;
                }
                case 3:
                {
                    mSetI2CClkLow();
                    iState = 0;
                    // Look for NACK.
                    if (!ack)
                    {
                        i2cState++;
                    }
                    else
                    {
                        // Error occurred.
                        i2cState = STOP;
                        error = 1;
                    }
                    break;
                }
            }
            break;
        }
        //-----------------------------------------------------------
        case WRITE_BYTE:
        {
            if ((iState % 4) == 0)
            {
                // Load the proper value to transmit.
                if (iState == 0)
                {
                    value = ptr->buf[i2cBytesWritten];
                }

                // Set data line appropriately.
                if ((value >> (8 - (iState / 4) - 1)) & 0x0001)
                {
                    mSetI2CDataHigh();
                }
                else
                {
                    mSetI2CDataLow();
                }
            }
            else if ((iState % 4) == 1)
            {
                // Clock high.
                mSetI2CClkHigh();
            }
            else if ((iState % 4) == 2)
            {
                // Freeze data.
            }
            else if ((iState % 4) == 3)
            {
                // Clock low.
                mSetI2CClkLow();
                if (iState == 31)
                {
                    i2cBytesWritten++;

                    // Ready to check ACK.
                    iState = 0;
                    i2cState++;
                    break;
                }
            }
            iState++;
            break;
        }
        //-----------------------------------------------------------
        case STOP:
        {
            // Stop condition: Data transitions low-to-high 
            //                 while clock is high.
            switch (iState)
            {
                case 0:
                {
                    mSetI2CDataLow();
                    iState++;
                    break;
                }
                case 1:
                {
                    mSetI2CClkHigh();
                    iState++;
                    break;
                }
                case 2:
                {
                    mSetI2CDataHigh();
                    iState++;
                    break;
                }
                case 3:
                {
                    // Leave the clock high.
                    iState = 0;
                    i2cBytesRead = 0;
                    i2cBytesWritten = 0;
                    i2cState = 0;

                    // Disable interrupts, set completed flag.
                    mT4IntEnable(0);
                    if (error != 0)
                    {
                        *ptr->status = I2C_DRV_STATUS_ERROR;
                    }
                    else
                    {
                        *ptr->status = I2C_DRV_STATUS_DONE;
                    }
                    break;
                }
            }
            break;
        }
    }
    
    mT4ClearIntFlag();
    
ts_end(I2C_T4_ISR);
    return;
}