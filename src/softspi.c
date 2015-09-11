////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Soft Serial Peripheral Interface (SPI) driver.
////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

#include "tcpip/tcpip.h"
#include "system.h"
#include "softspi.h"

// *****************************************************************************
// ************************** Macros *******************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

static volatile uint32_t spiTxBuf;
static volatile uint32_t *spiRxBuf;
static volatile int spiXferBits;
static volatile int *completeFlag;

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************


// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

int SoftSPIXfer(SOFTSPI_TRANSFER *xfer)
{
    // Sanitize the inputs. -----------------------------------------

    if ((xfer->numOfBits < 1) || (xfer->numOfBits > 32))
    {
        // Invalid number of bits to transfer.
        return -1;
    }
    if ((xfer->freq < 0) || (xfer->freq > (GetPeripheralClock() / 2)))
    {
        // Invalid SPI clock.
        return -1;
    }

    // Take control of software SPI bus if it's available. ----------

    if (mT5GetIntEnable())
    {
        return -1;              // Software SPI bus is busy.
    }

    // Assert slave select line for desired bus device.
    switch (xfer->dev)
    {
        case SOFTSPI_KSZ8895:   // Micrel KSZ8895
        {
            mSetSpiSsLow_KSZ8895();
            break;
        }
    }

    // Configure the software SPI timer for desired clock frequency.
    if (GetPeripheralClock() % (xfer->freq * 2))
    {
        // No perfect frequency divisor possible.
        // Round up to next period number to be conservative.
        WritePeriod5((GetPeripheralClock() / (xfer->freq * 2)) + 1);
    }
    else
    {
        // Perfect frequency match.
        WritePeriod5(GetPeripheralClock() / (xfer->freq * 2));
    }

    // Clear the receive buffer.
    if (xfer->rxBuf != 0)
    {
        *xfer->rxBuf = 0;
    }

    // Set global buffer pointers and bit counter.
    spiTxBuf = xfer->txDat;
    spiRxBuf = xfer->rxBuf;
    spiXferBits = xfer->numOfBits;
    completeFlag = &xfer->xferDone;

    // Clear the xferDone flag before initiating transfer.
    xfer->xferDone = 0;

    TMR5 = 0;
    mT5ClearIntFlag();
    mT5IntEnable(1);        // Initiate software SPI transfer.

    return 0;
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************

////////////////////////////////////////////////////////////////////////////////
/// @brief  Period soft SPI interrupt.
///
/// This function implements the big-bang operation to perform the SPI
/// protocol.
////////////////////////////////////////////////////////////////////////////////
void __ISR (_TIMER_5_VECTOR, IPL7SRS) Timer5ISR(void)
{
    static int spiState = 0;

    if (spiState < (2 * spiXferBits))
    {
        if ((spiState % 2) == 0)
        {
            // Even state.
            mSetSpiClkLow();
            if ((spiTxBuf >> (spiXferBits - (spiState / 2) - 1)) & 0x0001)
            {
                mSetSpiMosiHigh();
            }
            else
            {
                mSetSpiMosiLow();
            }
        }
        else
        {
            // Odd state.
            mSetSpiClkHigh();
            if (spiRxBuf != 0)
            {
                *spiRxBuf |=
                        (mGetSpiMiso() << (spiXferBits - (spiState / 2) - 1));
            }
        }
        spiState++;
    }
    else
    {
        mSetSpiClkHigh();
        spiState = 0;
        mT5IntEnable(0);            // Disable Timer 5 interrupts.

        // De-assert slave select lines.
        mSetSpiSsHigh_KSZ8895();

        // Set transfer complete flag.
        *completeFlag = 1;
    }

    mT5ClearIntFlag();
    return;
}