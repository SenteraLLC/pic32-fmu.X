/*******************************************************************************
/
/   Filename:   spi.c
/
*******************************************************************************/

#include <xc.h>
#include <sys/attribs.h>
#include "spi.h"


static SPI_XFER *spi2Xfer;      // Null if bus is idle, else pointer to data.
static volatile int rxCount;    // Count of SPI bytes received.
static volatile int txCount;    // Count of SPI bytes transmitted.


//==============================================================================

void SPITask()
{
    static enum {
        SM_IDLE,
        SM_START,
        SM_PROCESS,
    } spiTaskState = SM_IDLE;

    switch (spiTaskState)
    {
        case SM_IDLE:
        {
            if ((SPI2STATbits.SPIBUSY == 1) || (spi2Xfer == 0))
            {
                break;
            }
            spiTaskState = SM_START;
            // No break.
        }
        case SM_START:
        {
            // Disable SPI2 Interrupts.
            IEC1CLR = _IEC1_SPI2EIE_MASK | _IEC1_SPI2RXIE_MASK |
                    _IEC1_SPI2TXIE_MASK;

            while (SPI2STATbits.SPIRBE == 0)
            {
                SPI2BUF;                        // Flush RX buffer.
            }

            mSPI2_SS_SET();                     // Assert slave select.
            rxCount = 0;                        // Clear RX byte counter.
            txCount = 0;                        // Clear TX byte counter.

            // Clear SPI2 Interrupt flags.
            IFS1CLR = _IFS1_SPI2EIF_MASK | _IFS1_SPI2RXIF_MASK |
                    _IFS1_SPI2TXIF_MASK;

            // Enable SPI2 Interrupts.
            IEC1SET = _IEC1_SPI2EIE_MASK | _IEC1_SPI2RXIE_MASK |
                    _IEC1_SPI2TXIE_MASK;

            spiTaskState = SM_PROCESS;
            break;
        }
        case SM_PROCESS:
        {
            // Disable SPI2 Interrupts.
            IEC1CLR = _IEC1_SPI2EIE_MASK | _IEC1_SPI2RXIE_MASK |
                    _IEC1_SPI2TXIE_MASK;

            // Unload RX buffer. --------------------------

            while (SPI2STATbits.SPIRBE == 0)
            {
                spi2Xfer->rxBuf[rxCount] = SPI2BUF;
                rxCount++;
            }

            // Load TX buffer. ----------------------------

            // (See SPI2 TX interrupt routine for transmit logic.)

            // Check if transfer is complete. -------------

            if (rxCount == spi2Xfer->length)
            {
                mSPI2_SS_CLR();             // Deassert slave select.
                spi2Xfer->xferDone = 1;     // Set transfer done flag.
                spi2Xfer = 0;               // Clear data pointer.
                spiTaskState = SM_IDLE;
            }
            else
            {
                // Enable SPI2 Interrupts.
                IEC1SET = _IEC1_SPI2EIE_MASK | _IEC1_SPI2RXIE_MASK |
                        _IEC1_SPI2TXIE_MASK;
            }
            break;
        }
    }
}


//==============================================================================

int SPIXfer(SPI_XFER *xfer)
{
    enum {
        SUCCESS = 0,
        FAILURE = 1,
    } spiXferStatus = FAILURE;
    
    if (xfer != 0)
    {
        switch (xfer->port)
        {
            case SPI_PORT_SPI2:
            {
                if (spi2Xfer == 0)
                {
                    spi2Xfer = xfer;            // Copy data pointer.
                    xfer->xferDone = 0;         // Clear transfer done flag.
                    spiXferStatus = SUCCESS;    // Success.
                }
                break;
            }
            default:
            {
                // Unknown SPI port.
                break;
            }
        }
    }
    
    return spiXferStatus;
}


//==============================================================================

// SPI2 ISR

void __ISR (_SPI_2_VECTOR, IPL7SRS) SPI2ISR(void)
{
    // SPI2 Error Interrupt -------------------------------

    if (IFS1bits.SPI2EIF == 1)
    {
        // TODO: Error handling. See FRMERREN, SPIROVEN, and SPITUREN bits
        //       in the SPIxCON2 register.

        IFS1CLR = _IFS1_SPI2EIF_MASK;       // Clear error interrupt flag.
    }

    // SPI2 RX Interrupt ----------------------------------

    if (IFS1bits.SPI2RXIF == 1)
    {
        // RX buffer is one-half or more full.
        while (SPI2STATbits.SPIRBE == 0)
        {
            // Unload RX buffer.
            spi2Xfer->rxBuf[rxCount] = SPI2BUF;
            rxCount++;
        }
        IFS1CLR = _IFS1_SPI2RXIF_MASK;      // Clear RX interrupt flag.
    }

    // SPI2 TX Interrupt ----------------------------------

    if (IFS1bits.SPI2TXIF == 1)
    {
        // TX buffer is one-half or more empty.
        if ((SPI2STATbits.SPITBF == 0) && (txCount < spi2Xfer->length))
        {
            // Load the TX buffer.
            if (spi2Xfer->txBuf == 0)
            {
                SPI2BUF = 0x00;
            }
            else
            {
                SPI2BUF = spi2Xfer->txBuf[txCount];
            }
            txCount++;
        }
        if (txCount == spi2Xfer->length)
        {
            // TX data is fully loaded.  Disable the SPI2 TX interrupt.
            IEC1CLR = _IEC1_SPI2TXIE_MASK;
        }
        IFS1CLR = _IFS1_SPI2TXIF_MASK;      // Clear TX interrupt flag.
    }
}

