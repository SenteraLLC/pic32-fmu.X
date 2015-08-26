/*******************************************************************************
/
/   Filename:   softspi.h
/
*******************************************************************************/

#ifndef SOFTSPI_H_
#define SOFTSPI_H_


#include <inttypes.h>


// Data Types -----------------------------------------------------------------

typedef enum {
    SOFTSPI_KSZ8895 = 0,
} SOFTSPI_DEVICE;

typedef volatile struct {
    SOFTSPI_DEVICE dev; // SPI bus target device, slave select dependent.
    uint32_t txDat;     // TX data.
    uint32_t *rxBuf;    // RX buffer.
    int numOfBits;      // Number of bits to transfer.
    int freq;           // SPI clock frequency.
    int xferDone;       // 0 when transfer is initiated, 1 when complete.
} SOFTSPI_TRANSFER;


// Software SPI Macros --------------------------------------------------------

#define mGetSpiMiso()               (PORTBbits.RB10)                // Read MISO.

#define mSetSpiMosiHigh()           (LATBSET = _LATB_LATB9_MASK)    // Drive MOSI high.
#define mSetSpiMosiLow()            (LATBCLR = _LATB_LATB9_MASK)    // Drive MOSI low.

#define mSetSpiClkHigh()            (LATBSET = _LATB_LATB6_MASK)    // Drive CLK high.
#define mSetSpiClkLow()             (LATBCLR = _LATB_LATB6_MASK)    // Drive CLK low.

#define mSetSpiSsHigh_KSZ8895()     (LATBSET = _LATB_LATB7_MASK)    // Drive SS high.
#define mSetSpiSsLow_KSZ8895()      (LATBCLR = _LATB_LATB7_MASK)    // Drive SS low.


// Function Prototypes --------------------------------------------------------

int SoftSPIXfer(SOFTSPI_TRANSFER *xfer);


#endif  // SOFTSPI_H_
