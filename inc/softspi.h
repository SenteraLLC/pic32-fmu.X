////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Soft Serial Peripheral Interface (SPI) driver.
////////////////////////////////////////////////////////////////////////////////

#ifndef SOFTSPI_H_
#define SOFTSPI_H_

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

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

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

#define mGetSpiMiso()               (PORTBbits.RB10)                // Read MISO.

#define mSetSpiMosiHigh()           (LATBSET = _LATB_LATB9_MASK)    // Drive MOSI high.
#define mSetSpiMosiLow()            (LATBCLR = _LATB_LATB9_MASK)    // Drive MOSI low.

#define mSetSpiClkHigh()            (LATBSET = _LATB_LATB6_MASK)    // Drive CLK high.
#define mSetSpiClkLow()             (LATBCLR = _LATB_LATB6_MASK)    // Drive CLK low.

#define mSetSpiSsHigh_KSZ8895()     (LATBSET = _LATB_LATB7_MASK)    // Drive SS high.
#define mSetSpiSsLow_KSZ8895()      (LATBCLR = _LATB_LATB7_MASK)    // Drive SS low.

// *****************************************************************************
// ************************** Declarations *************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

////////////////////////////////////////////////////////////////////////////////
/// @brief  Queue SPI data for transfer.
///
/// @param  xfer
///             Buffer of control and communication data for transfer.
///
/// @return Identification of queuing of SPI data as successful.
///             -1 - Unsuccessful.
///             0  - Success.
///
/// This function queues SPI data for transfer.  Transfer is identifies as
/// \e soft as SPI communication is implemented as bit-banged design.
////////////////////////////////////////////////////////////////////////////////
int SoftSPIXfer(SOFTSPI_TRANSFER *xfer);

#endif  // SOFTSPI_H_
