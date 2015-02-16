/*******************************************************************************
/
/   Filename:   spi.h
/
*******************************************************************************/

#ifndef SPI_H
#define	SPI_H

#include <stdint.h>


// Macros and Defines ==========================================================

#define mSPI2_SS_SET()  (LATGCLR = _LATG_LATG9_MASK)    // Assert select.
#define mSPI2_SS_CLR()  (LATGSET = _LATG_LATG9_MASK)    // Deassert select.


// Typedefs ====================================================================

typedef enum {
    SPI_PORT_SPI2 = 2,
} SPI_PORT;

typedef struct {
    SPI_PORT        port;
    uint8_t         *rxBuf;
    uint8_t         *txBuf;
    unsigned int    length;
    unsigned int    xferDone;
} SPI_XFER;


// Function Prototypes =========================================================

void SPITask();
int SPIXfer(SPI_XFER *xfer);


#endif	/* SPI_H */

