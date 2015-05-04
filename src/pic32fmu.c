/*******************************************************************************
/
/   Filename:   pic32fmu.c
/
*******************************************************************************/

#include <xc.h>
#include "pic32fmu.h"


//==============================================================================

int main()
{
    InitBoard();    // Initialize FMU processor and peripherals.
    
    for (;;)        // Main program loop.
    {
        WDTCONSET = _WDTCON_WDTCLR_MASK;        // Clear watchdog timer.
        LATFbits.LATF3 = CoreTime64sGet() % 2;  // Blink LED at 0.5Hz

        // Low-level communication tasks. -----------------
        SPITask();

        // Acquire sensor data. ---------------------------
        VN100Task();
        OEMStarTask();
    }

    return 0;
}

