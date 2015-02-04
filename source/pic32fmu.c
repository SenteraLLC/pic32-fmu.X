/*******************************************************************************
/
/   Filename:   pic32fmu.c
/
*******************************************************************************/

#include "cfgbits.h"        // Device configuration bits.

#include <p32xxxx.h>


//==============================================================================

int main()
{
    for (;;)
    {
        WDTCONSET = _WDTCON_WDTCLR_MASK;    // Clear watchdog timer.
    }
    
    return 0;
}

