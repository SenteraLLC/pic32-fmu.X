/*******************************************************************************
/
/   Filename:   pic32fmu.c
/
*******************************************************************************/

#include <xc.h>
#include "spi.h"
#include "stdtypes.h"
#include "coretime.h"
#include "init.h"
#include "vn100.h"
#include "oemstar.h"
#include "fmucomm.h"

// Include all headers for any enabled TCPIP Stack functions
#include "tcpip/tcpip.h"

// Declare AppConfig structure and some other supporting stack variables
APP_CONFIG AppConfig;


//==============================================================================

int main()
{
    InitBoard();    // Initialize FMU processor and peripherals.
    
    for (;;)        // Main program loop.
    {
        WDTCONSET = _WDTCON_WDTCLR_MASK;        // Clear watchdog timer.
        LATFbits.LATF3 = CoreTime64sGet() % 2;  // Blink LED at 0.5Hz

        // Low-level communication tasks. ---------------------------
        SPITask();

        // Acquire sensor data. -------------------------------------
        VN100Task();
        OEMStarTask();
        
        FMUCommTask();
        
        // This task performs normal stack task including checking
        // for incoming packet, type of packet and calling
        // appropriate stack entity to process it.
        StackTask();

        // This tasks invokes each of the core stack application tasks.
        StackApplications();
    }

    return 0;
}

