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
#include "uart.h"

// Include all headers for any enabled TCPIP Stack functions
#include "tcpip/tcpip.h"

// Declare AppConfig structure and some other supporting stack variables
APP_CONFIG AppConfig;


//==============================================================================

int main()
{
    InitBoard();    // Initialize FMU processor and peripherals.
    
    // INTEnableSystemMultiVectoredInt();
    asm volatile ("ei");
    
    UARTStartup();
    
    for (;;)        // Main program loop.
    {
        WDTCONSET = _WDTCON_WDTCLR_MASK;        // Clear watchdog timer.
        LATFbits.LATF3 = CoreTime64sGet() % 2;  // Blink LED at 0.5Hz

        // Low-level communication tasks. ---------------------------
        SPITask();

        // This task reads UDP data for processing; therefore this task
        // must be executed in the software cycle before any function
        // which gets UDP data.
        FMUCommTask();
        
        // Acquire sensor data. -------------------------------------
        OEMStarTask();
        
        // This task performs normal stack task including checking
        // for incoming packet, type of packet and calling
        // appropriate stack entity to process it.
        StackTask();

        // This tasks invokes each of the core stack application tasks.
        StackApplications();
    }

    return 0;
}

