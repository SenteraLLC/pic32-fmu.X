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
#include "uart.h"
#include "status.h"
#include "adc.h"
#include "emc1412.h"
#include "fmucomm.h"
#include "snode.h"
#include "ksz8895.h"
#include "sbus.h"
#include "rc.h"

// Include all headers for any enabled TCPIP Stack functions
#include "tcpip/tcpip.h"

// Declare AppConfig structure and some other supporting stack variables
APP_CONFIG AppConfig;


//==============================================================================

int main()
{
    InitBoard();            // Initialize FMU processor and peripherals.
    
    asm volatile ("ei");    // Enable Global interrupts.
    
    UARTStartup();          // Startup UART operation.
    KSZ8895Init();          // Initialize external Ethernet switch.
    
    for (;;)                // Main program loop.
    {
        WDTCONSET = _WDTCON_WDTCLR_MASK;        // Clear watchdog timer.

        // Low-level communication tasks. ---------------------------
        ADCTask();
        SPITask();
        UARTTask();  // NOTE: Task must occur before any function in software cycle which used received UART data.
        
        // This task reads UDP data for processing; therefore this task
        // must be executed in the software cycle before any function
        // which gets UDP data.
        FMUCommTask();
        
        // Acquire sensor data. -------------------------------------
        VN100Task();
        OEMStarTask();
        EMC1412Task();
        SBusTask();
        RCTask();
        
        StatusTask();
        SNodeTask();
        
        // This task performs normal stack task including checking
        // for incoming packet, type of packet and calling
        // appropriate stack entity to process it.
        StackTask();

        // This tasks invokes each of the core stack application tasks.
        StackApplications();
    }

    return 0;
}

