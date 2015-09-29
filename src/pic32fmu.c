////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief PIC32 FMU Executive.
////////////////////////////////////////////////////////////////////////////////

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
#include "tcpip/tcpip.h"
#include "ts.h"

// *****************************************************************************
// ************************** Macros *******************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

// Declare AppConfig structure and some other supporting stack variables
APP_CONFIG AppConfig;

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************


// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

////////////////////////////////////////////////////////////////////////////////
/// @brief  C-Environment control flow entry point.
///
/// The function manages processing for the C embedded environment.
////////////////////////////////////////////////////////////////////////////////

int main()
{
    InitBoard();            // Initialize FMU processor and peripherals.
    
    asm volatile ("ei");    // Enable Global interrupts.
    
    UARTStartup();          // Startup UART operation.
    KSZ8895Init();          // Initialize external Ethernet switch.
    
    for (;;)                // Main program loop.
    {
        WDTCONSET = _WDTCON_WDTCLR_MASK;        // Clear watchdog timer.

        ts_start(ADC_TASK);
        ADCTask();
        ts_end(ADC_TASK);
        
        ts_start(SPI_TASK);
        SPITask();
        ts_end(SPI_TASK);
        
        ts_start(UART_TASK);
        UARTTask();
        ts_end(UART_TASK);
        
        ts_start(FMUCOMM_TASK);
        FMUCommTask();
        ts_end(FMUCOMM_TASK);
        
        ts_start(VN100_TASK);
        VN100Task();
        ts_end(VN100_TASK);
        
        ts_start(OEMSTAR_TASK);
        OEMStarTask();
        ts_end(OEMSTAR_TASK);
        
        ts_start(EMC1412_TASK);
        EMC1412Task();
        ts_end(EMC1412_TASK);
        
        CoreTimeDelayUs(500);
        
        ts_start(SBUS_TASK);
        SBusTask();
        ts_end(SBUS_TASK);
        
        ts_start(RC_TASK);
        RCTask();
        ts_end(RC_TASK);
        
        ts_start(STATUS_TASK);
        StatusTask();
        ts_end(STATUS_TASK);
        
        ts_start(SNODE_TASK);
        SNodeTask();
        ts_end(SNODE_TASK);
        
        ts_start(STACK_TASK);
        StackTask();
        ts_end(STACK_TASK);
        
        ts_start(STACK_APPLICATION);
        StackApplications();
        ts_end(STACK_APPLICATION);
        
        exe_cycle_inc();
    }
    
    return 0;
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************
