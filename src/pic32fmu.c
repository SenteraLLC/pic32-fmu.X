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


typedef enum
{
    ADC_TASK              ,
    SPI_TASK    ,
    UART_TASK    ,
    FMUCOMM_TASK    ,
    VN100_TASK,
    OEMSTAR_TASK    ,
    EMC1412_TASK    ,
    STATUS_TASK    ,
    SBUS_TASK,
    RC_TASK,
    SNODE_TASK    ,
    STACK_TASK    ,
    STACK_APPLICATION    ,
            
            TASK_MAX,
            
} TASK_E;

static uint32_t us_max[ TASK_MAX ] = { 0 };
static uint64_t us_sum[ TASK_MAX ] = { 0 };
static uint32_t exe_cnt = 0;

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************


// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************


static uint32_t ts_start_value;

static void ts_start( void )
{
    ts_start_value = CoreTime32usGet();
}

static void ts_end( TASK_E task_item )
{
    uint32_t ts_diff;
    
    ts_diff = CoreTime32usGet() - ts_start_value;
    
    us_sum[ task_item ] += ts_diff;
            
    if( ts_diff > us_max[ task_item ] )
    {
        us_max[ task_item ] = ts_diff;
    }
}



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

        ts_start();
        ADCTask();
        ts_end(ADC_TASK);
        
        ts_start();
        SPITask();
        ts_end(SPI_TASK);
        
        ts_start();
        UARTTask();
        ts_end(UART_TASK);
        
        ts_start();
        FMUCommTask();
        ts_end(FMUCOMM_TASK);
        
        ts_start();
        VN100Task();
        ts_end(OEMSTAR_TASK);
        
        ts_start();
        OEMStarTask();
        ts_end(OEMSTAR_TASK);
        
        ts_start();
        EMC1412Task();
        ts_end(EMC1412_TASK);
        
        ts_start();
        SBusTask();
        ts_end(STATUS_TASK);
        
        ts_start();
        RCTask();
        ts_end(STATUS_TASK);
        
        ts_start();
        SNodeTask();
        ts_end(SNODE_TASK);
        
        ts_start();
        StackTask();
        ts_end(STACK_TASK);
        
        ts_start();
        StackApplications();
        ts_end(STACK_APPLICATION);
        
        exe_cnt++;
    }
    
    return 0;
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************
