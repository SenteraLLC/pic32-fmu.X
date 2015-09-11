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

        // Low-level communication tasks. --------------------------------------
        ADCTask();      // Read ATD input(s).
        SPITask();      // Manage completion of SPI transfer.
        UARTTask();     // Manage UART Rx buffers.

        FMUCommTask();  // Read Ethernet packets.
        
        // High-Level communication tasks. -------------------------------------
        VN100Task();    // Read VN100 data and queue Ethernet packet for transfer.
        OEMStarTask();  // Forward data between OEMStar<->Host.
        EMC1412Task();  // Read temperature sensor data.
        SBusTask();     // Read SBUS data.
        RCTask();       // Packetize SBUS data for Ethernet transfer and decode SBUS data.
        
        StatusTask();   // Annunciate FMU status.
        SNodeTask();    // Manage communication with the Servo-Node network.
        
        // This task performs normal stack task including checking
        // for incoming packet, type of packet and calling
        // appropriate stack entity to process it.
        StackTask();

        // This tasks invokes each of the core stack application tasks.
        StackApplications();
    }

    return 0;
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************
