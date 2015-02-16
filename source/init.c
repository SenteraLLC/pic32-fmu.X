/*******************************************************************************
/
/   Filename:   init.c
/
*******************************************************************************/

#include <xc.h>
#include <sys/attribs.h>
#include "init.h"


//==============================================================================

static void InitCPU()
{
    switch (DEVIDbits.VER)
    {
        case 0x0:
        case 0x1:
        case 0x2:
        case 0x3:
        {
            // NOTE: See errata #43: CPU: Constant Data Access from Flash
            break;
        }
        case 0x4:
        case 0x5:
        default:
        {
            break;
        }
    }
}

//==============================================================================

static void InitGPIO()
{
    // Disable JTAG port.
    DDPCONbits.JTAGEN = 0;

    // Red Status LED (RF3)
    TRISFbits.TRISF3 = 0;       // Output
    ODCFbits.ODCF3 = 1;         // Open Drain
    LATFbits.LATF3 = 1;         // Off

    // SPI2 SS (RG9)
    TRISGbits.TRISG9 = 0;       // Output
    ODCGbits.ODCG9 = 0;         // Normal
    LATGbits.LATG9 = 1;         // High
}

//==============================================================================

static void InitTMR()
{
    // Core Timer

    IPC0bits.CTIP = 7;              // Set core timer interrupt priority.
    IPC0bits.CTIS = 0;              // Set core timer interrupt subpriority.
    IFS0CLR = _IFS0_CTIF_MASK;      // Clear core timer interrupt flag.
    IEC0SET = _IEC0_CTIE_MASK;      // Enable CT interrupts.
}

//==============================================================================

static void InitSPI()
{
    // SPI2

    SPI2CONCLR = 0xFFFFFFFF;

    SPI2CONSET = _SPI2CON_CKP_MASK;     // Clock idles high.
    SPI2CONSET = _SPI2CON_MSTEN_MASK;   // Master mode.
    SPI2CONSET = _SPI2CON_ENHBUF_MASK;  // Enhanced buffer mode.

    SPI2BRG = 15;                       // 2.5 MHz SPI clock.

    SPI2CONbits.STXISEL = 0b10;         // TX interrupt when buffer is empty
                                        //   by one-half or more.

    SPI2CONbits.SRXISEL = 0b10;         // RX interrupt when buffer is full
                                        //   by one-half or more.

    IPC7bits.SPI2IP = 7;                // Set SPI2 interrupt priority.
    IPC7bits.SPI2IS = 0;                // Set SPI2 interrupt subpriority.

    IFS1CLR = _IFS1_SPI2EIF_MASK;       // Clear SPI2 error interrupt flag.
    IFS1CLR = _IFS1_SPI2RXIF_MASK;      // Clear SPI2 RX interrupt flag.
    IFS1CLR = _IFS1_SPI2TXIF_MASK;      // Clear SPI2 TX interrupt flag.

    IEC1CLR = _IEC1_SPI2EIE_MASK;       // Disable SPI2 error interrupt.
    IEC1CLR = _IEC1_SPI2RXIE_MASK;      // Disable SPI2 RX interrupt.
    IEC1CLR = _IEC1_SPI2TXIE_MASK;      // Disable SPI2 TX interrupt.

    SPI2CONSET = _SPI2CON_ON_MASK;      // Enable SPI peripheral.
}

//==============================================================================

static void InitINT()
{
    INTCONSET = _INTCON_MVEC_MASK;      // Enable multi-vectored mode.
    asm("ei");                          // Enable interrupts.
}

//==============================================================================

static void InitWDT()
{
    if ((RCONbits.POR == 1) && (RCONbits.BOR == 1))
    {
        // Initial power on detected.  Clear status bits.
        RCONCLR = _RCON_CMR_MASK;
        RCONCLR = _RCON_EXTR_MASK;
        RCONCLR = _RCON_WDTO_MASK;
        RCONCLR = _RCON_BOR_MASK;
        RCONCLR = _RCON_POR_MASK;
    }
}

//==============================================================================

void InitBoard()
{
    // Initialize microcontroller peripherals.
    InitCPU();
    InitGPIO();
    InitTMR();
    InitSPI();
    InitINT();
    
    // Initialize watchdog timer.
    InitWDT();
}

