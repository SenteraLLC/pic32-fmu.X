/*******************************************************************************
/
/   Filename:   init.c
/
*******************************************************************************/

#include <xc.h>
#include <sys/attribs.h>
#include "tcpip/tcpip.h"
#include "ksz8895.h"
#include "init.h"
#include "can.h"
#include "uart.h"


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
    
    //-----------------------------------------------------

    // Software SPI Bit-bang CLK (RB6)

    AD1PCFGbits.PCFG6 = 1;      // Analog input pin in digital mode.
    ODCBbits.ODCB6 = 0;         // Normal
    TRISBbits.TRISB6 = 0;       // Output
    LATBbits.LATB6 = 0;         // Logic Low
    
    // Software SPI Bit-bang MOSI (RB9)

    AD1PCFGbits.PCFG9 = 1;      // Analog input pin in digital mode.
    ODCBbits.ODCB9 = 0;         // Normal
    TRISBbits.TRISB9 = 0;       // Output
    LATBbits.LATB9 = 0;         // Logic Low
    
    // Software SPI Bit-bang MISO (RB10)

    AD1PCFGbits.PCFG10 = 1;     // Analog input pin in digital mode.
    ODCBbits.ODCB10 = 0;        // Normal
    TRISBbits.TRISB10 = 1;      // Input
    
    // Software SPI Bit-bang !SS for Ethernet Switch (RB7)

    AD1PCFGbits.PCFG7 = 1;      // Analog input pin in digital mode.
    ODCBbits.ODCB7 = 0;         // Normal
    TRISBbits.TRISB7 = 0;       // Output
    LATBbits.LATB7 = 1;         // Logic High
}

//==============================================================================

static void InitTMR()
{
    // Core Timer
    IPC0bits.CTIP = 7;              // Set core timer interrupt priority.
    IPC0bits.CTIS = 0;              // Set core timer interrupt subpriority.
    IFS0CLR = _IFS0_CTIF_MASK;      // Clear core timer interrupt flag.
    IEC0SET = _IEC0_CTIE_MASK;      // Enable CT interrupts.
    
    // Initialize Timer 4 (software I2C clock)
    ConfigIntTimer4(T4_INT_OFF | T4_INT_PRIOR_6 | T4_INT_SUB_PRIOR_0);
    OpenTimer4(T4_ON | T4_IDLE_CON | T4_PS_1_1, 800);       // 100 kHz
    
    //  Timer 5 (software SPI clock)
    ConfigIntTimer5(T5_INT_OFF | T5_INT_PRIOR_7 | T5_INT_SUB_PRIOR_0);
    OpenTimer5(T5_ON | T5_IDLE_CON | T5_PS_1_1, 800);       // 100 kHz
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

static void InitI2C(void)
{
    // RB3 = SMBCLK
    AD1PCFGbits.PCFG3 = 1;  // Analog input pin in digital mode.
    ODCBbits.ODCB3 = 1;     // Open Drain
    LATBbits.LATB3 = 1;     // Off
    TRISBbits.TRISB3 = 0;   // Output

    // RB4 = SMBDAT
    AD1PCFGbits.PCFG4 = 1;  // Analog input pin in digital mode.
    ODCBbits.ODCB4 = 1;     // Open Drain
    LATBbits.LATB4 = 1;     // Off
    TRISBbits.TRISB4 = 0;   // Output

    return;
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

static void InitTCPIPStack()
{
    static ROM uint8_t SerializedMACAddress[6] = 
            {MY_DEFAULT_MAC_BYTE1, MY_DEFAULT_MAC_BYTE2, 
             MY_DEFAULT_MAC_BYTE3, MY_DEFAULT_MAC_BYTE4, 
             MY_DEFAULT_MAC_BYTE5, MY_DEFAULT_MAC_BYTE6};

    TickInit();

    #if defined(STACK_USE_MPFS2)
    MPFSInit();
    #endif

    // Initialize Stack and application related NV variables into AppConfig.

    // Start out zeroing all AppConfig bytes to ensure all fields are
    // deterministic for checksum generation
    memset((void*)&AppConfig, 0x00, sizeof(AppConfig));    
    
    AppConfig.Flags.bIsDHCPEnabled = false;
    AppConfig.Flags.bInConfigMode = true;
    
    // Use default MAC address of the module
    memcpypgm2ram((void*)&AppConfig.MyMACAddr, 
            (ROM void*)SerializedMACAddress, sizeof(AppConfig.MyMACAddr));    

    AppConfig.MyIPAddr.Val = 
            MY_DEFAULT_IP_ADDR_BYTE1 | 
            MY_DEFAULT_IP_ADDR_BYTE2 << 8ul | 
            MY_DEFAULT_IP_ADDR_BYTE3 << 16ul | 
            MY_DEFAULT_IP_ADDR_BYTE4 << 24ul;
    AppConfig.DefaultIPAddr.Val = AppConfig.MyIPAddr.Val;
    AppConfig.MyMask.Val = 
            MY_DEFAULT_MASK_BYTE1 | 
            MY_DEFAULT_MASK_BYTE2 << 8ul | 
            MY_DEFAULT_MASK_BYTE3 << 16ul | 
            MY_DEFAULT_MASK_BYTE4 << 24ul;
    AppConfig.DefaultMask.Val = AppConfig.MyMask.Val;
    AppConfig.MyGateway.Val = 
            MY_DEFAULT_GATE_BYTE1 | 
            MY_DEFAULT_GATE_BYTE2 << 8ul | 
            MY_DEFAULT_GATE_BYTE3 << 16ul | 
            MY_DEFAULT_GATE_BYTE4 << 24ul;
    AppConfig.PrimaryDNSServer.Val = 
            MY_DEFAULT_PRIMARY_DNS_BYTE1 | 
            MY_DEFAULT_PRIMARY_DNS_BYTE2 << 8ul | 
            MY_DEFAULT_PRIMARY_DNS_BYTE3 << 16ul |
            MY_DEFAULT_PRIMARY_DNS_BYTE4 << 24ul;
    AppConfig.SecondaryDNSServer.Val = 
            MY_DEFAULT_SECONDARY_DNS_BYTE1 | 
            MY_DEFAULT_SECONDARY_DNS_BYTE2 << 8ul |
            MY_DEFAULT_SECONDARY_DNS_BYTE3 << 16ul |
            MY_DEFAULT_SECONDARY_DNS_BYTE4 << 24ul;

    // Load the default NetBIOS Host Name
    memcpypgm2ram(AppConfig.NetBIOSName, (ROM void*) MY_DEFAULT_HOST_NAME, 16);
    FormatNetBIOSName(AppConfig.NetBIOSName);

    // Initialize core stack layers (MAC, ARP, TCP, UDP) and
    // application modules (HTTP, SNMP, etc.)
    StackInit();

    return;
}

//==============================================================================

static void InitADC(void)
{
    // Configure the 10-bit Analog to Digital converter.
    //      AN5 = Battery Voltage

    CloseADC10();
    asm("nop");
    asm("nop");
    asm("nop");

    // AD1CON1: ADC Control Register 1
    AD1CON1bits.SIDL = 1;       // Discontinue operation in Idle mode.
    AD1CON1bits.FORM = 0b000;   // 16-bit integer data output format.
    AD1CON1bits.SSRC = 0b111;   // Auto convert.
    AD1CON1bits.CLRASAM = 0;    // Normal operation
    AD1CON1bits.ASAM = 0;       // Sampling begins when SAMP bit is set.
    AD1CON1bits.DONE = 0;

    // AD1CON2: ADC Control Register 2
    AD1CON2bits.VCFG = 0;       // VR+ = AVdd, VR- = AVss
    AD1CON2bits.OFFCAL = 0;     // Disable Offset Calibration mode.
    AD1CON2bits.CSCNA = 0;      // Scan inputs.
    AD1CON2bits.SMPI = 0b0000;  // Interrupts for each sample/convert.
    AD1CON2bits.BUFM = 0;       // Buffer configured as one 16-word buffer.
    AD1CON2bits.ALTS = 0;       // Always use MUX A input mux settings.

    // AD1CON3: ADC Control Register 3
    AD1CON3bits.ADRC = 0;       // Clock derived from Peripheral Bus Clock.
    AD1CON3bits.SAMC = 0b11111; // Auto-sample Time = 31 TAD.
    AD1CON3bits.ADCS = 0x0F;    // ADC Conversion Clock = 32 * TPB = TAD.

    // AD1CHS: ADC Input Select Register
    AD1CHSbits.CH0NA = 0;       // Channel 0 negative input is VR-.
    AD1CHSbits.CH0SA = 5;       // Channel 0 positive input is AN5.

    // AD1PCFG: ADC Port Configuration Register
    AD1PCFG = 0xFFDF;           // All Analog input pins in Digital mode,
                                // except AN5.

    // AD1CSSL: ADC Input Scan Select Register
    AD1CSSL = 0;

    EnableADC10();
    DelayMs(1);

    return;
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
    CANInit();
    UARTInit();
    InitADC();
    InitI2C();
    
    // Initialize external hardware peripherals.
    KSZ8895Init();

    // Initialize software libraries.
    InitTCPIPStack();
    
    // Initialize watchdog timer.
    InitWDT();
}

