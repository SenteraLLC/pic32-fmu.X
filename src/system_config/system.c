/*******************************************************************************
  System Specific Initializations

  Company:
    Microchip Technology Inc.

  File Name:
    system.c

  Summary:
    System level initializations for the specific 
    Microchip Development Board used.

*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
// DOM-IGNORE-END

// Set configuration fuses
// Previously it was applied only in main.c where 
// THIS_IS_STACK_APPLICATION is defined
// Now it is moved to system.c without having #if defined around

// Configuration Bits ---------------------------------------------------------

// NOTE:
//  SYSCLK := Clock for CPU and select peripherals.
//  PBCLK  := Clock for peripherals.
//
//  SYSCLK  = Fosc  / FPLLIDIV * PLLMULT / PLLODIV
//          = 10MHz / 2        * 16      / 1
//          = 80MHz
//
//  PBCLK   = SYSCLK / PBDIV
//          = 80MHz  / 1
//          = 80MHz
//

    // USB VBUS_ON Selection bit: ---------------------------------------------
    //
    //   OFF = VBUS_ON pin is controlled by the Port Function
    //   ON  = VBUS_ON pin is controlled by the USB Module
    //
    #pragma config FVBUSONIO = OFF

    // USB USBID Selection bit: -----------------------------------------------
    //
    //   OFF = USBID pin is controlled by the Port Function
    //   ON  = USBID pin is controlled by the USB Module
    //
    #pragma config FUSBIDIO = OFF

    // CAN IO Pin Selection bit: ----------------------------------------------
    //
    //   OFF = Alternate CAN IO Pins
    //   ON  = Default CAN IO Pins
    //
    #pragma config FCANIO = ON

    // Ethernet IO Pin Selection bit: -----------------------------------------
    //
    //   OFF = Alternate Ethernet IO Pins
    //   ON  = Default Ethernet IO Pins
    //
    #pragma config FETHIO = ON

    // Ethernet MII Enable bit: -----------------------------------------------
    //
    //   OFF = RMII enabled
    //   ON  = MII enabled
    //
    #pragma config FMIIEN = ON

    // SRS (Shadow Register Set) Select: --------------------------------------
    //
    //   PRIORITY_0 = SRS Interrupt Priority Level 0
    //   PRIORITY_1 = SRS Interrupt Priority Level 1
    //   PRIORITY_2 = SRS Interrupt Priority Level 2
    //   PRIORITY_3 = SRS Interrupt Priority Level 3
    //   PRIORITY_4 = SRS Interrupt Priority Level 4
    //   PRIORITY_5 = SRS Interrupt Priority Level 5
    //   PRIORITY_6 = SRS Interrupt Priority Level 6
    //   PRIORITY_7 = SRS Interrupt Priority Level 7
    //
    #pragma config FSRSSEL = PRIORITY_7

    // PLL Output Divider Value: ----------------------------------------------
    //
    //   DIV_1   = Divide by 1
    //   DIV_2   = Divide by 2
    //   DIV_4   = Divide by 4
    //   DIV_8   = Divide by 8
    //   DIV_16  = Divide by 16
    //   DIV_32  = Divide by 32
    //   DIV_64  = Divide by 64
    //   DIV_256 = Divide by 256
    //
    #pragma config FPLLODIV = DIV_1

    // USB PLL Enable bit: ----------------------------------------------------
    //
    //   OFF = Disabled
    //   ON  = Enabled
    //
    #pragma config UPLLEN = OFF

    // USB PLL Input Divider bits: --------------------------------------------
    //
    //   DIV_1  = Divide by 1
    //   DIV_2  = Divide by 2
    //   DIV_3  = Divide by 3
    //   DIV_4  = Divide by 4
    //   DIV_5  = Divide by 5
    //   DIV_6  = Divide by 6
    //   DIV_10 = Divide by 10
    //   DIV_12 = Divide by 12
    //
    #pragma config UPLLIDIV = DIV_1

    // PLL Multiplier bits: ---------------------------------------------------
    //
    //   MUL_15 = Multiply by 15
    //   MUL_16 = Multiply by 16
    //   MUL_17 = Multiply by 17
    //   MUL_18 = Multiply by 18
    //   MUL_19 = Multiply by 19
    //   MUL_20 = Multiply by 20
    //   MUL_21 = Multiply by 21
    //   MUL_24 = Multiply by 24
    //
    #pragma config FPLLMUL = MUL_16

    // PLL Input Divider bits: ------------------------------------------------
    //
    //   DIV_1  = Divide by 1
    //   DIV_2  = Divide by 2
    //   DIV_3  = Divide by 3
    //   DIV_4  = Divide by 4
    //   DIV_5  = Divide by 5
    //   DIV_6  = Divide by 6
    //   DIV_10 = Divide by 10
    //   DIV_12 = Divide by 12
    //
    #pragma config FPLLIDIV = DIV_2

    // Watchdog Timer Enable bit: ---------------------------------------------
    //
    //   OFF = Disabled (it can be enabled in software)
    //   ON  = Enabled (it cannnot be disabled by software)
    //
    #pragma config FWDTEN = ON

    // Watchdog Timer Postscale Select bits: ----------------------------------
    //
    //   PS1       = 1:1
    //   PS2       = 1:2
    //   PS4       = 1:4
    //   PS8       = 1:8
    //   PS16      = 1:16
    //   PS32      = 1:32
    //   PS64      = 1:64
    //   PS128     = 1:128
    //   PS256     = 1:256
    //   PS512     = 1:512
    //   PS1024    = 1:1024
    //   PS2048    = 1:2048
    //   PS4096    = 1:4096
    //   PS8192    = 1:8,192
    //   PS16384   = 1:16,384
    //   PS32768   = 1:32,768
    //   PS65536   = 1:65,536
    //   PS131072  = 1:131,072
    //   PS262144  = 1:262,144
    //   PS524288  = 1:524,288
    //   PS1048576 = 1:1,048,576
    //
    #pragma config WDTPS = PS4096

    // Clock Switching and Monitor Selection bits: ----------------------------
    //
    //   CSECME = Clock Switching Enabled, Clock Monitoring Enabled
    //   CSECMD = Clock Switching Enabled, Clock Monitoring Disabled
    //   CSDCMD = Clock Switching Disabled, Clock Monitoring Disabled
    //
    #pragma config FCKSM = CSDCMD

    // Bootup PBCLK divider: --------------------------------------------------
    //
    //   DIV_1 = Divide by 1
    //   DIV_2 = Divide by 2
    //   DIV_4 = Divide by 4
    //   DIV_8 = Divide by 8
    //
    #pragma config FPBDIV = DIV_1

    // CLKO Enable bit: -------------------------------------------------------
    //
    //   ON  = Enabled
    //   OFF = Disabled
    //
    #pragma config OSCIOFNC = OFF

    // Primary Oscillator bits: -----------------------------------------------
    //
    //   EC  = EC oscillator (External Clock input)
    //   XT  = XT oscillator (Resonator, crystal or resonator)
    //   HS  = HS oscillator (High-speed crystal)
    //   OFF = Disabled
    //
    #pragma config POSCMOD = EC

    // Internal External Switch Over bit: -------------------------------------
    //
    //   OFF = Disabled (Two-Speed Start-up is disabled)
    //   ON  = Enabled (Two-Speed Start-up is enabled)
    //
    #pragma config IESO = OFF

    // Secondary oscillator Enable bit: ---------------------------------------
    //
    //   OFF = Disabled
    //   ON  = Enabled
    //
    #pragma config FSOSCEN = OFF

    // Oscillator Selection bits: ---------------------------------------------
    //
    //   FRC      = Fast RC oscillator
    //   FRCPLL   = Fast RC oscillator w/ PLL
    //   PRI      = Primary oscillator (XT, HS, EC)
    //   PRIPLL   = Primary oscillator (XT, HS, EC) w/ PLL
    //   SOSC     = Secondary oscillator
    //   LPRC     = Low power RC oscillator
    //   FRCDIV16 = Fast RC oscillator with divide by 16
    //   FRCDIV   = Fast RC oscillator with divide
    //
    #pragma config FNOSC = PRIPLL

    // Code Protect bit: ------------------------------------------------------
    //
    //   Prevents boot and program flash memory from being read or modified by
    //   an external programming device.
    //
    //   ON  = Enabled
    //   OFF = Disabled
    //
    #pragma config CP = OFF

    // Boot Flash Write Protect bit: ------------------------------------------
    //
    //   Prevents boot flash memory from being modified during code execution.
    //
    //   ON  = Enabled
    //   OFF = Disabled
    //
    #pragma config BWP = OFF

    // Program Flash Write Protect bits: --------------------------------------
    //
    //   Prevents selected program Flash memory pages from being modified
    //   during code execution. The PWP bits represent the 1’s complement of
    //   the number of write-protected program Flash memory pages.
    //
    //   PWP512K = First 512K | PWP508K = First 508K | PWP504K = First 504K
    //   PWP500K = First 500K | PWP496K = First 496K | PWP492K = First 492K
    //   PWP488K = First 488K | PWP484K = First 484K | PWP480K = First 480K
    //   PWP476K = First 476K | PWP472K = First 472K | PWP468K = First 468K
    //   PWP464K = First 464K | PWP460K = First 460K | PWP456K = First 456K
    //   PWP452K = First 452K | PWP448K = First 448K | PWP444K = First 444K
    //   PWP440K = First 440K | PWP436K = First 436K | PWP432K = First 432K
    //   PWP428K = First 428K | PWP424K = First 424K | PWP420K = First 420K
    //   PWP416K = First 416K | PWP412K = First 412K | PWP408K = First 408K
    //   PWP404K = First 404K | PWP400K = First 400K | PWP396K = First 396K
    //   PWP392K = First 392K | PWP388K = First 388K | PWP384K = First 384K
    //   PWP380K = First 380K | PWP376K = First 376K | PWP372K = First 372K
    //   PWP368K = First 368K | PWP364K = First 364K | PWP360K = First 360K
    //   PWP356K = First 356K | PWP352K = First 352K | PWP348K = First 348K
    //   PWP344K = First 344K | PWP340K = First 340K | PWP336K = First 336K
    //   PWP332K = First 332K | PWP328K = First 328K | PWP324K = First 324K
    //   PWP320K = First 320K | PWP316K = First 316K | PWP312K = First 312K
    //   PWP308K = First 308K | PWP304K = First 304K | PWP300K = First 300K
    //   PWP296K = First 296K | PWP292K = First 292K | PWP288K = First 288K
    //   PWP284K = First 284K | PWP280K = First 280K | PWP276K = First 276K
    //   PWP272K = First 272K | PWP268K = First 268K | PWP264K = First 264K
    //   PWP260K = First 260K | PWP256K = First 256K | PWP252K = First 252K
    //   PWP248K = First 248K | PWP244K = First 244K | PWP240K = First 240K
    //   PWP236K = First 236K | PWP232K = First 232K | PWP228K = First 228K
    //   PWP224K = First 224K | PWP220K = First 220K | PWP216K = First 216K
    //   PWP212K = First 212K | PWP208K = First 208K | PWP204K = First 204K
    //   PWP200K = First 200K | PWP196K = First 196K | PWP192K = First 192K
    //   PWP188K = First 188K | PWP184K = First 184K | PWP180K = First 180K
    //   PWP176K = First 176K | PWP172K = First 172K | PWP168K = First 168K
    //   PWP164K = First 164K | PWP160K = First 160K | PWP156K = First 156K
    //   PWP152K = First 152K | PWP148K = First 148K | PWP144K = First 144K
    //   PWP140K = First 140K | PWP136K = First 136K | PWP132K = First 132K
    //   PWP128K = First 128K | PWP124K = First 124K | PWP120K = First 120K
    //   PWP116K = First 116K | PWP112K = First 112K | PWP108K = First 108K
    //   PWP104K = First 104K | PWP100K = First 100K | PWP96K  = First 96K
    //   PWP92K  = First 92K  | PWP88K  = First 88K  | PWP84K  = First 84K
    //   PWP80K  = First 80K  | PWP76K  = First 76K  | PWP72K  = First 72K
    //   PWP68K  = First 68K  | PWP64K  = First 64K  | PWP60K  = First 60K
    //   PWP56K  = First 56K  | PWP52K  = First 52K  | PWP48K  = First 48K
    //   PWP44K  = First 44K  | PWP40K  = First 40K  | PWP36K  = First 36K
    //   PWP32K  = First 32K  | PWP28K  = First 28K  | PWP24K  = First 24K
    //   PWP20K  = First 20K  | PWP16K  = First 16K  | PWP12K  = First 12K
    //   PWP8K   = First 8K   | PWP4K   = First 4K   |
    //   ON      = Enabled    | OFF     = Disabled   |
    //
    #pragma config PWP = OFF

    // ICE/ICD Comm Channel Select: -------------------------------------------
    //
    //   In-Circuit Emulator/Debugger Communication Channel Select bit.
    //
    //   ICS_PGx1 = ICE pins are shared with PGC1, PGD1
    //   ICS_PGx2 = ICE pins are shared with PGC2, PGD2
    //
    #pragma config ICESEL = ICS_PGx1

    // Background Debugger Enable bit: ----------------------------------------
    //
    //   Background Debugger Enable bits are forced to '11' if
    //   code-protect is enabled.
    //
    //   OFF = Disabled
    //   ON  = Enabled
    //
    #pragma config DEBUG = ON
