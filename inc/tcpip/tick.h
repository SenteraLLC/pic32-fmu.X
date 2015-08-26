/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    tick.h

  Summary:
    

  Description:
    Tick Manager for PIC18

 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) <2014> released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#ifndef __TICK_H_
#define __TICK_H_

// All TICKS are stored as 32-bit unsigned integers.
// This is deprecated since it conflicts with other TICK definitions used in
// other Microchip software libraries and therefore poses a merge and maintence
// problem.  Instead of using the TICK data type, just use the base uint32_t data
// type instead.
typedef __attribute__((__deprecated__)) uint32_t TICK;

// This value is used by TCP and other modules to implement timeout actions.
// For this definition, the Timer must be initialized to use a 1:256 prescalar
// in tick.c.  If using a 32kHz watch crystal as the time base, modify the
// tick.c file to use no prescalar.
#define TICKS_PER_SECOND        ((GetPeripheralClock()+128ull)/256ull)  // Internal core clock drives timer with 1:256 prescaler
//#define TICKS_PER_SECOND      (32768ul)                               // 32kHz crystal drives timer with no scalar

#if defined(__XC8)
// Represents one second in Ticks
#define TICK_SECOND             (TICKS_PER_SECOND)
// Represents one minute in Ticks
#define TICK_MINUTE             (TICKS_PER_SECOND*60ull)
// Represents one hour in Ticks
#define TICK_HOUR               (TICKS_PER_SECOND*3600ull)
#else
// Represents one second in Ticks
#define TICK_SECOND             ((uint64_t)TICKS_PER_SECOND)
// Represents one minute in Ticks
#define TICK_MINUTE             ((uint64_t)TICKS_PER_SECOND*60ull)
// Represents one hour in Ticks
#define TICK_HOUR               ((uint64_t)TICKS_PER_SECOND*3600ull)
#endif /* __XC8 */

void TickInit(void);
uint32_t TickGet(void);
uint32_t TickGetDiv256(void);
uint32_t TickGetDiv64K(void);
uint32_t TickConvertToMilliseconds(uint32_t dwTickValue);
void TickUpdate(void);

#endif
