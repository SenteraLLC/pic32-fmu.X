/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    icmp.h

  Summary:
    

  Description:
    ICMP Module Defs for Microchip TCP/IP Stack

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

#ifndef __ICMP_H_
#define __ICMP_H_

void ICMPProcess(NODE_INFO *remote, uint16_t len);

bool ICMPBeginUsage(void);
void ICMPSendPing(uint32_t dwRemoteIP);
void ICMPSendPingToHost(uint8_t * szRemoteHost);
int32_t ICMPGetReply(void);
void ICMPEndUsage(void);

#if defined(__XC8)
    void ICMPSendPingToHostROM(ROM uint8_t * szRemoteHost);
#else
    #define ICMPSendPingToHostROM(a)    ICMPSendPingToHost((uint8_t*)(a))
#endif

#endif
