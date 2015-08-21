/*****************************************************************************
  AUTO-GENERATED CODE:  Microchip MPFS Generator version: 2.2.2

  Microchip TCP/IP Stack Application Demo

  Company:
    Microchip Technology Inc.

  File Name:
    http_print.h

  Summary:
    This file is automatically generated by the MPFS Generator Utility.
    ALL MODIFICATIONS WILL BE OVERWRITTEN BY THE MPFS GENERATOR.

  Description:
    Provides callback headers and resolution for user's custom
    HTTP Application.
 *****************************************************************************/

// DOM-IGNORE-BEGIN
/*****************************************************************************
Software License Agreement
Copyright(c) 2014 Microchip Technology Inc. All rights reserved.
Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital
signal controller that is integrated into your product or third party
product (pursuant to the sublicense terms in the accompanying license
agreement).

You should refer to the license agreement accompanying this Software
for additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
OR OTHER SIMILAR COSTS.
 *****************************************************************************/
// DOM-IGNORE-END

#ifndef __HTTPPRINT_H_
#define __HTTPPRINT_H_

/*****************************************************************************
 * SECTION:  Includes
 *****************************************************************************/
#include "tcpip/tcpip.h"

/*****************************************************************************
 * SECTION:  Global Variables
 *****************************************************************************/
#if defined(STACK_USE_HTTP2_SERVER)

extern HTTP_STUB httpStubs[MAX_HTTP_CONNECTIONS];
extern uint8_t curHTTPID;

/*****************************************************************************
 * SECTION:  Generated Function Prototypes
 *****************************************************************************/
void HTTPPrint(uint32_t callbackID);
void HTTPPrint_calVal(void);
void HTTPPrint_calReadResp(void);
void HTTPPrint_calWriteResp(void);
void HTTPPrint_idWriteResp(void);
void HTTPPrint_builddate(void);
void HTTPPrint_nwVal(void);
void HTTPPrint_uptime(void);

/*****************************************************************************
 * FUNCTION: HTTPPrint
 *
 * RETURNS:  None
 *
 * PARAMS:   callbackID
 *****************************************************************************/
void HTTPPrint(uint32_t callbackID)
{
   switch(callbackID)
   {
        case 0x00000000:
			HTTPPrint_calVal();
			break;
        case 0x00000001:
			HTTPPrint_calReadResp();
			break;
        case 0x00000002:
			HTTPPrint_calWriteResp();
			break;
        case 0x00000003:
			HTTPPrint_idWriteResp();
			break;
        case 0x00000004:
			HTTPPrint_builddate();
			break;
        case 0x00000005:
			HTTPPrint_nwVal();
			break;
        case 0x00000006:
			HTTPPrint_uptime();
			break;
       default:
           // Output notification for undefined values
           TCPPutROMArray(sktHTTP, (ROM uint8_t*)"!DEF", 4);
   }

   return;
}

void HTTPPrint_(void)
{
   TCPPut(sktHTTP, '~');
   return;
}

#endif /*STACK_USE_HTTP2_SERVER*/

#endif /*__HTTPPRINT_H_*/
