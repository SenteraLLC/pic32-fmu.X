/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    helpers.h

  Summary:
    

  Description:
    Header file for helpers.c

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

#ifndef __HELPERS_H_
#define __HELPERS_H_

char *strupr(char* s);

// Implement consistent ultoa() function
#if (defined(__XC32) && (__C32_VERSION__ < 112)) || (defined (__XC16) && (__C30_VERSION__ < 325)) || defined(__C30_LEGACY_LIBC__) || defined(__C32_LEGACY_LIBC__)
    // C32 < 1.12 and C30 < v3.25 need this 2 parameter stack implemented function
    void ultoa(uint32_t Value, uint8_t* Buffer);
#else
    // HI-TECH PICC-18 PRO 9.63, C30 v3.25+, and C32 v1.12+ already have a ultoa() stdlib
    // library function, but it requires 3 parameters.  The TCP/IP Stack
    // assumes the C18 style 2 parameter ultoa() function, so we shall
    // create a macro to automatically convert the code.
    #include <stdlib.h>
    #define ultoa(val,buf)  ultoa((char*)(buf),(val),10)
#endif

#if defined(DEBUG)
    #define DebugPrint(a)   {putrsUART(a);}
#else
    #define DebugPrint(a)
#endif

uint32_t    LFSRSeedRand(uint32_t dwSeed);
uint16_t    LFSRRand(void);
uint32_t    GenerateRandomDWORD(void);
void    uitoa(uint16_t Value, uint8_t* Buffer);
void    UnencodeURL(uint8_t* URL);
uint16_t    Base64Decode(uint8_t* cSourceData, uint16_t wSourceLen, uint8_t* cDestData, uint16_t wDestLen);
uint16_t    Base64Encode(uint8_t* cSourceData, uint16_t wSourceLen, uint8_t* cDestData, uint16_t wDestLen);
bool    StringToIPAddress(uint8_t* str, IP_ADDR* IPAddress);
uint8_t     ReadStringUART(uint8_t* Dest, uint8_t BufferLen);
uint8_t hexatob(TCPIP_UINT16_VAL AsciiChars);
uint8_t btohexa_high(uint8_t b);
uint8_t btohexa_low(uint8_t b);
signed char stricmppgm2ram(uint8_t* a, ROM uint8_t* b);
char *  strnchr(const char *searchString, size_t count, char c);
size_t  strncpy_m(char* destStr, size_t destSize, int nStrings, ...);

#if defined(__XC8)
    bool    ROMStringToIPAddress(ROM uint8_t* str, IP_ADDR* IPAddress);
#else
    // Non-ROM variant for PIC24 and PIC32
    #define ROMStringToIPAddress(a,b)   StringToIPAddress((uint8_t*)a,b)
#endif

uint16_t    swaps(uint16_t v);

#if defined(__XC32)
uint32_t   __attribute__((nomips16)) swapl(uint32_t v);
#else
uint32_t   swapl(uint32_t v);
#endif

uint16_t    CalcIPChecksum(uint8_t* buffer, uint16_t len);

#if defined(__XC8)
    uint32_t leftRotateDWORD(uint32_t val, uint8_t bits);
#else
    // Rotations are more efficient in PIC24 and PIC32
    #define leftRotateDWORD(x, n) (((x) << (n)) | ((x) >> (32-(n))))
#endif

void FormatNetBIOSName(uint8_t Name[16]);

// Protocols understood by the ExtractURLFields() function.  IMPORTANT: If you
// need to reorder these (change their constant values), you must also reorder
// the constant arrays in ExtractURLFields().
typedef enum
{
    PROTOCOL_HTTP = 0u,
    PROTOCOL_HTTPS,
    PROTOCOL_MMS,
    PROTOCOL_RTSP
} PROTOCOLS;

uint8_t ExtractURLFields(uint8_t *vURL, PROTOCOLS *protocol, uint8_t *vUsername, uint16_t *wUsernameLen, uint8_t *vPassword, uint16_t *wPasswordLen, uint8_t *vHostname, uint16_t *wHostnameLen, uint16_t *wPort, uint8_t *vFilePath, uint16_t *wFilePathLen);
int16_t Replace(uint8_t *vExpression, ROM uint8_t *vFind, ROM uint8_t *vReplacement, uint16_t wMaxLen, bool bSearchCaseInsensitive);

#endif
