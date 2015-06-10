/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    stack_task.h

  Summary:
    

  Description:
    Microchip TCP/IP Stack Definitions

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

#ifndef __STACK_TSK_H_
#define __STACK_TSK_H_

#include "tcpip/tcpip_types.h"

#if defined (WF_CS_TRIS)
    #include "drv_wifi_config.h"
#endif

// Check for potential configuration errors in "tcpip_config.h"
#if (MAX_UDP_SOCKETS <= 0 || MAX_UDP_SOCKETS > 255 )
#error Invalid MAX_UDP_SOCKETS value specified
#endif

// Check for potential configuration errors in "tcpip_config.h"
#if (MAX_HTTP_CONNECTIONS <= 0 || MAX_HTTP_CONNECTIONS > 255 )
#error Invalid MAX_HTTP_CONNECTIONS value specified.
#endif

// Structure to contain a MAC address
typedef struct __attribute__((__packed__))
{
    uint8_t v[6];
} MAC_ADDR;

// Definition to represent an IP address
#define IP_ADDR     TCPIP_UINT32_VAL

// Address structure for a node
typedef struct __attribute__((__packed__))
{
    IP_ADDR     IPAddr;
    MAC_ADDR    MACAddr;
} NODE_INFO;

// Application-dependent structure used to contain address information
typedef struct __attribute__((__packed__)) appConfigStruct
{
    IP_ADDR     MyIPAddr;               // IP address
    IP_ADDR     MyMask;                 // Subnet mask
    IP_ADDR     MyGateway;              // Default Gateway
    IP_ADDR     PrimaryDNSServer;       // Primary DNS Server
    IP_ADDR     SecondaryDNSServer;     // Secondary DNS Server
    IP_ADDR     DefaultIPAddr;          // Default IP address
    IP_ADDR     DefaultMask;            // Default subnet mask
    uint8_t     NetBIOSName[16];        // NetBIOS name
    struct
    {
        unsigned char : 6;
        unsigned char bIsDHCPEnabled : 1;
        unsigned char bInConfigMode : 1;
    } Flags;                            // Flag structure
    MAC_ADDR    MyMACAddr;              // Application MAC address

#if defined(WF_CS_TRIS)
    uint8_t        MySSID[32];             // Wireless SSID (if using MRF24W)
    uint8_t        SsidLength;             // number of bytes in SSID
    uint8_t        SecurityMode;           // WF_SECURITY_OPEN or one of the other security modes
    uint8_t        SecurityKey[64];        // WiFi Security key, or passphrase.
    uint8_t        SecurityKeyLength;      // number of bytes in security key (can be 0)
    uint8_t        WepKeyIndex;            // WEP key index (only valid for WEP)
    uint8_t        dataValid;
    uint8_t        networkType;
    uint8_t        passPhraseToKeyFlag;    // when security mode is WAP with pass phrase, set to 1
    uint8_t        changeSecurityModeFlag; // when mixed mode is not supported, downgrade the security mode
    uint8_t        hibernateFlag;          // flag used in some initialization after hibernate
    #if defined (LED_STOP_BLINKING_IF_CONNECTION_FAILED)
    uint8_t        connectionFailedFlag;   // used to toggle LED0 in WiFi G Demo if connection failed
    #endif
    #if defined(EZ_CONFIG_STORE)           // WLAN configuration data stored to NVM
    uint8_t        saveSecurityInfo;       // Save 32-byte PSK
    #endif
#endif
} APP_CONFIG;

#ifndef THIS_IS_STACK_APPLICATION
    extern APP_CONFIG AppConfig;
#endif

void StackInit(void);
void StackTask(void);
void StackApplications(void);

#endif
