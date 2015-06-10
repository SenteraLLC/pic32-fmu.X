/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    arp.h

  Summary:
    

  Description:
    ARP Module Defs for Microchip TCP/IP Stack

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

#ifndef __ARP_H_
#define __ARP_H_

#ifdef STACK_CLIENT_MODE
    void ARPInit(void);
#else
    #define ARPInit()
#endif

#define ARP_OPERATION_REQ       0x0001u     // Operation code indicating an ARP Request
#define ARP_OPERATION_RESP      0x0002u     // Operation code indicating an ARP Response

#define HW_ETHERNET             (0x0001u)   // ARP Hardware type as defined by IEEE 802.3
#define ARP_IP                  (0x0800u)   // ARP IP packet type as defined by IEEE 802.3

// ARP packet structure
typedef struct __attribute__((aligned(2), packed))
{
    uint16_t    HardwareType;   // Link-layer protocol type (Ethernet is 1).
    uint16_t    Protocol;       // The upper-layer protocol issuing an ARP request (0x0800 for IPv4)..
    uint8_t     MACAddrLen;     // MAC address length (6).
    uint8_t     ProtocolLen;    // Length of addresses used in the upper-layer protocol (4).
    uint16_t    Operation;      // The operation the sender is performing (ARP_REQ or ARP_RESP).
    MAC_ADDR    SenderMACAddr;  // The sender's hardware (MAC) address.
    IP_ADDR     SenderIPAddr;   // The sender's IP address.
    MAC_ADDR    TargetMACAddr;  // The target node's hardware (MAC) address.
    IP_ADDR     TargetIPAddr;   // The target node's IP address.
} ARP_PACKET;

bool ARPProcess(void);
void ARPResolve(IP_ADDR* IPAddr);
bool ARPIsResolved(IP_ADDR* IPAddr, MAC_ADDR* MACAddr);
void SwapARPPacket(ARP_PACKET* p);

#ifdef STACK_USE_ZEROCONF_LINK_LOCAL
    /* API specific Definitions */
    #define ARP_REQ       0x0001u       // Operation code indicating an ARP Request
    #define ARP_RESP      0x0002u       // Operation code indicating an ARP Response

    struct arp_app_callbacks {
        bool used;
        void (*ARPPkt_notify)(uint32_t SenderIPAddr, uint32_t TargetIPAddr,
                            MAC_ADDR* SenderMACAddr, MAC_ADDR* TargetMACAddr, uint8_t op_req);
    };
    int8_t ARPRegisterCallbacks(struct arp_app_callbacks *app);
    bool ARPDeRegisterCallbacks(int8_t id);
#endif
    bool ARPSendPkt(uint32_t SrcIPAddr, uint32_t DestIPAddr, uint8_t op_req );
#endif
