/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    udp.h

  Summary:
    

  Description:
    UDP Module Defs for Microchip TCP/IP Stack

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

#ifndef __UDP_H_
#define __UDP_H_

// Stores a UDP Port Number
typedef uint16_t UDP_PORT;

// Provides a handle to a UDP Socket
typedef uint8_t UDP_SOCKET;

// UDP States
typedef enum
{
    UDP_DNS_IS_RESOLVED,        // Special state for UDP client mode sockets
    UDP_DNS_RESOLVE,        // Special state for UDP client mode sockets
    UDP_GATEWAY_SEND_ARP,   // Special state for UDP client mode sockets
    UDP_GATEWAY_GET_ARP,        // Special state for UDP client mode sockets
    UDP_CLOSED,             // Socket is idle and unallocated
    UDP_OPENED
} UDP_STATE;


// Stores information about a current UDP socket
typedef struct
{
    union
    {
        NODE_INFO   remoteNode;     // 10 bytes for MAC and IP address
        uint32_t        remoteHost;     // RAM or ROM pointer to a hostname string (ex: "www.microchip.com")
    } remote;
    //NODE_INFO   remoteNode;       // IP and MAC of remote node
    UDP_PORT    remotePort;     // Remote node's UDP port number
    UDP_PORT    localPort;      // Local UDP port number, or INVALID_UDP_PORT when free
    UDP_STATE smState;          // State of this socket
    uint32_t retryInterval;
    uint8_t retryCount;
    struct
    {
        unsigned char bRemoteHostIsROM : 1; // Remote host is stored in ROM
    }flags;
    uint16_t eventTime;
} UDP_SOCKET_INFO;


#define INVALID_UDP_SOCKET      (0xffu)     // Indicates a UDP socket that is not valid
#define INVALID_UDP_PORT        (0ul)       // Indicates a UDP port that is not valid

/****************************************************************************
  Section:
    External Global Variables
  ***************************************************************************/
#if !defined(__UDP_C_)
    extern UDP_SOCKET activeUDPSocket;
    extern UDP_SOCKET_INFO  UDPSocketInfo[MAX_UDP_SOCKETS];
    extern uint16_t UDPTxCount;
    extern uint16_t UDPRxCount;
#endif

// Stores the header of a UDP packet
typedef struct
{
    UDP_PORT    SourcePort;             // Source UDP port
    UDP_PORT    DestinationPort;        // Destination UDP port
    uint16_t        Length;                 // Length of data
    uint16_t        Checksum;               // UDP checksum of the data
} UDP_HEADER;


// Create a server socket and ignore dwRemoteHost.
#define UDP_OPEN_SERVER     0u
#if defined(STACK_CLIENT_MODE)
    #if defined(STACK_USE_DNS)
        // Create a client socket and use dwRemoteHost as a RAM pointer to a hostname string.
        #define UDP_OPEN_RAM_HOST   1u
        // Create a client socket and use dwRemoteHost as a ROM pointer to a hostname string.
        #define UDP_OPEN_ROM_HOST   2u
    #else
        // Emit an undeclared identifier diagnostic if code tries to use UDP_OPEN_RAM_HOST while the DNS client module is not enabled.
        #define UDP_OPEN_RAM_HOST   You_need_to_enable_STACK_USE_DNS_to_use_UDP_OPEN_RAM_HOST
        // Emit an undeclared identifier diagnostic if code tries to use UDP_OPEN_ROM_HOST while the DNS client module is not enabled.
        #define UDP_OPEN_ROM_HOST   You_need_to_enable_STACK_USE_DNS_to_use_UDP_OPEN_ROM_HOST
    #endif
    // Create a client socket and use dwRemoteHost as a literal IP address.
    #define UDP_OPEN_IP_ADDRESS 3u
#endif

// Create a client socket and use dwRemoteHost as a pointer to a NODE_INFO structure containing the exact remote IP address and MAC address to use.
#define UDP_OPEN_NODE_INFO  4u


/****************************************************************************
  Section:
    Function Prototypes
  ***************************************************************************/
void UDPInit(void);
void UDPTask(void);
UDP_SOCKET UDPOpenEx(uint32_t remoteHost, uint8_t remoteHostType, UDP_PORT localPort,UDP_PORT remotePort);

//UDP_SOCKET UDPOpen(UDP_PORT localPort, NODE_INFO *remoteNode, UDP_PORT remotePort);
void UDPClose(UDP_SOCKET s);
bool UDPProcess(NODE_INFO *remoteNode, IP_ADDR *localIP, uint16_t len);

void UDPSetTxBuffer(uint16_t wOffset);
void UDPSetRxBuffer(uint16_t wOffset);
uint16_t UDPIsPutReady(UDP_SOCKET s);
bool UDPPut(uint8_t v);
uint16_t UDPPutArray(uint8_t *cData, uint16_t wDataLen);
uint8_t* UDPPutString(uint8_t *strData);
void UDPFlush(void);

// ROM function variants for PIC18
#if defined(__XC8)
    uint16_t UDPPutROMArray(ROM uint8_t *cData, uint16_t wDataLen);
    ROM uint8_t* UDPPutROMString(ROM uint8_t *strData);
#else
    #define UDPPutROMArray(a,b) UDPPutArray((uint8_t*)a,b)
    #define UDPPutROMString(a)  UDPPutString((uint8_t*)a)
#endif

uint16_t UDPIsGetReady(UDP_SOCKET s);
bool UDPGet(uint8_t *v);
uint16_t UDPGetArray(uint8_t *cData, uint16_t wDataLen);
void UDPDiscard(void);
bool UDPIsOpened(UDP_SOCKET socket);

/*****************************************************************************
  Function:
    UDP_SOCKET UDPOpen(UDP_PORT localPort, NODE_INFO* remoteNode,
                        UDP_PORT remotePort)

  Summary:
    Macro of the legacy version of UDPOpen.

  Description:
    UDPOpen is a macro replacement of the legacy implementation of UDPOpen.
    Creates a UDP socket handle for transmiting or receiving UDP packets.
    Call this function to obtain a handle required by other UDP function.

  Precondition:
    UDPInit() must have been previously called.

  Parameters:
    localPort - UDP port number to listen on.  If 0, stack will dynamically
        assign a unique port number to use.
    remoteNode - Pointer to remote node info (MAC and IP address) for this
        connection.  If this is a server socket (receives the first packet)
        or the destination is the broadcast address, then this parameter
        should be NULL.
    remotePort - For client sockets, the remote port number.

  Return Values:
    Success -
        A UDP socket handle that can be used for subsequent UDP API calls.
    Failure -
        INVALID_UDP_SOCKET.  This function fails when no more UDP socket
        handles are available.  Increase MAX_UDP_SOCKETS to make more sockets
        available.

  Remarks:
    When finished using the UDP socket handle, call the UDPClose() function
    to free the socket and delete the handle.
  ***************************************************************************/
#define UDPOpen(localPort,remoteNode,remotePort)  UDPOpenEx((uint32_t)remoteNode,UDP_OPEN_NODE_INFO,localPort,remotePort)

#endif
