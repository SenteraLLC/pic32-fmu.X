/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    tcp.h

  Summary:
    

  Description:
    TCP Module Defs for Microchip TCP/IP Stack

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

#ifndef __TCP_H_
#define __TCP_H_

/****************************************************************************
  Section:
    Type Definitions
  ***************************************************************************/

// A TCP_SOCKET is stored as a single uint8_t
typedef uint8_t TCP_SOCKET;

#define INVALID_SOCKET      (0xFE)  // The socket is invalid or could not be opened
#define UNKNOWN_SOCKET      (0xFF)  // The socket is not known

/****************************************************************************
  Section:
    State Machine Variables
  ***************************************************************************/

// TCP States as defined by RFC 793
typedef enum
{
    TCP_GET_DNS_MODULE,     // Special state for TCP client mode sockets
    TCP_DNS_RESOLVE,        // Special state for TCP client mode sockets
    TCP_GATEWAY_SEND_ARP,   // Special state for TCP client mode sockets
    TCP_GATEWAY_GET_ARP,    // Special state for TCP client mode sockets

    TCP_LISTEN,             // Socket is listening for connections
    TCP_SYN_SENT,           // A SYN has been sent, awaiting an SYN+ACK
    TCP_SYN_RECEIVED,       // A SYN has been received, awaiting an ACK
    TCP_ESTABLISHED,        // Socket is connected and connection is established
    TCP_FIN_WAIT_1,         // FIN WAIT state 1
    TCP_FIN_WAIT_2,         // FIN WAIT state 2
    TCP_CLOSING,            // Socket is closing
//  TCP_TIME_WAIT, state is not implemented
    TCP_CLOSE_WAIT,         // Waiting to close the socket
    TCP_LAST_ACK,           // The final ACK has been sent
    TCP_CLOSED,             // Socket is idle and unallocated

    TCP_CLOSED_BUT_RESERVED // Special state for TCP client mode sockets.  Socket is idle, but still allocated pending application closure of the handle.
} TCP_STATE;

typedef enum
{
    SSL_NONE = 0,           // No security is enabled
    SSL_HANDSHAKING,        // Handshake is progressing (no application data allowed)
    SSL_ESTABLISHED,        // Connection is established and secured
    SSL_CLOSED              // Connection has been closed (no applicaiton data is allowed)
} SSL_STATE;

/****************************************************************************
  Section:
    TCB Definitions
  ***************************************************************************/

// TCP Control Block (TCB) stub data storage.  Stubs are stored in local PIC RAM for speed.
// Current size is 34 bytes (PIC18), 36 bytes (PIC24), or 56 (PIC32)
typedef struct
{
    PTR_BASE bufferTxStart;     // First byte of TX buffer
    PTR_BASE bufferRxStart;     // First byte of RX buffer.  TX buffer ends 1 byte prior
    PTR_BASE bufferEnd;         // Last byte of RX buffer
    PTR_BASE txHead;            // Head pointer for TX
    PTR_BASE txTail;            // Tail pointer for TX
    PTR_BASE rxHead;            // Head pointer for RX
    PTR_BASE rxTail;            // Tail pointer for RX
    uint32_t eventTime;         // Packet retransmissions, state changes
    uint16_t eventTime2;            // Window updates, automatic transmission
    union
    {
        uint16_t delayedACKTime;    // Delayed Acknowledgement timer
        uint16_t closeWaitTime;     // TCP_CLOSE_WAIT timeout timer
    } OverlappedTimers;
    TCP_STATE smState;          // State of this socket
    struct
    {
        unsigned char vUnackedKeepalives : 3;       // Count of how many keepalives have been sent with no response
        unsigned char bServer : 1;                  // Socket should return to listening state when closed
        unsigned char bTimerEnabled : 1;            // Timer is enabled
        unsigned char bTimer2Enabled : 1;           // Second timer is enabled
        unsigned char bDelayedACKTimerEnabled : 1;  // DelayedACK timer is enabled
        unsigned char bOneSegmentReceived : 1;      // A segment has been received
        unsigned char bHalfFullFlush : 1;           // Flush is for being half full
        unsigned char bTXASAP : 1;                  // Transmit as soon as possible (for Flush)
        unsigned char bTXASAPWithoutTimerReset : 1; // Transmit as soon as possible (for Flush), but do not reset retransmission timers
        unsigned char bTXFIN : 1;                   // FIN needs to be transmitted
        unsigned char bSocketReset : 1;             // Socket has been reset (self-clearing semaphore)
        unsigned char bSSLHandshaking : 1;          // Socket is in an SSL handshake
        unsigned char filler : 2;                   // Future expansion
    } Flags;
    TCPIP_UINT16_VAL remoteHash;    // Consists of remoteIP, remotePort, localPort for connected sockets.  It is a localPort number only for listening server sockets.

    #if defined(STACK_USE_SSL)
    PTR_BASE sslTxHead;     // Position of data being written in next SSL application record
                            //   Also serves as cache of localSSLPort when smState = TCP_LISTENING
    PTR_BASE sslRxHead;     // Position of incoming data not yet handled by SSL
    uint8_t sslStubID;          // Which sslStub is associated with this connection
    uint8_t sslReqMessage;      // Currently requested SSL message
    #endif

    uint8_t vMemoryMedium;      // Which memory medium the TCB is actually stored

} TCB_STUB;

// Remainder of TCP Control Block data.
// The rest of the TCB is stored in Ethernet buffer RAM or elsewhere as defined by vMemoryMedium.
// Current size is 41 (PIC18), 42 (PIC24), or 48 bytes (PIC32)
typedef struct
{
    uint32_t        retryInterval;          // How long to wait before retrying transmission
    uint32_t        MySEQ;                  // Local sequence number
    uint32_t        RemoteSEQ;              // Remote sequence number
    PTR_BASE    txUnackedTail;          // TX tail pointer for data that is not yet acked
    TCPIP_UINT16_VAL    remotePort;             // Remote port number
    TCPIP_UINT16_VAL    localPort;              // Local port number
    uint16_t        remoteWindow;           // Remote window size
    uint16_t        wFutureDataSize;        // How much out-of-order data has been received
    union
    {
        NODE_INFO   niRemoteMACIP;      // 10 bytes for MAC and IP address
        uint32_t        dwRemoteHost;       // RAM or ROM pointer to a hostname string (ex: "www.microchip.com")
    } remote;
    int16_t     sHoleSize;              // Size of the hole, or -1 for none exists.  (0 indicates hole has just been filled)
    struct
    {
        unsigned char bFINSent : 1;     // A FIN has been sent
        unsigned char bSYNSent : 1;     // A SYN has been sent
        unsigned char bRemoteHostIsROM : 1; // Remote host is stored in ROM
        unsigned char bRXNoneACKed1 : 1;    // A duplicate ACK was likely received
        unsigned char bRXNoneACKed2 : 1;    // A second duplicate ACK was likely received
        unsigned char filler : 3;       // future use
    } flags;
    uint16_t        wRemoteMSS;             // Maximum Segment Size option advirtised by the remote node during initial handshaking
    #if defined(STACK_USE_SSL)
    TCPIP_UINT16_VAL    localSSLPort;           // Local SSL port number (for listening sockets)
    #endif
    uint8_t     retryCount;             // Counter for transmission retries
    uint8_t     vSocketPurpose;         // Purpose of socket (as defined in tcpip_config.h)
} TCB;

// Information about a socket
typedef struct
{
    NODE_INFO remote;       // NODE_INFO structure for remote node
    TCPIP_UINT16_VAL remotePort;    // Port number associated with remote node
} SOCKET_INFO;

/****************************************************************************
  Section:
    Function Declarations
  ***************************************************************************/

void TCPInit(void);
SOCKET_INFO* TCPGetRemoteInfo(TCP_SOCKET hTCP);
bool TCPWasReset(TCP_SOCKET hTCP);
bool TCPIsConnected(TCP_SOCKET hTCP);
void TCPDisconnect(TCP_SOCKET hTCP);
void TCPClose(TCP_SOCKET hTCP);
uint16_t TCPIsPutReady(TCP_SOCKET hTCP);
bool TCPPut(TCP_SOCKET hTCP, uint8_t byte);
uint16_t TCPPutArray(TCP_SOCKET hTCP, uint8_t* Data, uint16_t Len);
uint8_t* TCPPutString(TCP_SOCKET hTCP, uint8_t* Data);
uint16_t TCPIsGetReady(TCP_SOCKET hTCP);
uint16_t TCPGetRxFIFOFree(TCP_SOCKET hTCP);
bool TCPGet(TCP_SOCKET hTCP, uint8_t* byte);
uint16_t TCPGetArray(TCP_SOCKET hTCP, uint8_t* buffer, uint16_t count);
uint8_t TCPPeek(TCP_SOCKET hTCP, uint16_t wStart);
uint16_t TCPPeekArray(TCP_SOCKET hTCP, uint8_t *vBuffer, uint16_t wLen, uint16_t wStart);
uint16_t TCPFindEx(TCP_SOCKET hTCP, uint8_t cFind, uint16_t wStart, uint16_t wSearchLen, bool bTextCompare);
uint16_t TCPFindArrayEx(TCP_SOCKET hTCP, uint8_t* cFindArray, uint16_t wLen, uint16_t wStart, uint16_t wSearchLen, bool bTextCompare);
void TCPDiscard(TCP_SOCKET hTCP);
bool TCPProcess(NODE_INFO* remote, IP_ADDR* localIP, uint16_t len);
void TCPTick(void);
void TCPFlush(TCP_SOCKET hTCP);

// Create a server socket and ignore dwRemoteHost.
#define TCP_OPEN_SERVER     0u
#if defined(STACK_CLIENT_MODE)
    #if defined(STACK_USE_DNS)
        // Create a client socket and use dwRemoteHost as a RAM pointer to a hostname string.
        #define TCP_OPEN_RAM_HOST   1u
        // Create a client socket and use dwRemoteHost as a ROM pointer to a hostname string.
        #define TCP_OPEN_ROM_HOST   2u
    #else
        // Emit an undeclared identifier diagnostic if code tries to use TCP_OPEN_RAM_HOST while the DNS client module is not enabled.
        #define TCP_OPEN_RAM_HOST   You_need_to_enable_STACK_USE_DNS_to_use_TCP_OPEN_RAM_HOST
        // Emit an undeclared identifier diagnostic if code tries to use TCP_OPEN_ROM_HOST while the DNS client module is not enabled.
        #define TCP_OPEN_ROM_HOST   You_need_to_enable_STACK_USE_DNS_to_use_TCP_OPEN_ROM_HOST
    #endif
    // Create a client socket and use dwRemoteHost as a literal IP address.
    #define TCP_OPEN_IP_ADDRESS 3u
    // Create a client socket and use dwRemoteHost as a pointer to a NODE_INFO structure containing the exact remote IP address and MAC address to use.
    #define TCP_OPEN_NODE_INFO  4u
#else
    // Emit an undeclared identifier diagnostic if code tries to use TCP_OPEN_RAM_HOST while STACK_CLIENT_MODE feature is not enabled.
    #define TCP_OPEN_RAM_HOST   You_need_to_enable_STACK_CLIENT_MODE_to_use_TCP_OPEN_RAM_HOST
    // Emit an undeclared identifier diagnostic if code tries to use TCP_OPEN_ROM_HOST while STACK_CLIENT_MODE feature is not enabled.
    #define TCP_OPEN_ROM_HOST   You_need_to_enable_STACK_CLIENT_MODE_to_use_TCP_OPEN_ROM_HOST
    // Emit an undeclared identifier diagnostic if code tries to use TCP_OPEN_IP_ADDRESS while STACK_CLIENT_MODE feature is not enabled.
    #define TCP_OPEN_IP_ADDRESS You_need_to_enable_STACK_CLIENT_MODE_to_use_TCP_OPEN_IP_ADDRESS
    // Emit an undeclared identifier diagnostic if code tries to use TCP_OPEN_NODE_INFO while STACK_CLIENT_MODE feature is not enabled.
    #define TCP_OPEN_NODE_INFO  You_need_to_enable_STACK_CLIENT_MODE_to_use_TCP_OPEN_NODE_INFO
#endif
TCP_SOCKET TCPOpen(uint32_t dwRemoteHost, uint8_t vRemoteHostType, uint16_t wPort, uint8_t vSocketPurpose);

#if defined(__XC8)
    uint16_t TCPFindROMArrayEx(TCP_SOCKET hTCP, ROM uint8_t* cFindArray, uint16_t wLen, uint16_t wStart, uint16_t wSearchLen, bool bTextCompare);

    /*****************************************************************************
      Summary:
        Alias to TCPFindROMArrayEx with no length parameter.

      Description:
        This function is an alias to TCPFindROMArrayEx with no length parameter.
        It is provided for backwards compatibility with an older API.
      ***************************************************************************/
    #define TCPFindROMArray(a,b,c,d,e)      TCPFindROMArrayEx(a,b,c,d,0,e)

    uint16_t TCPPutROMArray(TCP_SOCKET hTCP, ROM uint8_t* Data, uint16_t Len);
    ROM uint8_t* TCPPutROMString(TCP_SOCKET hTCP, ROM uint8_t* Data);
#else
    #define TCPFindROMArray(a,b,c,d,e)      TCPFindArray(a,(uint8_t*)b,c,d,e)
    #define TCPFindROMArrayEx(a,b,c,d,e,f)  TCPFindArrayEx(a,(uint8_t*)b,c,d,e,f)
    #define TCPPutROMArray(a,b,c)           TCPPutArray(a,(uint8_t*)b,c)
    #define TCPPutROMString(a,b)            TCPPutString(a,(uint8_t*)b)
#endif

uint16_t TCPGetTxFIFOFull(TCP_SOCKET hTCP);
// Alias to TCPIsGetReady provided for API completeness
#define TCPGetRxFIFOFull(a)                 TCPIsGetReady(a)
// Alias to TCPIsPutReady provided for API completeness
#define TCPGetTxFIFOFree(a)                 TCPIsPutReady(a)

#define TCP_ADJUST_GIVE_REST_TO_RX  0x01u   // Resize flag: extra bytes go to RX
#define TCP_ADJUST_GIVE_REST_TO_TX  0x02u   // Resize flag: extra bytes go to TX
#define TCP_ADJUST_PRESERVE_RX      0x04u   // Resize flag: attempt to preserve RX buffer
#define TCP_ADJUST_PRESERVE_TX      0x08u   // Resize flag: attempt to preserve TX buffer
bool TCPAdjustFIFOSize(TCP_SOCKET hTCP, uint16_t wMinRXSize, uint16_t wMinTXSize, uint8_t vFlags);

#if defined(STACK_USE_SSL)
bool TCPStartSSLClient(TCP_SOCKET hTCP, uint8_t* host);
bool TCPStartSSLClientEx(TCP_SOCKET hTCP, uint8_t* host, void * buffer, uint8_t suppDataType);
bool TCPStartSSLServer(TCP_SOCKET hTCP);
bool TCPAddSSLListener(TCP_SOCKET hTCP, uint16_t port);
bool TCPRequestSSLMessage(TCP_SOCKET hTCP, uint8_t msg);
bool TCPSSLIsHandshaking(TCP_SOCKET hTCP);
bool TCPIsSSL(TCP_SOCKET hTCP);
void TCPSSLHandshakeComplete(TCP_SOCKET hTCP);
void TCPSSLDecryptMAC(TCP_SOCKET hTCP, ARCFOUR_CTX* ctx, uint16_t len);
void TCPSSLInPlaceMACEncrypt(TCP_SOCKET hTCP, ARCFOUR_CTX* ctx, uint8_t* MACSecret, uint16_t len);
void TCPSSLPutRecordHeader(TCP_SOCKET hTCP, uint8_t* hdr, bool recDone);
uint16_t TCPSSLGetPendingTxSize(TCP_SOCKET hTCP);
void TCPSSLHandleIncoming(TCP_SOCKET hTCP);
#endif

/*****************************************************************************
  Summary:
    Alias to TCPFindEx with no length parameter.

  Description:
    This function is an alias to TCPFindEx with no length parameter.  It is
    provided for backwards compatibility with an older API.
  ***************************************************************************/
#define TCPFind(a,b,c,d)                    TCPFindEx(a,b,c,0,d)


/*****************************************************************************
  Summary:
    Alias to TCPFindArrayEx with no length parameter.

  Description:
    This function is an alias to TCPFindArrayEx with no length parameter.  It is
    provided for backwards compatibility with an older API.
  ***************************************************************************/
#define TCPFindArray(a,b,c,d,e)             TCPFindArrayEx(a,b,c,d,0,e)

/*****************************************************************************
  Summary:
    Alias to TCPOpen as a server.

  Description:
    This function is an alias to TCPOpen for server sockets.  It is provided
    for backwards compatibility with older versions of the stack.  New
    applications should use the TCPOpen API instead.
  ***************************************************************************/
#define TCPListen(port)         TCPOpen(0, TCP_OPEN_SERVER, port, TCP_PURPOSE_DEFAULT)

/*****************************************************************************
  Summary:
    Alias to TCPOpen as a client.

  Description:
    This function is an alias to TCPOpen for client sockets.  It is provided
    for backwards compatibility with older versions of the stack.  New
    applications should use the TCPOpen API instead.
  ***************************************************************************/
#define TCPConnect(remote,port) TCPOpen((uint32_t)remote, TCP_OPEN_NODE_INFO, port, TCP_PURPOSE_DEFAULT)


#endif
