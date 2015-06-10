/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    mac.h

  Summary:
    

  Description:
    MAC Module Defs for Microchip Stack

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

#ifndef __MAC_H_
#define __MAC_H_

//#include "system_config.h"

#if defined(WF_CS_TRIS)
    // Do not use the DMA and other goodies that Microchip Ethernet modules have
    #define NON_MCHP_MAC
#endif

#if defined(ENC_CS_TRIS) && defined(WF_CS_TRIS)
    #error "Error in system_config.h.  Must select either the ENC28J60 or the MRF24WB10 but not both ENC_CS_TRIS and WF_CS_TRIS."
#endif
#if defined(ENC100_INTERFACE_MODE) && defined(WF_CS_TRIS)
    #error "Error in system_config.h.  Must select either the ENCX24J600 or the MRF24WB10 but not both ENC100_INTERFACE_MODE and WF_CS_TRIS."
#endif
#if defined(ENC100_INTERFACE_MODE) && defined(ENC_CS_TRIS)
    #error "Error in system_config.h.  Must select either the ENC28J60 or the ENCX24J600 but not both ENC_CS_TRIS and ENC100_INTERFACE_MODE."
#endif


#if !defined(ENC_CS_TRIS) && !defined(WF_CS_TRIS) && !defined(ENC100_INTERFACE_MODE) && \
     (defined(__18F97J60) || defined(__18F96J65) || defined(__18F96J60) || defined(__18F87J60) || defined(__18F86J65) || defined(__18F86J60) || defined(__18F67J60) || defined(__18F66J65) || defined(__18F66J60) || \
      defined(_18F97J60) ||  defined(_18F96J65) ||  defined(_18F96J60) ||  defined(_18F87J60) ||  defined(_18F86J65) ||  defined(_18F86J60) ||  defined(_18F67J60) ||  defined(_18F66J65) ||  defined(_18F66J60))
    #include "eth97j60.h"
#elif defined(ENC_CS_TRIS) || defined(WF_CS_TRIS)
        #include "tcpip/src/enc28j60.h"
#elif defined(ENC100_INTERFACE_MODE)
    #include "tcpip/src/encx24j600.h"
    #define PHYREG uint16_t
#elif defined(__XC32) && defined(_ETH)
    // extra includes for PIC32MX with embedded ETH Controller
#else
    #error No Ethernet/WiFi controller defined in HardwareProfile.h.  Defines for an ENC28J60, ENC424J600/624J600, or WiFi MRF24WB10 must be present.
#endif


#define MAC_TX_BUFFER_SIZE          (1500ul)

// A generic structure representing the Ethernet header starting all Ethernet
// frames
typedef struct  __attribute__((aligned(2), packed))
{
    MAC_ADDR        DestMACAddr;
    MAC_ADDR        SourceMACAddr;
    TCPIP_UINT16_VAL        Type;
} ETHER_HEADER;


#define MAC_IP          (0x00u)
#define MAC_ARP         (0x06u)
#define MAC_UNKNOWN     (0xFFu)


#if !defined(STACK_USE_HTTP2_SERVER)
    #define RESERVED_HTTP_MEMORY 0ul
#endif

#if !defined(STACK_USE_SSL)
    #define RESERVED_SSL_MEMORY 0ul
#endif

#if defined(WF_CS_TRIS)
    #define MAX_PACKET_SIZE     (1514ul)
#endif

// MAC RAM definitions
#if defined(ENC100_INTERFACE_MODE)
    #define RESERVED_CRYPTO_MEMORY  (128ul)
    #define RAMSIZE         (24*1024ul)
    #define TXSTART         (0x0000ul)
    #define RXSTART         ((TXSTART + 1518ul + TCP_ETH_RAM_SIZE + RESERVED_HTTP_MEMORY + RESERVED_SSL_MEMORY + RESERVED_CRYPTO_MEMORY + 1ul) & 0xFFFE)
    #define RXSTOP          (RAMSIZE-1ul)
    #define RXSIZE          (RXSTOP-RXSTART+1ul)
    #define BASE_TX_ADDR    (TXSTART)
    #define BASE_TCB_ADDR   (BASE_TX_ADDR + 1518ul)
    #define BASE_HTTPB_ADDR (BASE_TCB_ADDR + TCP_ETH_RAM_SIZE)
    #define BASE_SSLB_ADDR  (BASE_HTTPB_ADDR + RESERVED_HTTP_MEMORY)
    #define BASE_CRYPTOB_ADDR   (BASE_SSLB_ADDR + RESERVED_SSL_MEMORY)
#elif defined(WF_CS_TRIS)
    #define RAMSIZE         (14170ul - 8192ul - RESERVED_HTTP_MEMORY - RESERVED_SSL_MEMORY)
    #define TXSTART         (RAMSIZE - (4ul + MAX_PACKET_SIZE + 4ul))
    #define RXSTART         (0ul)
    #define RXSTOP          ((TXSTART-2ul) | 0x0001ul)
    #define RXSIZE          (RXSTOP-RXSTART+1ul)
    #define BASE_TX_ADDR    (TXSTART + 4ul)
    #define BASE_SCRATCH_ADDR (BASE_TX_ADDR + (MAX_PACKET_SIZE + 4ul))
    #define BASE_HTTPB_ADDR  (BASE_SCRATCH_ADDR)
    #define BASE_SSLB_ADDR  (BASE_HTTPB_ADDR + RESERVED_HTTP_MEMORY)
    #define BASE_TCB_ADDR   (BASE_SSLB_ADDR + RESERVED_SSL_MEMORY)
#elif defined(__XC32) && defined(_ETH) && !defined(ENC_CS_TRIS)
    #define BASE_TX_ADDR    (MACGetTxBaseAddr())
    #define BASE_HTTPB_ADDR (MACGetHttpBaseAddr())
    #define BASE_SSLB_ADDR  (MACGetSslBaseAddr())
    #define RXSIZE          (EMAC_RX_BUFF_SIZE)
    #define RAMSIZE         (2*RXSIZE)  // not used but silences the compiler

    #define BASE_TCB_ADDR   (BASE_TX_ADDR + (1514ul+7ul))

#else   // ENC28J60 or PIC18F97J60 family internal Ethernet controller
    #define RAMSIZE         (8*1024ul)
    #define TXSTART         (RAMSIZE - (1ul+1518ul+7ul) - TCP_ETH_RAM_SIZE - RESERVED_HTTP_MEMORY - RESERVED_SSL_MEMORY)
    #define RXSTART         (0ul)                       // Should be an even memory address; must be 0 for errata
    #define RXSTOP          ((TXSTART-2ul) | 0x0001ul)  // Odd for errata workaround
    #define RXSIZE          (RXSTOP-RXSTART+1ul)
    #define BASE_TX_ADDR    (TXSTART + 1ul)
    #define BASE_TCB_ADDR   (BASE_TX_ADDR + (1514ul+7ul))
    #define BASE_HTTPB_ADDR (BASE_TCB_ADDR + TCP_ETH_RAM_SIZE)
    #define BASE_SSLB_ADDR  (BASE_HTTPB_ADDR + RESERVED_HTTP_MEMORY)
#endif

#if (RXSIZE < 1400) || (RXSIZE > RAMSIZE)
    #error Warning, Ethernet RX buffer is tiny.  Reduce TCP socket count, the size of each TCP socket, or move sockets to a different RAM
#endif

uint16_t    MACCalcRxChecksum(uint16_t offset, uint16_t len);
uint16_t    CalcIPBufferChecksum(uint16_t len);

void    MACPowerDown(void);
void    MACEDPowerDown(void);
void    MACPowerUp(void);
#if defined(ENC_CS_TRIS) || defined(ENC100_INTERFACE_MODE) || \
    (defined(__18F97J60) || defined(__18F96J65) || defined(__18F96J60) || defined(__18F87J60) || defined(__18F86J65) || defined(__18F86J60) || defined(__18F67J60) || defined(__18F66J65) || defined(__18F66J60) || \
      defined(_18F97J60) ||  defined(_18F96J65) ||  defined(_18F96J60) ||  defined(_18F87J60) ||  defined(_18F86J65) ||  defined(_18F86J60) ||  defined(_18F67J60) ||  defined(_18F66J65) ||  defined(_18F66J60))
void    WritePHYReg(uint8_t Register, uint16_t Data);
PHYREG  ReadPHYReg(uint8_t Register);
#endif
void    SetRXHashTableEntry(MAC_ADDR DestMACAddr);

// ENC28J60 specific
void    SetCLKOUT(uint8_t NewConfig);
uint8_t GetCLKOUT(void);

/******************************************************************************
 * Macro:           void SetLEDConfig(uint16_t NewConfig)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *
 * Input:           NewConfig - xxx0: Pulse stretching disabled
 *                              xxx2: Pulse stretch to 40ms (default)
 *                              xxx6: Pulse stretch to 73ms
 *                              xxxA: Pulse stretch to 139ms
 *
 *                              xx1x: LEDB - TX
 *                              xx2x: LEDB - RX (default)
 *                              xx3x: LEDB - collisions
 *                              xx4x: LEDB - link
 *                              xx5x: LEDB - duplex
 *                              xx7x: LEDB - TX and RX
 *                              xx8x: LEDB - on
 *                              xx9x: LEDB - off
 *                              xxAx: LEDB - blink fast
 *                              xxBx: LEDB - blink slow
 *                              xxCx: LEDB - link and RX
 *                              xxDx: LEDB - link and TX and RX
 *                              xxEx: LEDB - duplex and collisions
 *
 *                              x1xx: LEDA - TX
 *                              x2xx: LEDA - RX
 *                              x3xx: LEDA - collisions
 *                              x4xx: LEDA - link (default)
 *                              x5xx: LEDA - duplex
 *                              x7xx: LEDA - TX and RX
 *                              x8xx: LEDA - on
 *                              x9xx: LEDA - off
 *                              xAxx: LEDA - blink fast
 *                              xBxx: LEDA - blink slow
 *                              xCxx: LEDA - link and RX
 *                              xDxx: LEDA - link and TX and RX
 *                              xExx: LEDA - duplex and collisions
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Writes the value of NewConfig into the PHLCON PHY register.
 *                  The LED pins will beginning outputting the new
 *                  configuration immediately.
 *
 * Note:
 *****************************************************************************/
#define SetLEDConfig(NewConfig)     WritePHYReg(PHLCON, NewConfig)


/******************************************************************************
 * Macro:           uint16_t GetLEDConfig(void)
 *
 * PreCondition:    SPI bus must be initialized (done in MACInit()).
 *
 * Input:           None
 *
 * Output:          uint16_t -  xxx0: Pulse stretching disabled
 *                          xxx2: Pulse stretch to 40ms (default)
 *                          xxx6: Pulse stretch to 73ms
 *                          xxxA: Pulse stretch to 139ms
 *
 *                          xx1x: LEDB - TX
 *                          xx2x: LEDB - RX (default)
 *                          xx3x: LEDB - collisions
 *                          xx4x: LEDB - link
 *                          xx5x: LEDB - duplex
 *                          xx7x: LEDB - TX and RX
 *                          xx8x: LEDB - on
 *                          xx9x: LEDB - off
 *                          xxAx: LEDB - blink fast
 *                          xxBx: LEDB - blink slow
 *                          xxCx: LEDB - link and RX
 *                          xxDx: LEDB - link and TX and RX
 *                          xxEx: LEDB - duplex and collisions
 *
 *                          x1xx: LEDA - TX
 *                          x2xx: LEDA - RX
 *                          x3xx: LEDA - collisions
 *                          x4xx: LEDA - link (default)
 *                          x5xx: LEDA - duplex
 *                          x7xx: LEDA - TX and RX
 *                          x8xx: LEDA - on
 *                          x9xx: LEDA - off
 *                          xAxx: LEDA - blink fast
 *                          xBxx: LEDA - blink slow
 *                          xCxx: LEDA - link and RX
 *                          xDxx: LEDA - link and TX and RX
 *                          xExx: LEDA - duplex and collisions
 *
 * Side Effects:    None
 *
 * Overview:        Returns the current value of the PHLCON register.
 *
 * Note:            None
 *****************************************************************************/
#define GetLEDConfig()      ReadPHYReg(PHLCON).Val


void MACInit(void);
void MACProcess(void);
bool MACIsLinked(void);

bool MACGetHeader(MAC_ADDR *remote, uint8_t* type);
void MACSetReadPtrInRx(uint16_t offset);
PTR_BASE MACSetWritePtr(PTR_BASE address);
PTR_BASE MACSetReadPtr(PTR_BASE address);
uint8_t MACGet(void);
uint16_t MACGetArray(uint8_t *val, uint16_t len);
void MACDiscardRx(void);
uint16_t MACGetFreeRxSize(void);
void MACMemCopyAsync(PTR_BASE destAddr, PTR_BASE sourceAddr, uint16_t len);
bool MACIsMemCopyDone(void);

void MACPutHeader(MAC_ADDR *remote, uint8_t type, uint16_t dataLen);
bool MACIsTxReady(void);
void MACPut(uint8_t val);
void MACPutArray(uint8_t *val, uint16_t len);
void MACFlush(void);


// ROM function variants for PIC18
#if defined(__XC8)
    void MACPutROMArray(ROM uint8_t *val, uint16_t len);
#else
    #define MACPutROMArray(a,b) MACPutArray((uint8_t*)a,b)
#endif

// PIC32MX with embedded ETHC functions
#if defined(__XC32) && defined(_ETH)
    PTR_BASE MACGetTxBaseAddr(void);
    PTR_BASE MACGetHttpBaseAddr(void);
    PTR_BASE MACGetSslBaseAddr(void);
#endif

#endif
