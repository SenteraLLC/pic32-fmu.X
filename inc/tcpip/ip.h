/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    ip.h

  Summary:
    

  Description:
    IP Defs for Microchip TCP/IP Stack

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

#ifndef __IP_H_
#define __IP_H_


#define IP_PROT_ICMP    (1u)
#define IP_PROT_TCP     (6u)
#define IP_PROT_UDP     (17u)


// IP packet header definition
typedef struct _IP_HEADER
{
    uint8_t     VersionIHL;
    uint8_t     TypeOfService;
    uint16_t    TotalLength;
    uint16_t    Identification;
    uint16_t    FragmentInfo;
    uint8_t     TimeToLive;
    uint8_t     Protocol;
    uint16_t    HeaderChecksum;
    IP_ADDR     SourceAddress;
    IP_ADDR     DestAddress;
} IP_HEADER;


// IP Pseudo header as defined by RFC 793 (needed for TCP and UDP
// checksum calculations/verification)
typedef struct _PSEUDO_HEADER
{
    IP_ADDR SourceAddress;
    IP_ADDR DestAddress;
    uint8_t Zero;
    uint8_t Protocol;
    uint16_t Length;
} PSEUDO_HEADER;

#define SwapPseudoHeader(h)  (h.Length = swaps(h.Length))


/*********************************************************************
 * Function:        bool IPIsTxReady(bool HighPriority)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          true if transmit buffer is empty
 *                  false if transmit buffer is not empty
 *
 * Side Effects:    None
 *
 * Note:            None
 *
 ********************************************************************/
#define IPIsTxReady()       MACIsTxReady()


/*********************************************************************
 * Macro:           IPSetTxBuffer(a, b)
 *
 * PreCondition:    None
 *
 * Input:           a       - Buffer identifier
 *                  b       - Offset
 *
 * Output:          Next Read/Write access to transmit buffer 'a'
 *                  set to offset 'b'
 *
 * Side Effects:    None
 *
 * Note:            None
 *
 ********************************************************************/
#define IPSetTxBuffer(b) MACSetWritePtr(b + BASE_TX_ADDR + sizeof(ETHER_HEADER) + sizeof(IP_HEADER))


/*********************************************************************
 * Function:        uint16_t IPPutHeader(   IP_ADDR *Dest,
 *                                      uint8_t    Protocol,
 *                                      uint16_t    Identifier,
 *                                      uint16_t    DataLen)
 *
 * PreCondition:    IPIsTxReady() == true
 *
 * Input:           Src         - Destination node address
 *                  Protocol    - Current packet protocol
 *                  Identifier  - Current packet identifier
 *                  DataLen     - Current packet data length
 *
 * Output:          Handle to current packet - For use by
 *                  IPSendByte() function.
 *
 * Side Effects:    None
 *
 * Note:            Only one IP message can be transmitted at any
 *                  time.
 *                  Caller may not transmit and receive a message
 *                  at the same time.
 *
 ********************************************************************/
uint16_t    IPPutHeader(NODE_INFO *remote,
                    uint8_t protocol,
                    uint16_t len);


/*********************************************************************
 * Function:        bool IPGetHeader( IP_ADDR    *localIP,
 *                                    NODE_INFO  *remote,
 *                                    uint8_t        *Protocol,
 *                                    uint16_t        *len)
 *
 * PreCondition:    MACGetHeader() == true
 *
 * Input:           localIP     - Local node IP Address as received
 *                                in current IP header.
 *                                If this information is not required
 *                                caller may pass NULL value.
 *                  remote      - Remote node info
 *                  Protocol    - Current packet protocol
 *                  len         - Current packet data length
 *
 * Output:          true, if valid packet was received
 *                  false otherwise
 *
 * Side Effects:    None
 *
 * Note:            Only one IP message can be received.
 *                  Caller may not transmit and receive a message
 *                  at the same time.
 *
 ********************************************************************/
bool IPGetHeader(IP_ADDR *localIP,
                 NODE_INFO *remote,
                 uint8_t *protocol,
                 uint16_t *len);


/*********************************************************************
 * Macro:           IPDiscard()
 *
 * PreCondition:    MACGetHeader() == true
 *
 * Input:           None
 *
 * Output:          Current packet is discarded and buffer is
 *                  freed-up
 *
 * Side Effects:    None
 *
 * Note:            None
 *
 ********************************************************************/
#define IPDiscard()         MACDiscard()


/*********************************************************************
 * Macro:           IPGetArray(a, b)
 *
 * PreCondition:    MACGetHeader() == true
 *
 * Input:           a       - Data buffer
 *                  b       - Buffer length
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            Data is copied from IP data to given buffer
 *
 ********************************************************************/
#define IPGetArray(a, b)    MACGetArray(a, b)


/*********************************************************************
 * Function:        IPSetRxBuffer(uint16_t Offset)
 *
 * PreCondition:    IPHeaderLen must have been intialized by
 *                  IPGetHeader() or IPPutHeader()
 *
 * Input:           Offset from beginning of IP data field
 *
 * Output:          Next Read/Write access to receive buffer is
 *                  set to Offset
 *
 * Side Effects:    None
 *
 * Note:            None
 *
 ********************************************************************/
void IPSetRxBuffer(uint16_t Offset);


#endif
