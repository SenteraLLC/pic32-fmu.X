/*********************************************************************
 *
 *                  External Phy API header file
 *
 *********************************************************************
 * FileName:        ETHPIC32ExtPhy.h
 * Dependencies:
 * Processor:       PIC32
 *
 * Complier:        MPLAB C32
 *                  MPLAB IDE
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 * Microchip Audio Library – PIC32 Software.
 * Copyright © 2008 Microchip Technology Inc.  All rights reserved.
 * 
 * Microchip licenses the Software for your use with Microchip microcontrollers
 * and Microchip digital signal controllers pursuant to the terms of the
 * Non-Exclusive Software License Agreement accompanying this Software.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY
 * OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION,
 * ANY WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS
 * FOR A PARTICULAR PURPOSE.
 * MICROCHIP AND ITS LICENSORS ASSUME NO RESPONSIBILITY FOR THE ACCURACY,
 * RELIABILITY OR APPLICATION OF THE SOFTWARE AND DOCUMENTATION.
 * IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED
 * UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH
 * OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL,
 * SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS
 * OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY,
 * SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED
 * TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *
 *$Id: $
 ********************************************************************/


#ifndef _ETH_PHY_H_
#define _ETH_PHY_H_

#include <peripheral/eth.h>

// definitions

#ifdef _ETH		// ETHC present


typedef enum {
    // PHY flags, connection flags
    ETH_PHY_CFG_RMII = 0x01,        // check that configuration fuses is RMII
    ETH_PHY_CFG_MII = 0x00,         // check that configuration fuses is MII
    ETH_PHY_CFG_ALTERNATE = 0x02,   // check that configuration fuses is ALT
    ETH_PHY_CFG_DEFAULT = 0x00,     // check that configuration fuses is DEFAULT
    ETH_PHY_CFG_AUTO = 0x10         // use the fuses configuration to detect if you are RMII/MII and ALT/DEFAULT configuration
    // NOTE: - this option does not check the consistency btw the software call and the way the
    //         fuses are configured. If just assumes that the fuses are properly configured.
    //       - option is valid for EthPhyInit() call only!

} eEthPhyCfgFlags; // flags for EthPhyInit() call


/****************************************************************************
 * Function:        EthPhyInit
 *
 * PreCondition:    - EthInit should have been called.
 *
 * Input:           oFlags - the requested open flags
 *                  cFlags - PHY MII/RMII configuration flags
 *                  pResFlags - address to store the initialization result  
 *
 * Output:          ETH_RES_OK for success,
 *                  an error code otherwise
 *
 *
 * Side Effects:    None
 *
 * Overview:        This function initializes the PHY communication.
 *                  It tries to detect the external PHY, to read the capabilties and find a match
 *                  with the requested features.
 *                  Then it programs the PHY accordingly.
 *
 * Note:            None
 *****************************************************************************/
eEthRes EthPhyInit(eEthOpenFlags oFlags, eEthPhyCfgFlags cFlags, eEthOpenFlags* pResFlags);


/****************************************************************************
 * Function:        EthPhyGetHwConfigFlags
 *
 * PreCondition:    None.
 *
 * Input:           None  
 *
 * Output:          a eEthPhyCfgFlags value
 *
 *
 * Side Effects:    None
 *
 * Overview:        This function returns the current PHY hardware MII/RMII and ALTERNATE/DEFAULT configuration flags.
 *
 * Note:            None
 *****************************************************************************/
eEthPhyCfgFlags EthPhyGetHwConfigFlags(void);


#endif	// _ETH

#endif	// _ETH_PHY_H_

