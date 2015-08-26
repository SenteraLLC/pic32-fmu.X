/*********************************************************************
 *
 *     PHY external API implementation for Microchip TCP/IP Stack
 *
 *********************************************************************
 * FileName:        ETHPIC32ExtPhy.c
 * Dependencies:
 * Processor:       PIC32
 *
 * Complier:        MPLAB C32
 *                  MPLAB IDE
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright © 2009 Microchip Technology Inc.  All rights reserved.
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
 * $Id: $
 ********************************************************************/


#include "system_config.h"

// Compile only for PIC32MX with Ethernet MAC interface (must not have external ENCX24J600, ENC28J60, or MRF24WB0M hardware defined)
#if defined(__PIC32MX__) && defined(_ETH) && !defined(ENC100_INTERFACE_MODE) && !defined(ENC_CS_TRIS) && !defined(WF_CS_TRIS)

#include "tcpip/eth_pic32_ext_phy.h"


// local definitions
//
#define	PROT_802_3	0x01	// IEEE 802.3 capability
#define	MAC_COMM_CPBL_MASK	(_BMSTAT_BASE10T_HDX_MASK|_BMSTAT_BASE10T_FDX_MASK|_BMSTAT_BASE100TX_HDX_MASK|_BMSTAT_BASE100TX_FDX_MASK)
// all comm capabilities our MAC supports


// local prototypes
//
static void _PhyInitIo(void);


/****************************************************************************
 *                 interface functions
 ****************************************************************************/

/****************************************************************************
 * Function:        EthPhyGetHwConfigFlags
 *
 * PreCondition:    - EthPhyInit should have been called.
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
eEthPhyCfgFlags __attribute__((weak)) EthPhyGetHwConfigFlags(void) {
    eEthPhyCfgFlags hwFlags;
    // the way the hw is configured
    hwFlags = (DEVCFG3bits.FMIIEN != 0) ? ETH_PHY_CFG_MII : ETH_PHY_CFG_RMII;
    hwFlags |= (DEVCFG3bits.FETHIO != 0) ? ETH_PHY_CFG_DEFAULT : ETH_PHY_CFG_ALTERNATE;

    return hwFlags;
}

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
eEthRes __attribute__((weak)) EthPhyInit(eEthOpenFlags oFlags, eEthPhyCfgFlags cFlags, eEthOpenFlags* pResFlags) {
    eEthPhyCfgFlags hwFlags, swFlags;

    // the way the hw is configured
    hwFlags = EthPhyGetHwConfigFlags();

    if (cFlags & ETH_PHY_CFG_AUTO)
    {
        cFlags = hwFlags;
    }
    else
    {
        // some minimal check against the way the hw is configured
        swFlags = cFlags & (ETH_PHY_CFG_RMII | ETH_PHY_CFG_ALTERNATE);

        if ((swFlags ^ hwFlags) != 0) 
        {
            // hw-sw configuration mismatch MII/RMII, ALT/DEF config
            return ETH_RES_CFG_ERR;
        }
    }

    _PhyInitIo(); // init IO pins

    EthMIIMConfig(GetSystemClock(), ETH_PHY_MIIM_CLK);

    // now update the open flags
    // the upper layer needs to know the PHY set-up to further set-up the MAC.

    // clear the capabilities
    oFlags &= ~(ETH_OPEN_AUTO | ETH_OPEN_FDUPLEX | ETH_OPEN_HDUPLEX | ETH_OPEN_100 | ETH_OPEN_10);

    oFlags |= ETH_OPEN_100;
    oFlags |= ETH_OPEN_FDUPLEX;

    *pResFlags = oFlags; // upper layer needs to know the PHY set-up to further set-up the MAC.

    return ETH_RES_OK;
}


/****************************************************************************
 *                 local functions
 ****************************************************************************/

/****************************************************************************
 * Function:        _PhyInitIo
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Helper to properly set the Eth i/o pins to digital pins.
 *
 * Note:            Even when the Eth device is turned on the analog shared pins have to be configured.
 *****************************************************************************/
static void _PhyInitIo(void) {
    __DEVCFG3bits_t bcfg3;

    bcfg3 = DEVCFG3bits;
    if (bcfg3.FETHIO) { // default setting, both RMII and MII
        PORTSetPinsDigitalOut(_ETH_MDC_PORT, _ETH_MDC_BIT);
        PORTSetPinsDigitalIn(_ETH_MDIO_PORT, _ETH_MDIO_BIT);

        PORTSetPinsDigitalOut(_ETH_TXEN_PORT, _ETH_TXEN_BIT);
        PORTSetPinsDigitalOut(_ETH_TXD0_PORT, _ETH_TXD0_BIT);
        PORTSetPinsDigitalOut(_ETH_TXD1_PORT, _ETH_TXD1_BIT);


        PORTSetPinsDigitalIn(_ETH_RXCLK_PORT, _ETH_RXCLK_BIT);
        PORTSetPinsDigitalIn(_ETH_RXDV_PORT, _ETH_RXDV_BIT);
        PORTSetPinsDigitalIn(_ETH_RXD0_PORT, _ETH_RXD0_BIT);
        PORTSetPinsDigitalIn(_ETH_RXD1_PORT, _ETH_RXD1_BIT);
        PORTSetPinsDigitalIn(_ETH_RXERR_PORT, _ETH_RXERR_BIT);


        if (bcfg3.FMIIEN) { // just MII
            PORTSetPinsDigitalIn(_ETH_TXCLK_PORT, _ETH_TXCLK_BIT);
            PORTSetPinsDigitalOut(_ETH_TXD2_PORT, _ETH_TXD2_BIT);
            PORTSetPinsDigitalOut(_ETH_TXD3_PORT, _ETH_TXD3_BIT);
            PORTSetPinsDigitalOut(_ETH_TXERR_PORT, _ETH_TXERR_BIT);

            PORTSetPinsDigitalIn(_ETH_RXD2_PORT, _ETH_RXD2_BIT);
            PORTSetPinsDigitalIn(_ETH_RXD3_PORT, _ETH_RXD3_BIT);
            PORTSetPinsDigitalIn(_ETH_CRS_PORT, _ETH_CRS_BIT);
            PORTSetPinsDigitalIn(_ETH_COL_PORT, _ETH_COL_BIT);
        }
    } else { // alternate setting, both RMII and MII
        PORTSetPinsDigitalOut(_ETH_ALT_MDC_PORT, _ETH_ALT_MDC_BIT);
        PORTSetPinsDigitalIn(_ETH_ALT_MDIO_PORT, _ETH_ALT_MDIO_BIT);

        PORTSetPinsDigitalOut(_ETH_ALT_TXEN_PORT, _ETH_ALT_TXEN_BIT);
        PORTSetPinsDigitalOut(_ETH_ALT_TXD0_PORT, _ETH_ALT_TXD0_BIT);
        PORTSetPinsDigitalOut(_ETH_ALT_TXD1_PORT, _ETH_ALT_TXD1_BIT);


        PORTSetPinsDigitalIn(_ETH_ALT_RXCLK_PORT, _ETH_ALT_RXCLK_BIT);
        PORTSetPinsDigitalIn(_ETH_ALT_RXDV_PORT, _ETH_ALT_RXDV_BIT);
        PORTSetPinsDigitalIn(_ETH_ALT_RXD0_PORT, _ETH_ALT_RXD0_BIT);
        PORTSetPinsDigitalIn(_ETH_ALT_RXD1_PORT, _ETH_ALT_RXD1_BIT);
        PORTSetPinsDigitalIn(_ETH_ALT_RXERR_PORT, _ETH_ALT_RXERR_BIT);


        if (bcfg3.FMIIEN) { // just MII
            PORTSetPinsDigitalIn(_ETH_ALT_TXCLK_PORT, _ETH_ALT_TXCLK_BIT);
            PORTSetPinsDigitalOut(_ETH_ALT_TXD2_PORT, _ETH_ALT_TXD2_BIT);
            PORTSetPinsDigitalOut(_ETH_ALT_TXD3_PORT, _ETH_ALT_TXD3_BIT);
            PORTSetPinsDigitalOut(_ETH_ALT_TXERR_PORT, _ETH_ALT_TXERR_BIT);

            PORTSetPinsDigitalIn(_ETH_ALT_RXD2_PORT, _ETH_ALT_RXD2_BIT);
            PORTSetPinsDigitalIn(_ETH_ALT_RXD3_PORT, _ETH_ALT_RXD3_BIT);
            PORTSetPinsDigitalIn(_ETH_ALT_CRS_PORT, _ETH_ALT_CRS_BIT);
            PORTSetPinsDigitalIn(_ETH_ALT_COL_PORT, _ETH_ALT_COL_BIT);
        }
    }
}

#endif	// defined(__PIC32MX__) && defined(_ETH)	// ETHC present

