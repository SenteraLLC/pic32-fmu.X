/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    tcpip.h

  Summary:
    

  Description:
    Microchip TCP/IP Stack Include File

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

#ifndef __TCPIP_H_
#define __TCPIP_H_

#define TCPIP_STACK_VERSION         "v5.43"     // TCP/IP stack version

// RESERVED FEATURE -- do not change from current value of 1u as this is not
// fully implemented yet.
// Defines the number of different network interfaces to support (ex: 2 for
// Wifi and Ethernet simultaneously).
#define NETWORK_INTERFACES      (1u)

/*******************************************************************
 * Memory Configuration
 *   The following section sets up the memory types for use by
 *   this application.
 *******************************************************************/
    // Represents data stored in Ethernet buffer RAM
    #define TCP_ETH_RAM 0u
    // The base address for TCP data in Ethernet RAM
    #define TCP_ETH_RAM_BASE_ADDRESS            (BASE_TCB_ADDR)
    // Represents data stored in local PIC RAM
    #define TCP_PIC_RAM 1u
    // The base address for TCP data in PIC RAM
    #define TCP_PIC_RAM_BASE_ADDRESS            ((PTR_BASE)&TCPBufferInPIC[0])
    // Represents data stored in external SPI RAM
    #define TCP_SPI_RAM 2u

/*******************************************************************
 * User Configuration
 *   Load the user-specific configuration from tcpip_config.h
 *******************************************************************/
#include "system_config.h"

/*******************************************************************
 * Configuration Rules Enforcement
 *   The following section enforces requirements for modules based
 *   on configurations selected in tcpip_config.h
 *******************************************************************/

#ifndef STACK_USE_MDD

    #if defined(STACK_USE_HTTP2_SERVER) || defined(STACK_USE_FTP_SERVER)
        #define STACK_USE_MPFS2
    #endif

#endif

    // FTP is not supported in MPFS2 or when MPFS is stored in internal program
    // memory (instead of external EEPROM).
    #if ( (!defined(MPFS_USE_EEPROM) && !defined(MPFS_USE_SPI_FLASH)) || defined(STACK_USE_MPFS2) ) && defined(STACK_USE_FTP)
        #error FTP server is not supported with HTTP2 / MPFS2, or with internal Flash memory storage
    #endif

    // When IP Gleaning is enabled, ICMP must also be enabled.
    #if defined(STACK_USE_IP_GLEANING)
        #if !defined(STACK_USE_ICMP_SERVER)
            #define STACK_USE_ICMP_SERVER
        #endif
    #endif

    // Include modules required by specific HTTP demos
    #if !defined(STACK_USE_HTTP2_SERVER)
        #undef STACK_USE_HTTP_EMAIL_DEMO
        #undef STACK_USE_HTTP_MD5_DEMO
        #undef STACK_USE_HTTP_APP_RECONFIG
    #endif
    #if defined(STACK_USE_HTTP_EMAIL_DEMO)
        #if !defined(STACK_USE_SMTP_CLIENT)
            #error HTTP E-mail Demo requires SMTP_CLIENT and HTTP2
        #endif
    #endif
    #if defined(STACK_USE_HTTP_MD5_DEMO)
        #if !defined(STACK_USE_MD5)
            #define STACK_USE_MD5
        #endif
    #endif

    // Can't do MPFS upload without POST or external memory
    #if defined(HTTP_MPFS_UPLOAD)
        #if !defined(HTTP_USE_POST) || (!defined(MPFS_USE_EEPROM) && !defined(MPFS_USE_SPI_FLASH))
            #undef HTTP_MPFS_UPLOAD
        #endif
    #endif

    // Make sure that the DNS client is enabled if services require it
    #if defined(STACK_USE_GENERIC_TCP_CLIENT_EXAMPLE) || \
        defined(STACK_USE_SNTP_CLIENT) || \
        defined(STACK_USE_DYNAMICDNS_CLIENT) || \
        defined(STACK_USE_SMTP_CLIENT) || \
        defined(STACK_USE_AUTOUPDATE_TCPCLIENT)
        #if !defined(STACK_USE_DNS)
            #define STACK_USE_DNS
        #endif
    #endif

    // Make sure that STACK_CLIENT_MODE is defined if a service
    // depends on it
    #if defined(STACK_USE_FTP_SERVER) || \
        defined(STACK_USE_DNS) || \
        defined(STACK_USE_GENERIC_TCP_CLIENT_EXAMPLE) || \
        defined(STACK_USE_TFTP_CLIENT) || \
        defined(STACK_USE_SMTP_CLIENT) || \
        defined(STACK_USE_ICMP_CLIENT) || \
        defined(STACK_USE_DYNAMICDNS_CLIENT) || \
        defined(STACK_USE_SNTP_CLIENT) || \
        defined(STACK_USE_BERKELEY_API) || \
        defined(STACK_USE_SSL_CLIENT) || \
        defined(STACK_USE_AUTO_IP)
        #if !defined(STACK_CLIENT_MODE)
            #define STACK_CLIENT_MODE
        #endif
    #endif

    // Make sure that STACK_USE_TCP is defined if a service
    // depends on it
    #if defined(STACK_USE_UART2TCP_BRIDGE) || \
        defined(STACK_USE_HTTP2_SERVER) || \
        defined(STACK_USE_FTP_SERVER) || \
        defined(STACK_USE_TELNET_SERVER) || \
        defined(STACK_USE_GENERIC_TCP_CLIENT_EXAMPLE) || \
        defined(STACK_USE_GENERIC_TCP_SERVER_EXAMPLE) || \
        defined(STACK_USE_SMTP_CLIENT) || \
        defined(STACK_USE_TCP_PERFORMANCE_TEST) || \
        defined(STACK_USE_DYNAMICDNS_CLIENT) || \
        defined(STACK_USE_BERKELEY_API) || \
        defined(STACK_USE_SSL_CLIENT) || \
        defined(STACK_USE_SSL_SERVER)
        #if !defined(STACK_USE_TCP)
            #define STACK_USE_TCP
        #endif
    #endif

    // If TCP is not enabled, clear all memory allocations
    #if !defined(STACK_USE_TCP)
        #undef TCP_ETH_RAM_SIZE
        #undef TCP_PIC_RAM_SIZE
        #undef TCP_SPI_RAM_SIZE
        #define TCP_ETH_RAM_SIZE 0u
        #define TCP_PIC_RAM_SIZE 0u
        #define TCP_SPI_RAM_SIZE 0u
    #endif

    // If PIC RAM is used to store TCP socket FIFOs and TCBs,
    // let's allocate it so the linker dynamically chooses
    // where to locate it and prevents other variables from
    // overlapping with it
    #if defined(__TCP_C_) && TCP_PIC_RAM_SIZE > 0u
        static uint8_t TCPBufferInPIC[TCP_PIC_RAM_SIZE] __attribute__((far));
    #endif

    // Make sure that STACK_USE_UDP is defined if a service
    // depends on it
    #if defined(STACK_USE_DHCP_CLIENT) || \
        defined(STACK_USE_DHCP_SERVER) || \
        defined(STACK_USE_DNS) || \
        defined(STACK_USE_NBNS) || \
        defined(STACK_USE_TFTP_CLIENT) || \
        defined(STACK_USE_ANNOUNCE) || \
        defined(STACK_USE_UDP_PERFORMANCE_TEST) || \
        defined(STACK_USE_SNTP_CLIENT) || \
        defined(STACK_USE_BERKELEY_API)
        #if !defined(STACK_USE_UDP)
            #define STACK_USE_UDP
        #endif
    #endif

    // When using SSL server, enable RSA decryption
    #if defined(STACK_USE_SSL_SERVER)
        #define STACK_USE_RSA_DECRYPT
        #define STACK_USE_SSL
    #endif

    // When using SSL client, enable RSA encryption
    #if defined(STACK_USE_SSL_CLIENT)
        #define STACK_USE_RSA_ENCRYPT
        #define STACK_USE_SSL
    #endif

    // If using SSL (either), include the rest of the support modules
    #if defined(STACK_USE_SSL)
        #define STACK_USE_ARCFOUR
        #define STACK_USE_MD5
        #define STACK_USE_SHA1
        #define STACK_USE_RANDOM
    #endif

    // When using either RSA operation, include the RSA module
    #if defined(STACK_USE_RSA_ENCRYPT) || defined(STACK_USE_RSA_DECRYPT)
        #define STACK_USE_RSA
        #define STACK_USE_BIGINT
    #endif

    // Enable the LCD if configured in the hardware profile
    #if defined(LCD_DATA_IO) || defined(LCD_DATA0_IO)
        #define USE_LCD
    #endif

    // SPI Flash MPFS images must start on a block boundary
    #if (defined(STACK_USE_MPFS2)) && \
        defined(MPFS_USE_SPI_FLASH) && ((MPFS_RESERVE_BLOCK & 0x0fff) != 0)
        #error MPFS_RESERVE_BLOCK must be a multiple of 4096 for SPI Flash storage
    #endif

    // HTTP2 requires 2 MPFS2 handles per connection, plus one spare
    #if defined(STACK_USE_HTTP2_SERVER)
        #if MAX_MPFS_HANDLES < ((MAX_HTTP_CONNECTIONS * 2) + 1)
            #error HTTP2 requires 2 MPFS2 file handles per connection, plus one additional.
        #endif
    #endif

#include "tcpip/stack_task.h"
#include "tcpip/helpers.h"
#include "tcpip/delay.h"
#include "tcpip/tick.h"
#include "tcpip/mac.h"
#include "tcpip/ip.h"
#include "tcpip/arp.h"

#if defined(STACK_USE_BIGINT)
    #include "tcpip/big_int.h"
#endif

#if defined(STACK_USE_RSA)
    #include "tcpip/rsa.h"
#endif

#if defined(STACK_USE_ARCFOUR)
    #include "tcpip/arc4.h"
#endif

#if defined(STACK_USE_AUTO_IP)
        #include "tcpip/auto_ip.h"
#endif

#if defined(STACK_USE_RANDOM)
    #include "tcpip/random.h"
#endif

#if defined(STACK_USE_MD5) || defined(STACK_USE_SHA1)
    #include "tcpip/hashes.h"
#endif

//#include "tcpip/spi_eeprom.h"
//#include "tcpip/spi_flash.h"
//#include "tcpip/spi_ram.h"

#if defined(STACK_USE_UDP)
    #include "tcpip/udp.h"
#endif

#if defined(STACK_USE_TCP)
    #include "tcpip/tcp.h"
#endif

#if defined(STACK_USE_BERKELEY_API)
    #include "tcpip/berkeley_api.h"
#endif

#if defined(USE_LCD)
    #include "tcpip/lcd_blocking.h"
#endif

#if defined(STACK_USE_UART2TCP_BRIDGE)
    #include "tcpip/uart_to_tcp_bridge.h"
#endif

#if defined(STACK_USE_UART)
    #include "tcpip/uart.h"
#endif

#if defined(STACK_USE_DHCP_CLIENT) || defined(STACK_USE_DHCP_SERVER)
    #include "tcpip/dhcp.h"
#endif

#if defined(STACK_USE_DNS) || defined(STACK_USE_DNS_SERVER)
    #include "tcpip/dns.h"
#endif

#if defined(STACK_USE_MPFS2)
    #include "tcpip/mpfs2.h"
#endif

#if defined(STACK_USE_FTP_SERVER)
    #include "tcpip/ftp.h"
#endif

#if defined(STACK_USE_HTTP2_SERVER)
    #ifdef STACK_USE_MDD
        #include "tcpip/file_system.h"
        #include "tcpip/http2_mdd.h"
    #else
        #include "tcpip/http2.h"
    #endif
#endif

#if defined(STACK_USE_ICMP_SERVER) || defined(STACK_USE_ICMP_CLIENT)
    #include "tcpip/icmp.h"
#endif

#if defined(STACK_USE_ANNOUNCE)
    #include "tcpip/announce.h"
#endif

#if defined(STACK_USE_NBNS)
    #include "tcpip/nbns.h"
#endif

#if defined(STACK_USE_DNS)
    #include "tcpip/dns.h"
#endif

#if defined(STACK_USE_DYNAMICDNS_CLIENT)
    #include "tcpip/ddns.h"
#endif

#if defined(STACK_USE_TELNET_SERVER)
    #include "tcpip/telnet.h"
#endif

#if defined(STACK_USE_SMTP_CLIENT)
    #include "tcpip/smtp.h"
#endif

#if defined(STACK_USE_TFTP_CLIENT)
    #include "tcpip/tftp.h"
#endif

#if defined(STACK_USE_REBOOT_SERVER)
    #include "tcpip/reboot.h"
#endif

#if defined(STACK_USE_SNTP_CLIENT)
    #include "tcpip/sntp.h"
#endif

#if defined(STACK_USE_UDP_PERFORMANCE_TEST)
    #include "tcpip/udp_performance_test.h"
#endif

#if defined(STACK_USE_TCP_PERFORMANCE_TEST)
    #include "tcpip/tcp_performance_test.h"
#endif

#if defined(STACK_USE_SSL)
    #include "tcpip/ssl.h"
#endif

#if defined(WF_CS_TRIS)
    #include "driver/wifi/mrf24w/src/drv_wifi_mac.h"
#endif

#endif
