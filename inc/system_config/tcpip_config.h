/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    tcpip_config.h

  Summary:
    

  Description:
    Microchip TCP/IP Stack Demo Application Configuration Header

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

#ifndef __TCPIP_CONFIG_H_
#define __TCPIP_CONFIG_H_

#include <stdint.h>

/*******************************************************************************
 *    Application Options
 *******************************************************************************/
/* Application Level Module Selection
 *   Uncomment or comment the following lines to enable or
 *   disabled the following high-level application modules.
 *
 * If certain compilations are enabled (eg STACK_USE_GENERIC_TCP_CLIENT_EXAMPLE),
 * check whether the files (eg generic_tcp_client.c) are located in folder (eg \apps\tcpip\wifi_console\firmware\src).
 * You may need to copy such files from the wifi_demo_app folder.
 *
 */

//#define STACK_USE_UART                        // Application demo using UART for IP address display and stack configuration
//#define STACK_USE_UART2TCP_BRIDGE             // UART to TCP Bridge application example
//#define STACK_USE_IP_GLEANING                 
#define STACK_USE_ICMP_SERVER                   // Ping query and response capability
//#define STACK_USE_ICMP_CLIENT                 // Ping transmission capability
#define STACK_USE_HTTP2_SERVER                  // New HTTP server with POST, Cookies, Authentication, etc.
//#define STACK_USE_SSL_SERVER                  // SSL server socket support (Requires SW300052)
//#define STACK_USE_SSL_CLIENT                  // SSL client socket support (Requires SW300052)
//#define STACK_USE_AUTO_IP                     // Dynamic link-layer IP address automatic configuration protocol
//#define STACK_USE_DHCP_CLIENT                 // Dynamic Host Configuration Protocol client for obtaining IP address and other parameters
//#define STACK_USE_DHCP_SERVER                 // Single host DHCP server
//#define STACK_USE_FTP_SERVER                  // File Transfer Protocol (old)
//#define STACK_USE_SMTP_CLIENT                 // Simple Mail Transfer Protocol for sending email
//#define STACK_USE_TFTP_CLIENT                 // Trivial File Transfer Protocol client
//#define STACK_USE_GENERIC_TCP_CLIENT_EXAMPLE  // HTTP Client example in generic_tcp_client.c
//#define STACK_USE_AUTOUPDATE_TCPCLIENT
//#define STACK_USE_AUTOUPDATE_UART
//#define STACK_USE_FTP_CLIENT
//#define STACK_USE_CERTIFICATE_DEBUG
//#define STACK_USE_GENERIC_TCP_SERVER_EXAMPLE  // ToUpper server example in generic_tcp_server.c
//#define STACK_USE_TELNET_SERVER               // Telnet server
//#define STACK_USE_ANNOUNCE                    // Microchip Embedded Ethernet Device Discoverer server/client
//#define STACK_USE_DNS                         // Domain Name Service Client for resolving hostname strings to IP addresses
//#define STACK_USE_DNS_SERVER                  // Domain Name Service Server for redirection to the local device
//#define STACK_USE_NBNS                        // NetBIOS Name Service Server for repsonding to NBNS hostname broadcast queries
//#define STACK_USE_REBOOT_SERVER               // Module for resetting this PIC remotely.  Primarily useful for a Bootloader.
//#define STACK_USE_SNTP_CLIENT                 // Simple Network Time Protocol for obtaining current date/time from Internet
//#define STACK_USE_UDP_PERFORMANCE_TEST        // Module for testing UDP TX performance characteristics.  NOTE: Enabling this will cause a huge amount of UDP broadcast packets to flood your network on the discard port.  Use care when enabling this on production networks, especially with VPNs (could tunnel broadcast traffic across a limited bandwidth connection).
//#define STACK_USE_TCP_PERFORMANCE_TEST        // Module for testing TCP TX performance characteristics
//#define STACK_USE_DYNAMICDNS_CLIENT           // Dynamic DNS client updater module
//#define STACK_USE_BERKELEY_API                // Berekely Sockets APIs are available
//#define STACK_USE_ZEROCONF_LINK_LOCAL         // Zeroconf IPv4 Link-Local Addressing
//#define STACK_USE_ZEROCONF_MDNS_SD            // Zeroconf mDNS and mDNS service discovery

// =======================================================================
//   Data Storage Options
// =======================================================================

/* MPFS Configuration
 *   MPFS is automatically included when required for other
 *   applications.  If your custom application requires it
 *   otherwise, uncomment the appropriate selection.
 */
//#define STACK_USE_MPFS2

/* MPFS Storage Location
 *   If html pages are stored in internal program memory,
 *   comment both MPFS_USE_EEPROM and MPFS_USE_SPI_FLASH, then
 *   include an MPFS image (.c or .s file) in the project.
 *   If html pages are stored in external memory, uncomment the
 *   appropriate definition.
 *
 *   Supported serial flash parts include the SST25VFxxxB series.
 */
//#define MPFS_USE_EEPROM
//#define MPFS_USE_SPI_FLASH

/* EEPROM Addressing Selection
 *   If using the 1Mbit EEPROM, uncomment this line
 */
//#define USE_EEPROM_25LC1024

/* EEPROM Reserved Area
 *   Number of EEPROM bytes to be reserved before MPFS storage starts.
 *   These bytes host application configurations such as IP Address,
 *   MAC Address, and any other required variables.
 *
 *   For MPFS Classic, this setting must match the Reserved setting
 *   on the Advanced Settings page of the MPFS2 Utility.
 */
#define MPFS_RESERVE_BLOCK              (205ul)

/* MPFS File Handles
 *   Maximum number of simultaneously open MPFS2 files.
 *   For MPFS Classic, this has no effect.
 */
#define MAX_MPFS_HANDLES                (7ul)

// =======================================================================
//   Network Addressing Options
// =======================================================================

/* Default Network Configuration
 *   These settings are only used if data is not found in EEPROM.
 *   To clear EEPROM, hold BUTTON0, reset the board, and continue
 *   holding until the LEDs flash.  Release, and reset again.
 */
#define MY_DEFAULT_HOST_NAME            "PIC32-FMU"

#define MY_DEFAULT_MAC_BYTE1            (0x00)  // Use the default of 00-04-A3-00-00-00
#define MY_DEFAULT_MAC_BYTE2            (0x04)  // if using an ENCX24J600, MRF24W, or
#define MY_DEFAULT_MAC_BYTE3            (0xA3)  // PIC32MX6XX/7XX internal Ethernet
#define MY_DEFAULT_MAC_BYTE4            (0x00)  // controller and wish to use the
#define MY_DEFAULT_MAC_BYTE5            (0x00)  // internal factory programmed MAC
#define MY_DEFAULT_MAC_BYTE6            (0x00)  // address instead.

#define MY_DEFAULT_IP_ADDR_BYTE1        (192ul)
#define MY_DEFAULT_IP_ADDR_BYTE2        (168ul)
#define MY_DEFAULT_IP_ADDR_BYTE3        (143ul)
#define MY_DEFAULT_IP_ADDR_BYTE4        (130ul)

#define MY_DEFAULT_MASK_BYTE1           (255ul)
#define MY_DEFAULT_MASK_BYTE2           (255ul)
#define MY_DEFAULT_MASK_BYTE3           (255ul)
#define MY_DEFAULT_MASK_BYTE4           (0ul)

#define MY_DEFAULT_GATE_BYTE1           (169ul)
#define MY_DEFAULT_GATE_BYTE2           (254ul)
#define MY_DEFAULT_GATE_BYTE3           (1ul)
#define MY_DEFAULT_GATE_BYTE4           (1ul)

#define MY_DEFAULT_PRIMARY_DNS_BYTE1    (169ul)
#define MY_DEFAULT_PRIMARY_DNS_BYTE2    (254ul)
#define MY_DEFAULT_PRIMARY_DNS_BYTE3    (1ul)
#define MY_DEFAULT_PRIMARY_DNS_BYTE4    (1ul)

#define MY_DEFAULT_SECONDARY_DNS_BYTE1  (0ul)
#define MY_DEFAULT_SECONDARY_DNS_BYTE2  (0ul)
#define MY_DEFAULT_SECONDARY_DNS_BYTE3  (0ul)
#define MY_DEFAULT_SECONDARY_DNS_BYTE4  (0ul)

// =======================================================================
//   PIC32MX7XX/6XX MAC Layer Options
//   If not using a PIC32MX7XX/6XX device, ignore this section.
// =======================================================================
#define ETH_CFG_LINK            0       // set to 1 if you need to config the link to specific following parameters
                                        // otherwise the default connection will be attempted
                                        // depending on the selected PHY
    #define ETH_CFG_AUTO        1       // use auto negotiation
    #define ETH_CFG_10          1       // use/advertise 10 Mbps capability
    #define ETH_CFG_100         1       // use/advertise 100 Mbps capability
    #define ETH_CFG_HDUPLEX     1       // use/advertise half duplex capability
    #define ETH_CFG_FDUPLEX     1       // use/advertise full duplex capability
    #define ETH_CFG_AUTO_MDIX   1       // use/advertise auto MDIX capability
    #define ETH_CFG_SWAP_MDIX   1       // use swapped MDIX. else normal MDIX

#define EMAC_TX_DESCRIPTORS     2       // number of the TX descriptors to be created
#define EMAC_RX_DESCRIPTORS     8       // number of the RX descriptors and RX buffers to be created

#define EMAC_RX_BUFF_SIZE       1536    // size of a RX buffer. should be multiple of 16
                                        // this is the size of all receive buffers processed by the ETHC
                                        // The size should be enough to accomodate any network received packet
                                        // If the packets are larger, they will have to take multiple RX buffers
                                        // The current implementation does not handle this situation right now and the packet is discarded.

// =======================================================================
//   Transport Layer Options
// =======================================================================

/* Transport Layer Configuration
 *   The following low level modules are automatically enabled
 *   based on module selections above.  If your custom module
 *   requires them otherwise, enable them here.
 */
#define STACK_USE_TCP
#define STACK_USE_UDP

/* Client Mode Configuration
 *   Uncomment following line if this stack will be used in CLIENT
 *   mode.  In CLIENT mode, some functions specific to client operation
 *   are enabled.
 */
#define STACK_CLIENT_MODE

/* TCP Socket Memory Allocation
 *   TCP needs memory to buffer incoming and outgoing data.  The
 *   amount and medium of storage can be allocated on a per-socket
 *   basis using the example below as a guide.
 */
    // Allocate how much total RAM (in bytes) you want to allocate
    // for use by your TCP TCBs, RX FIFOs, and TX FIFOs.
    #define TCP_ETH_RAM_SIZE                    (0ul)
    #define TCP_PIC_RAM_SIZE                    (10240ul)
    #define TCP_SPI_RAM_SIZE                    (0ul)
    #define TCP_SPI_RAM_BASE_ADDRESS            (0x00)

    // Define names of socket types
    #define TCP_SOCKET_TYPES
        #define TCP_PURPOSE_GENERIC_TCP_CLIENT 0
        #define TCP_PURPOSE_GENERIC_TCP_SERVER 1
        #define TCP_PURPOSE_TELNET 2
        #define TCP_PURPOSE_FTP_COMMAND 3
        #define TCP_PURPOSE_FTP_DATA 4
        #define TCP_PURPOSE_TCP_PERFORMANCE_TX 5
        #define TCP_PURPOSE_TCP_PERFORMANCE_RX 6
        #define TCP_PURPOSE_UART_2_TCP_BRIDGE 7
        #define TCP_PURPOSE_HTTP_SERVER 8
        #define TCP_PURPOSE_DEFAULT 9
        #define TCP_PURPOSE_BERKELEY_SERVER 10
        #define TCP_PURPOSE_BERKELEY_CLIENT 11
        #define TCP_PURPOSE_AUTOUPDATE_TCP_CLIENT 12
        #define TCP_PURPOSE_CLOUD_TCP_CLIENT 13
    #define END_OF_TCP_SOCKET_TYPES

    #if defined(__TCP_C_)
        // Define what types of sockets are needed, how many of
        // each to include, where their TCB, TX FIFO, and RX FIFO
        // should be stored, and how big the RX and TX FIFOs should
        // be.  Making this initializer bigger or smaller defines
        // how many total TCP sockets are available.
        //
        // Each socket requires up to 56 bytes of PIC RAM and
        // 48+(TX FIFO size)+(RX FIFO size) bytes of TCP_*_RAM each.
        //
        // Note: The RX FIFO must be at least 1 byte in order to
        // receive SYN and FIN messages required by TCP.  The TX
        // FIFO can be zero if desired.
        #define TCP_CONFIGURATION
        const struct
        {
            uint8_t vSocketPurpose;
            uint8_t vMemoryMedium;
            uint16_t wTXBufferSize;
            uint16_t wRXBufferSize;
        } TCPSocketInitializer[] =
        {
          // If you want to enable STACK_USE_GENERIC_TCP_CLIENT_EXAMPLE,
          // STACK_USE_GENERIC_TCP_SERVER_EXAMPLE and STACK_USE_IPERF at
          // the same time, you might need to configure these sockets to be smaller
          #if defined(STACK_USE_GENERIC_TCP_CLIENT_EXAMPLE)
            {TCP_PURPOSE_GENERIC_TCP_CLIENT, TCP_PIC_RAM, 1024, 100},
          #endif

          #if defined(STACK_USE_GENERIC_TCP_SERVER_EXAMPLE)
            {TCP_PURPOSE_GENERIC_TCP_SERVER, TCP_PIC_RAM, 20, 4096},
          #endif

          #if defined(STACK_USE_IPERF) // These sockets are also used in Iperf
            {TCP_PURPOSE_GENERIC_TCP_CLIENT, TCP_PIC_RAM, 1024, 100},
            {TCP_PURPOSE_GENERIC_TCP_SERVER, TCP_PIC_RAM, 20, 4096},
          #endif

          #ifdef STACK_USE_AUTOUPDATE_TCPCLIENT
            {TCP_PURPOSE_AUTOUPDATE_TCP_CLIENT, TCP_PIC_RAM, 256/*4026*/, 2048},
          #endif

          #ifdef STACK_USE_TELNET_SERVER
            {TCP_PURPOSE_TELNET, TCP_PIC_RAM, 200, 150},
            //{TCP_PURPOSE_TELNET, TCP_PIC_RAM, 200, 150},
            //{TCP_PURPOSE_TELNET, TCP_PIC_RAM, 200, 150},
          #endif

          #ifdef STACK_USE_FTP_SERVER
            {TCP_PURPOSE_FTP_COMMAND, TCP_PIC_RAM, 100, 40},
          #endif

          #ifdef STACK_USE_FTP_CLIENT
            {TCP_PURPOSE_FTP_DATA, TCP_PIC_RAM, 0, 128},
          #endif

          #ifdef STACK_USE_TCP_PERFORMANCE_TEST
            {TCP_PURPOSE_TCP_PERFORMANCE_TX, TCP_PIC_RAM, 200, 1},
            //{TCP_PURPOSE_TCP_PERFORMANCE_RX, TCP_PIC_RAM, 40, 1500}, // For measuring TCP throughput, was replaced by Iperf, so don't need anymore
          #endif

          #ifdef STACK_USE_UART2TCP_BRIDGE
            {TCP_PURPOSE_UART_2_TCP_BRIDGE, TCP_PIC_RAM, 256, 256},
          #endif

          #ifdef STACK_USE_HTTP2_SERVER
            #if defined(STACK_USE_SSL_SERVER) || defined(STACK_USE_SSL_CLIENT)
              {TCP_PURPOSE_HTTP_SERVER, TCP_PIC_RAM, 800, 800},
              {TCP_PURPOSE_HTTP_SERVER, TCP_PIC_RAM, 800, 800},
            #else
              {TCP_PURPOSE_HTTP_SERVER, TCP_PIC_RAM, 1500, 1500},
              {TCP_PURPOSE_HTTP_SERVER, TCP_PIC_RAM, 1500, 1500},
            #endif
          #endif

          #ifdef STACK_USE_SMTP_CLIENT
            {TCP_PURPOSE_DEFAULT, TCP_PIC_RAM, 1000, 1000},
          #endif

          #ifdef STACK_USE_BERKELEY_API
            {TCP_PURPOSE_BERKELEY_SERVER, TCP_PIC_RAM, 25, 20},
            {TCP_PURPOSE_BERKELEY_SERVER, TCP_PIC_RAM, 25, 20},
            {TCP_PURPOSE_BERKELEY_SERVER, TCP_PIC_RAM, 25, 20},
            {TCP_PURPOSE_BERKELEY_CLIENT, TCP_PIC_RAM, 125, 100},
          #endif
        };
        #define END_OF_TCP_CONFIGURATION
    #endif

/* UDP Socket Configuration
 *   Define the maximum number of available UDP Sockets, and whether
 *   or not to include a checksum on packets being transmitted.
 */
#define MAX_UDP_SOCKETS         (10u)
#define UDP_USE_TX_CHECKSUM     // This slows UDP TX performance by nearly 50%, 
                                // except when using the ENCX24J600 or 
                                // PIC32MX6XX/7XX, which have a super fast DMA 
                                // and incurs virtually no speed penalty.

/* Berkeley API Sockets Configuration
 *   Note that each Berkeley socket internally uses one TCP or UDP socket
 *   defined by MAX_UDP_SOCKETS and the TCPSocketInitializer[] array.
 *   Therefore, this number MUST be less than or equal to MAX_UDP_SOCKETS + the
 *   number of TCP sockets defined by the TCPSocketInitializer[] array
 *   (i.e. sizeof(TCPSocketInitializer)/sizeof(TCPSocketInitializer[0])).
 *   This define has no effect if STACK_USE_BERKELEY_API is not defined and
 *   Berkeley Sockets are disabled.  Set this value as low as your application
 *   requires to avoid wasting RAM.
 */
#define BSD_SOCKET_COUNT (5u)

// =======================================================================
//   Application-Specific Options
// =======================================================================

// -- HTTP2 Server options -----------------------------------------------

    // Maximum numbers of simultaneous HTTP connections allowed.
    // Each connection consumes 2 bytes of RAM and a TCP socket
    #define MAX_HTTP_CONNECTIONS    (2u)

    // Optional setting to use PIC RAM instead of Ethernet/Wi-Fi RAM for
    // storing HTTP Connection Context variables (HTTP_CONN structure for each
    // HTTP connection).  Undefining this macro results in the Ethernet/Wi-Fi
    // RAM being used (minimum PIC RAM usage, lower performance).  Defining
    // this macro results in PIC RAM getting used (higher performance, but uses
    // PIC RAM).  This option should not be enabled on PIC18 devices.  The
    // performance increase of having this option defined is only apparent when
    // the HTTP server is servicing multiple connections simultaneously.
    //#define HTTP_SAVE_CONTEXT_IN_PIC_RAM

    // Indicate what file to serve when no specific one is requested
    #define HTTP_DEFAULT_FILE       "index.htm"
    #define HTTPS_DEFAULT_FILE      "index.htm"
    #define HTTP_DEFAULT_LEN        (10u)       // For buffer overrun protection.
                                                // Set to longest length of above two strings.

    // Configure MPFS over HTTP updating
    // Comment this line to disable updating via HTTP
    #define HTTP_MPFS_UPLOAD        "mpfsupload"
    //#define HTTP_MPFS_UPLOAD_REQUIRES_AUTH    // Require password for MPFS uploads
    // Certain firewall and router combinations cause the MPFS2 Utility to fail
    // when uploading.  If this happens, comment out this definition.

    // Define which HTTP modules to use
    // If not using a specific module, comment it to save resources
    #define HTTP_USE_POST                   // Enable POST support
    //#define HTTP_USE_COOKIES              // Enable cookie support
    //#define HTTP_USE_AUTHENTICATION       // Enable basic authentication support

    //#define HTTP_NO_AUTH_WITHOUT_SSL      // Uncomment to require SSL before requesting a password

    // Define the listening port for the HTTP server
    #define HTTP_PORT               (80u)

    // Define the listening port for the HTTPS server (if STACK_USE_SSL_SERVER is enabled)
    #define HTTPS_PORT              (443u)

    // Define the maximum data length for reading cookie and GET/POST arguments (bytes)
    #define HTTP_MAX_DATA_LEN       (1024u)

    // Define the minimum number of bytes free in the TX FIFO before executing callbacks
    #define HTTP_MIN_CALLBACK_FREE  (16u)

    //#define STACK_USE_HTTP_APP_RECONFIG       // Use the AppConfig web page in the Demo App (~2.5kb ROM, ~0b RAM)
    //#define STACK_USE_HTTP_MD5_DEMO           // Use the MD5 Demo web page (~5kb ROM, ~160b RAM)
    //#define STACK_USE_HTTP_EMAIL_DEMO         // Use the e-mail demo web page

// -- SSL Options --------------------------------------------------------

    #define MAX_SSL_CONNECTIONS     (2ul)   // Maximum connections via SSL
    #define MAX_SSL_SESSIONS        (2ul)   // Max # of cached SSL sessions
    #define MAX_SSL_BUFFERS         (4ul)   // Max # of SSL buffers (2 per socket)
    #define MAX_SSL_HASHES          (5ul)   // Max # of SSL hashes  (2 per, plus 1 to avoid deadlock)

    // Bits in SSL RSA key.  This parameter is used for SSL sever
    // connections only.
    #define SSL_RSA_KEY_SIZE        (2048ul)

// -- Telnet Options -----------------------------------------------------

    // Number of simultaneously allowed Telnet sessions.  Note that you
    // must have an equal number of TCP_PURPOSE_TELNET type TCP sockets
    // declared in the TCPSocketInitializer[] array above for multiple
    // connections to work.  If fewer sockets are available than this
    // definition, then the the lesser of the two quantities will be the
    // actual limit.
    #define MAX_TELNET_CONNECTIONS  (1u)

    // Default local listening port for the Telnet server.  Port 23 is the
    // protocol default.
    #define TELNET_PORT             23

    // Default local listening port for the Telnet server when SSL secured.
    // Port 992 is the telnets protocol default.
    #define TELNETS_PORT            992

    // Force all connecting clients to be SSL secured and connected via
    // TELNETS_PORT.  Connections on port TELNET_PORT will be ignored.  If
    // STACK_USE_SSL_SERVER is undefined, this entire setting is ignored
    // (server will accept unsecured connections on TELNET_PORT and won't even
    // listen on TELNETS_PORT).
    //#define TELNET_REJECT_UNSECURED

    // Default username and password required to login to the Telnet server.
    #define TELNET_USERNAME         "admin"
    #define TELNET_PASSWORD         "microchip"

#endif /* __TCPIP_CONFIG_H_ */
