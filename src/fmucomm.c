////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief 
////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

#include <sys/attribs.h>
#include "fmucomm.h"
#include "tcpip/tcpip.h"
#include "util.h"

// *****************************************************************************
// ************************** Defines ******************************************
// *****************************************************************************

// The number of bytes for the packet wrapping (header and footer) information.
// This includes the fields:
//  - Header            3 bytes
//  - Packet Type       1 byte
//  - Payload Length    2 bytes
//  - CRC               2 bytes
//
#define FMUCOMM_WRAP_SIZE   8

typedef struct
{
    uint8_t           header0;
    uint8_t           header1;
    uint8_t           header2;
    uint8_t           type;
    uint16_t          length_lb;
    uint16_t          length_ub;
    FMUCOMM_PKT_WRAP* pkt_wrap_p;
    uint8_t*          pkt_pl_p;
    
} FMUCOMM_RX_CFG;

typedef struct
{
    uint8_t header0;
    uint8_t header1;
    uint8_t header2;
    uint8_t type;
    
} FMUCOMM_TX_CFG;


// *****************************************************************************
// ************************** Definitions **************************************
// *****************************************************************************

static FMUCOMM_HOST_HEARTBEAT_PKT   FMUCommHostHeartbeatPkt;
static FMUCOMM_CTRL_SURFACE_CMD_PKT FMUCommCtrlSurfaceCmdPkt;
static FMUCOMM_HOST_GPS_CMD_PKT     FMUCommHostGPSCmdPkt;
static FMUCOMM_HOST_EXCEPTION_PKT   FMUCommHostExeptionPkt;

static FMUCOMM_RX_CFG fmucomm_rx_cfg [ ] =
{ 
    { 'U', 'M', 'N', 0x00, 16,   16, &FMUCommHostHeartbeatPkt.wrap,  &FMUCommHostHeartbeatPkt.pl_u8[ 0 ]  },  // FMUCOMM_TYPE_HOST_HEARTBEAT     
    { 'U', 'M', 'N', 0x01,  3,   60, &FMUCommCtrlSurfaceCmdPkt.wrap, &FMUCommCtrlSurfaceCmdPkt.pl_u8[ 0 ] },  // FMUCOMM_TYPE_CTRL_SURFACE_CMD    
    { 'U', 'M', 'N', 0x02,  1, 1024, &FMUCommHostGPSCmdPkt.wrap,     &FMUCommHostGPSCmdPkt.pl_u8[ 0 ]     },  // FMUCOMM_TYPE_GPS_CMD   
    { 'U', 'M', 'N', 0x7F,  1, 1024, &FMUCommHostExeptionPkt.wrap,   &FMUCommHostExeptionPkt.pl_u8[ 0 ]   },  // FMUCOMM_TYPE_HOST_EXCEPTION 
};
    
static FMUCOMM_TX_CFG fmucomm_tx_cfg [ ] =
{
    { 'U', 'M', 'N', 0x80 },  // FMUCOMM_TYPE_FMU_HEARTBEAT     
    { 'U', 'M', 'N', 0x81 },  // FMUCOMM_TYPE_IMU_DATA          
    { 'U', 'M', 'N', 0x82 },  // FMUCOMM_TYPE_GPS_DATA          
    { 'U', 'M', 'N', 0x83 },  // FMUCOMM_TYPE_AIR_DATA          
    { 'U', 'M', 'N', 0x84 },  // FMUCOMM_TYPE_CTRL_SURFACE_DATA 
    { 'U', 'M', 'N', 0xFF },  // FMUCOMM_TYPE_FMU_EXCEPTION     
};

// Client IP Address (192.168.143.11) in big-endian.
static const uint32_t   clientIPAddr = 0x0B8FA8C0;
static const UDP_PORT   clientPort   = 55455;
static       UDP_SOCKET clientSocket = INVALID_UDP_SOCKET;

static const UDP_PORT   servertPort  = 55455;
static       UDP_SOCKET serverSocket = INVALID_UDP_SOCKET;

static bool fmucomm_rx_valid[ FMUCOMM_RX_TYPE_MAX ] =
{
    false,
    false,
    false,
    false,
};

// *****************************************************************************
// ************************** Function Prototypes ******************************
// *****************************************************************************

void FMUCommRead( void );

// *****************************************************************************
// ************************** Global Functions *********************************
// *****************************************************************************

void FMUCommTask()
{
    static enum
    {
        SM_INIT,
        SM_PROCESS,
    } FMUCommTaskState = SM_INIT;

    switch (FMUCommTaskState)
    {
        case SM_INIT:
        {
            // Open a client socket for transmission of UDP data.
            clientSocket = UDPOpenEx( clientIPAddr,
                                      UDP_OPEN_IP_ADDRESS,
                                      0,
                                      clientPort );
            
            // Open a server socket for reception of UDP data.
            serverSocket = UDPOpenEx( 0, 
                                      UDP_OPEN_SERVER,
                                      servertPort,
                                      0 );
            
            
            // Sockets opened successfully ?
            if( ( clientSocket != INVALID_UDP_SOCKET ) &&
                ( serverSocket != INVALID_UDP_SOCKET ) )
            {
                FMUCommTaskState++;
            }
            
            break;
        }
        case SM_PROCESS:
        {
            // Clear validity identifiers for all received messages.
            //
            // Note: validity flags are only set for one iteration of the 
            // FMUCommTask.  Therefore, modules which use the received data
            // must fully process the message or copy the message during
            // an execution.
            //
            memset( &fmucomm_rx_valid[ 0 ], false, sizeof( fmucomm_rx_valid ) );
            
            // Read UDP data and decode.
            FMUCommRead();
        }
    }
}

bool FMUCommSet( FMUCOMM_TX_TYPE_E pktType, uint8_t* pl_p, uint16_t plLen )
{
    bool            setSuccess = false;
    FMUCOMM_TX_CFG* pktCfg_p;
    uint16_t        calcCRC;
    
    // FMU socket is open for communication ?
    if( UDPIsOpened( clientSocket ) == true )
    {
        // Socket has space available ?
        if( UDPIsPutReady( clientSocket ) >= ( plLen + FMUCOMM_WRAP_SIZE ) )
        {
            // Update returned annunciation to identify success.
            setSuccess = true;
            
            // Index the selected packets configuration once - improves
            // processing speed and reduces line length.
            pktCfg_p = &fmucomm_tx_cfg[ pktType ];
            
            // Calculate CRC of the packet - includes the type, length, and
            // payload.
            //
            // Note: the starting CRC value is '0', and the previous CRC value
            // is used for the subsequent calculations.  This piece-wise
            // calculation is necessary because packet fields are in non-
            // contiguous memory locations.
            //
            calcCRC = utilCRC16( &pktCfg_p->type,    sizeof( pktCfg_p->type ), 0       );
            calcCRC = utilCRC16( &plLen,             sizeof( plLen ),          calcCRC );
            calcCRC = utilCRC16( (const void*) pl_p, plLen,                    calcCRC );
            
            // Queue the message for transmission. Content transmission order
            // is:
            //  - Header 0
            //  - Header 1
            //  - Header 2
            //  - Payload Type
            //  - Length (LSB)
            //  - Length (MSB)
            //  - Payload
            //  - CRC (LSB)
            //  - CRC (MSB)
            //
            // Note: Return value of UDP 'put' function is not checked since
            // available buffer space has already been verified.
            //
            (void) UDPPut( pktCfg_p->header0 );
            (void) UDPPut( pktCfg_p->header1 );
            (void) UDPPut( pktCfg_p->header2 );
            (void) UDPPut( pktCfg_p->type    );
            (void) UDPPut( (uint8_t) ( plLen >> 0 ) );
            (void) UDPPut( (uint8_t) ( plLen >> 8 ) );
            
            (void) UDPPutArray( pl_p, plLen );
            
            (void) UDPPut( (uint8_t) ( calcCRC >> 0 ) );
            (void) UDPPut( (uint8_t) ( calcCRC >> 8 ) );
            
            // Mark the UDP packet for transmission.
            UDPFlush();
        }
    }
    
    return setSuccess;
}

bool FMUCommGet( FMUCOMM_RX_TYPE_E pktType, void** pkt_wrap_pp, void** pkt_pl_pp )
{
    // Valid packed data exists ?
    if( fmucomm_rx_valid[ pktType ] == true )
    {
        // Return an alias to the packet wrapper information and packet payload.
        *pkt_wrap_pp = fmucomm_rx_cfg[ pktType ].pkt_wrap_p;
        *pkt_pl_pp   = fmucomm_rx_cfg[ pktType ].pkt_pl_p;
    }
    
    return fmucomm_rx_valid[ pktType ];
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************

// Read and decode UDP data
void FMUCommRead( void )
{
    // State for UPD packet reception.
    static enum
    {
        UDP_SM_HEADER_0,
        UDP_SM_HEADER_1,
        UDP_SM_HEADER_2,
                
        UDP_SM_TYPE,
        
        UDP_SM_LENGTH_LSB,
        UDP_SM_LENGTH_MSB,
        
        UDP_SM_PAYLOAD,
        
        UDP_SM_CRC_LSB,
        UDP_SM_CRC_MSB,
                 
    } UDPState = UDP_SM_HEADER_0;
    
    static FMUCOMM_RX_TYPE_E rx_cfg_idx;
    
    static FMUCOMM_PKT_WRAP pkt_wrap;
    
    uint16_t udp_byte_cnt;
    uint16_t udp_byte_idx;
    
    // FMU socket is open for reception ?
    if( UDPIsOpened( serverSocket ) == true )
    {
        udp_byte_cnt = UDPIsGetReady( serverSocket );
        
        // Socket has received a segment ?
        if( udp_byte_cnt != 0 )
        {
            // Increment through the received UDP bytes.
            for( udp_byte_idx = 0;
                 udp_byte_idx < udp_byte_cnt;
                 udp_byte_idx++ )
            {
                //
                // Note: Return value of function 'UDPGet' is not checked since 
                // number of available bytes already identified.
                // 
                switch( UDPState )
                {
                    case UDP_SM_HEADER_0:
                    {
                        (void) UDPGet( &pkt_wrap.header0 );
                        
                        // Byte matches Header_0 required value ?
                        if( pkt_wrap.header0 == 'U' )
                        {
                            // Header_0 received, move to next state.
                            UDPState++;
                        }
                        
                        break;
                    }
                    case UDP_SM_HEADER_1:
                    {
                        (void) UDPGet( &pkt_wrap.header1 );
                        
                        // Byte matches Header_1 required value ?
                        if( pkt_wrap.header1 == 'M' )
                        {
                            // Header_1 received, move to next state.
                            UDPState++;
                        }
                        else
                        {
                            // Packet format error, return to initial state.
                            UDPState = UDP_SM_HEADER_0;
                        }
                        
                        break;
                    }
                    case UDP_SM_HEADER_2:
                    {
                        (void) UDPGet( &pkt_wrap.header2 );
                        
                        // Byte matches Header_2 required value ?
                        if( pkt_wrap.header2 == 'N' )
                        {
                            // Header_2 received, move to next state.
                            UDPState++;
                        }
                        else
                        {
                            // Packet format error, return to initial state.
                            UDPState = UDP_SM_HEADER_0;
                        }
                        
                        break;
                    }
                    case UDP_SM_TYPE:
                    {
                        bool type_valid = false;
                         
                        (void) UDPGet( &pkt_wrap.type );
                        
                        for( rx_cfg_idx = 0;
                             rx_cfg_idx < FMUCOMM_RX_TYPE_MAX;
                             rx_cfg_idx++ )
                        {
                            // Received UDP byte matches expected value ?
                            if( pkt_wrap.type == fmucomm_rx_cfg[ rx_cfg_idx ].type )
                            {
                                type_valid = true;

                                UDPState++;
                                
                                // Exit for-loop so index of configuration
                                // data is maintained.
                                break;
                            }
                        }
                        
                        // Receive message type was not valid ?
                        if( type_valid == false )
                        {
                            // Packet format error, return to initial state.
                            UDPState = UDP_SM_HEADER_0;
                        }
                        
                        break;
                    }
                    case UDP_SM_LENGTH_LSB:
                    {
                        (void) UDPGet( &pkt_wrap.length_lsb );
                        
                        UDPState++;
                        
                        break;
                    }
                    case UDP_SM_LENGTH_MSB:
                    {
                        (void) UDPGet( &pkt_wrap.length_msb );
                        
                        // Received length matches required value ?
                        if( ( pkt_wrap.length >= fmucomm_rx_cfg[ rx_cfg_idx ].length_lb ) &&
                            ( pkt_wrap.length <= fmucomm_rx_cfg[ rx_cfg_idx ].length_ub ) )
                        {
                            UDPState++;
                        }
                        else
                        {
                            // Packet format error, return to initial state.
                            UDPState = UDP_SM_HEADER_0;
                        }
                        
                        break;
                    }
                    case UDP_SM_PAYLOAD:
                    {
                        static uint16_t pl_idx = 0;
                        
                        // Read the next UPD byte into the applicable packet's 
                        // payload allocation.
                        (void) UDPGet( &fmucomm_rx_cfg[ rx_cfg_idx ].pkt_pl_p[ pl_idx ] );
                        
                        pl_idx++;
                        
                        // Entire payload has been received ?
                        if( pl_idx >= pkt_wrap.length )
                        {
                            // Re-initialize payload index for next
                            // evaluation.
                            pl_idx = 0;
                            
                            UDPState++;
                        }
                        
                        break;
                    }
                    case UDP_SM_CRC_LSB:
                    {
                        (void) UDPGet( &pkt_wrap.crc_lsb );
                        
                        UDPState++;
                        
                        break;
                    }
                    case UDP_SM_CRC_MSB:
                    {
                        uint16_t calcCRC;
                        
                        (void) UDPGet( &pkt_wrap.crc_msb );
                        
                        // Calculate CRC of the packet - includes the type, length, and
                        // payload.
                        //
                        // Note: the starting CRC value is '0', and the previous CRC value
                        // is used for the subsequent calculations.  This piece-wise
                        // calculation is necessary because packet fields are in non-
                        // contiguous memory locations.
                        //
                        calcCRC = utilCRC16( &pkt_wrap.type,                              sizeof( pkt_wrap.type   ), 0       );
                        calcCRC = utilCRC16( &pkt_wrap.length,                            sizeof( pkt_wrap.length ), calcCRC );
                        calcCRC = utilCRC16( &fmucomm_rx_cfg[ rx_cfg_idx ].pkt_pl_p[ 0 ], pkt_wrap.length,           calcCRC );
                        
                        // Calculated CRC matches that received ?
                        if( calcCRC == pkt_wrap.crc )
                        {
                            // Identify message as valid.
                            fmucomm_rx_valid[ pkt_wrap.type ] = true;
                            
                            // Copy the wrapper field to the applicable message
                            // for possible processing by accessed module.
                            memcpy( fmucomm_rx_cfg[ rx_cfg_idx ].pkt_wrap_p,
                                    &pkt_wrap,
                                    sizeof( pkt_wrap ) );
                        }
                        
                        // Entire message has been received, reset the state
                        // machine for reception of next message.
                        UDPState = UDP_SM_HEADER_0;
                        
                        break;
                    }
                }
            }
        }
    }
}