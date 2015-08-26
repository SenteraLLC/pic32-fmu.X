////////////////////////////////////////////////////////////////////////////////
/// @file
/// @brief Flight Management Unit (FMU) Ethernet Communication.
////////////////////////////////////////////////////////////////////////////////

// *****************************************************************************
// ************************** System Include Files *****************************
// *****************************************************************************

// *****************************************************************************
// ************************** User Include Files *******************************
// *****************************************************************************

#include "fmucomm.h"
#include <sys/attribs.h>
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

// Structure defining elements for the configuration of received messages.
typedef struct
{
    uint8_t           header0;
    uint8_t           header1;
    uint8_t           header2;
    uint8_t           type;
    uint16_t          lengthLb;  // length - lower bound
    uint16_t          lengthUb;  // length - upper bound
    
} FMUCOMM_RX_CFG;

// Structure defining elements for the configuration of transmitted messages.
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

// Configuration data for received messages.
//
// Note: length lower-bound and upper-bound and required to be set to 
// correspond with packet buffer size definition (i.e. see structure definition 
// of message payload sizes); otherwise invalid array access can occur.
//
static const FMUCOMM_RX_CFG fmucommRxCfg [ ] =
{
    { 'U', 'M', 'N', 0x00, 14,   14 },  // FMUCOMM_TYPE_HOST_HEARTBEAT     
    { 'U', 'M', 'N', 0x01,  3,   60 },  // FMUCOMM_TYPE_CTRL_SURFACE_CMD    
    { 'U', 'M', 'N', 0x02,  1, 1024 },  // FMUCOMM_TYPE_GPS_CMD   
    { 'U', 'M', 'N', 0x7F,  1, 1024 },  // FMUCOMM_TYPE_HOST_EXCEPTION 
};

// Configuration data for transmitted messages
static const FMUCOMM_TX_CFG fmucommTxCfg [ ] =
{
    { 'U', 'M', 'N', 0x80 },  // FMUCOMM_TYPE_FMU_HEARTBEAT     
    { 'U', 'M', 'N', 0x81 },  // FMUCOMM_TYPE_IMU_DATA          
    { 'U', 'M', 'N', 0x82 },  // FMUCOMM_TYPE_GPS_DATA          
    { 'U', 'M', 'N', 0x83 },  // FMUCOMM_TYPE_AIR_DATA          
    { 'U', 'M', 'N', 0x84 },  // FMUCOMM_TYPE_CTRL_SURFACE_DATA 
    { 'U', 'M', 'N', 0xFF },  // FMUCOMM_TYPE_FMU_EXCEPTION     
};

// Ethernet client socket settings.
static const uint32_t   clientIPAddr = 0x0B8FA8C0;  // (192.168.143.11) in big-endian.
static const UDP_PORT   clientPort   = 55455;
static       UDP_SOCKET clientSocket = INVALID_UDP_SOCKET;

// Ethernet server socket settings.
static const UDP_PORT   servertPort  = 55455;
static       UDP_SOCKET serverSocket = INVALID_UDP_SOCKET;

// Allocation for received messages' payload.
static FMUCOMM_HOST_HEARTBEAT_PL   FMUCommHostHeartbeatPl;
static FMUCOMM_CTRL_SURFACE_CMD_PL FMUCommCtrlSurfaceCmdPl;
static FMUCOMM_HOST_GPS_CMD_PL     FMUCommHostGPSCmdPl;
static FMUCOMM_HOST_EXCEPTION_PL   FMUCommHostExeptionPl;

// Received packet data.
static FMUCOMM_RX_PKT FMUCommRxPkt[ FMUCOMM_RX_TYPE_MAX ] =
{
    { false, { 0 }, &FMUCommHostHeartbeatPl.pl_u8[ 0 ]  },   // FMUCOMM_TYPE_HOST_HEARTBEAT
    { false, { 0 }, &FMUCommCtrlSurfaceCmdPl.pl_u8[ 0 ] },   // FMUCOMM_TYPE_CTRL_SURFACE_CMD
    { false, { 0 }, &FMUCommHostGPSCmdPl.pl_u8[ 0 ]     },   // FMUCOMM_TYPE_GPS_CMD
    { false, { 0 }, &FMUCommHostExeptionPl.pl_u8[ 0 ]   },   // FMUCOMM_TYPE_HOST_EXCEPTION
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
            FMUCOMM_RX_TYPE_E rxIdx;
            
            // Clear validity identifiers for all received messages.
            //
            // Note: validity flags are only set for one iteration of the 
            // FMUCommTask.  Therefore, modules which use the received data
            // must fully process the message or copy the message during
            // an execution.
            //
            for( rxIdx = (FMUCOMM_RX_TYPE_E) 0;
                 rxIdx < FMUCOMM_RX_TYPE_MAX;
                 rxIdx++ )
            {
                FMUCommRxPkt[ rxIdx ].valid = false;
            }
            
            // Read UDP data and decode.
            FMUCommRead();
        }
    }
}

bool FMUCommSet( FMUCOMM_TX_TYPE_E pktType, uint8_t* pl_p, uint16_t plLen )
{
          bool            setSuccess = false;
    const FMUCOMM_TX_CFG* pktCfg_p;
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
            pktCfg_p = &fmucommTxCfg[ pktType ];
            
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

const FMUCOMM_RX_PKT* FMUCommGet( FMUCOMM_RX_TYPE_E pktType )
{
    return &FMUCommRxPkt[ pktType ];
}

// *****************************************************************************
// ************************** Static Functions *********************************
// *****************************************************************************

// Read and decode Ethernet data
//
// Note: Only a singe message type can be read per execution (e.g. a single
// Host-Heartbeat message).  If multiple message of the same type are received
// during a single execution the previous message will be overwritten with 
// the later message.
//
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
    
    static FMUCOMM_PKT_WRAP pkt_wrap;
    
    FMUCOMM_RX_TYPE_E rxCfgIdx;
    
    uint16_t UDPByteCnt;
    uint16_t UDPByteIdx;
    
    // FMU socket is open for reception ?
    if( UDPIsOpened( serverSocket ) == true )
    {
        UDPByteCnt = UDPIsGetReady( serverSocket );
        
        // Socket has received a segment ?
        if( UDPByteCnt != 0 )
        {
            // Increment through the received UDP bytes.
            for( UDPByteIdx = 0;
                 UDPByteIdx < UDPByteCnt;
                 UDPByteIdx++ )
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
                        bool typeValid = false;
                         
                        (void) UDPGet( &pkt_wrap.type );
                        
                        for( rxCfgIdx = 0;
                             rxCfgIdx < FMUCOMM_RX_TYPE_MAX;
                             rxCfgIdx++ )
                        {
                            // Received UDP byte matches expected value ?
                            if( pkt_wrap.type == fmucommRxCfg[ rxCfgIdx ].type )
                            {
                                typeValid = true;
                                
                                // Exit loop when match is found so 'rxCfgIdx'
                                // is maintained at the matching index
                                break;
                            }
                        }
                        
                        // Receive message type was valid ?
                        if( typeValid == true )
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
                    case UDP_SM_LENGTH_LSB:
                    {
                        (void) UDPGet( &pkt_wrap.lengthLsb );
                        
                        UDPState++;
                        
                        break;
                    }
                    case UDP_SM_LENGTH_MSB:
                    {
                        (void) UDPGet( &pkt_wrap.lengthMsb );
                        
                        // Received length matches required value ?
                        if( ( pkt_wrap.length >= fmucommRxCfg[ rxCfgIdx ].lengthLb ) &&
                            ( pkt_wrap.length <= fmucommRxCfg[ rxCfgIdx ].lengthUb ) )
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
                        static uint16_t plIdx = 0;
                        
                        // Read the next UPD byte into the applicable packet's 
                        // payload allocation.
                        (void) UDPGet( &FMUCommRxPkt[ rxCfgIdx ].pl_p[ plIdx ] );
                        
                        plIdx++;
                        
                        // Entire payload has been received ?
                        if( plIdx >= pkt_wrap.length )
                        {
                            // Re-initialize payload index for next
                            // evaluation.
                            plIdx = 0;
                            
                            UDPState++;
                        }
                        
                        break;
                    }
                    case UDP_SM_CRC_LSB:
                    {
                        (void) UDPGet( &pkt_wrap.crcLsb );
                        
                        UDPState++;
                        
                        break;
                    }
                    case UDP_SM_CRC_MSB:
                    {
                        uint16_t calcCRC;
                        
                        (void) UDPGet( &pkt_wrap.crcMsb );
                        
                        // Calculate CRC of the packet - includes the type, length, and
                        // payload.
                        //
                        // Note: the starting CRC value is '0', and the previous CRC value
                        // is used for the subsequent calculations.  This piece-wise
                        // calculation is necessary because packet fields are in non-
                        // contiguous memory locations.
                        //
                        calcCRC = utilCRC16( &pkt_wrap.type,                      sizeof( pkt_wrap.type   ), 0       );
                        calcCRC = utilCRC16( &pkt_wrap.length,                    sizeof( pkt_wrap.length ), calcCRC );
                        calcCRC = utilCRC16( &FMUCommRxPkt[ rxCfgIdx ].pl_p[ 0 ], pkt_wrap.length,           calcCRC );
                        
                        // Calculated CRC matches that received ?
                        if( calcCRC == pkt_wrap.crc )
                        {
                            // Identify message as valid.
                            FMUCommRxPkt[ rxCfgIdx ].valid = true;
                            
                            // Copy the wrapper field to the applicable message
                            // for possible processing by accessed module.
                            memcpy( &FMUCommRxPkt[ rxCfgIdx ].wrap,
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