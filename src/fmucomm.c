/*******************************************************************************
/
/   Filename:   fmucomm.c
/
*******************************************************************************/

#include <xc.h>
#include <sys/attribs.h>
#include "fmucomm.h"
#include "tcpip/tcpip.h"
#include "util.h"

// The number of bytes for the packet wrapping (header and footer) information.
// This includes the fields:
//  - Header            3 bytes
//  - Packet Type       1 byte
//  - Payload Length    2 bytes
//  - CRC               2 bytes
//
#define FMUCOMM_WRAP_SIZE 8

typedef struct
{
    uint8_t         header0;
    uint8_t         header1;
    uint8_t         header2;
    FMUCOMM_TYPE_E  type;
} FMUCOMM_CFG;

static FMUCOMM_CFG fmucomm_cfg [ ] = 
{
    { 'U', 'M', 'N', 0x00 },  // FMUCOMM_TYPE_FMU_HEARTBEAT     
    { 'U', 'M', 'N', 0x01 },  // FMUCOMM_TYPE_IMU_DATA          
    { 'U', 'M', 'N', 0x7F },  // FMUCOMM_TYPE_GPS_DATA 
    
    { 'U', 'M', 'N', 0x80 },  // FMUCOMM_TYPE_FMU_HEARTBEAT     
    { 'U', 'M', 'N', 0x81 },  // FMUCOMM_TYPE_IMU_DATA          
    { 'U', 'M', 'N', 0x82 },  // FMUCOMM_TYPE_GPS_DATA          
    { 'U', 'M', 'N', 0x83 },  // FMUCOMM_TYPE_AIR_DATA          
    { 'U', 'M', 'N', 0x84 },  // FMUCOMM_TYPE_CTRL_SURFACE_DATA 
    { 'U', 'M', 'N', 0xFF },  // FMUCOMM_TYPE_FMU_EXCEPTION     
};

//==============================================================================

// Client IP Address (239.192.143.140) in big-endian.   // DEBUG: (192.168.143.11) = 0x0B8EA8C0 Computer IP
static const uint32_t   clientIPAddr = 0x0B8FA8C0;
static const UDP_PORT   clientPort   = 55455;
static       UDP_SOCKET clientSocket = INVALID_UDP_SOCKET;

void FMUCommTask()
{
    static enum
    {
        SM_INIT,
        SM_IDLE,
    } FMUCommTaskState = SM_INIT;

    switch (FMUCommTaskState)
    {
        case SM_INIT:
        {
            // Open up the port. Unicast IP Address
            clientSocket = UDPOpenEx( clientIPAddr,
                                      UDP_OPEN_IP_ADDRESS,
                                      0,
                                      clientPort );
            
            // Port opened successfully ?
            if( clientSocket != INVALID_UDP_SOCKET )
            {
                FMUCommTaskState++;
            }
            
            break;
        }
        case SM_IDLE:
        {
            // Do nothing.
            break;
        }
    }
}

bool FMUCommSet( FMUCOMM_TYPE_E pktType, uint8_t* pl_p, uint16_t plLen )
{
    bool         setSuccess = false;
    FMUCOMM_CFG* pktCfg_p;
    uint16_t     calcCRC;
    
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
            pktCfg_p = &fmucomm_cfg[ pktType ];
            
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


//==============================================================================

