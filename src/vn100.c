/*******************************************************************************
/
/   Filename:   vn100.c
/
*******************************************************************************/

#include <xc.h>
#include "vn100.h"
#include "coretime.h"
#include "spi.h"
#include "stdtypes.h"


static VN100_MODEL_NUMBER       modelNumber;
static VN100_HARDWARE_REVISION  hardwareRevision;
static VN100_SERIAL_NUMBER      serialNumber;
static VN100_FIRMWARE_VERSION   firmwareVersion;
static VN100_IMU_MEASUREMENTS   imuMeasurements;


// Local function prototypes. ==================================================

static int VN100Init();
static unsigned int VN100RegSizeGet(VN100_REG_E reg);
static VN100_SPI_PKT* VN100ReadReg(VN100_REG_E reg);
//static VN100_SPI_PKT* VN100WriteReg(VN100_REG_E reg, void* regData);


//==============================================================================

void VN100Task()
{
    static enum
    {
        SM_INIT = 0,
        SM_GET_MODEL_NUMBER,
        SM_GET_HARDWARE_REVISION,
        SM_GET_SERIAL_NUMBER,
        SM_GET_FIRMWARE_VERSION,
        SM_INIT_IMU_TIME,
        SM_GET_IMU_MEASUREMENTS,
        SM_ADD_DELAY,
    } vn100TaskState = SM_INIT;

    VN100_SPI_PKT* pkt;

    static uint32_t imuTime;

    switch (vn100TaskState)
    {
        case SM_INIT:
        {
            if (VN100Init() == 0)
            {
                vn100TaskState = SM_GET_MODEL_NUMBER;
            }
            break;
        }
        case SM_GET_MODEL_NUMBER:
        {
            pkt = VN100ReadReg(VN100_REG_MODEL);
            if ((pkt != 0) && (pkt->header.response.errID == VN100_ERR_NONE))
            {
                memcpy(&modelNumber, pkt->payload,
                        sizeof(modelNumber));
                vn100TaskState = SM_GET_HARDWARE_REVISION;
            }
            break;
        }
        case SM_GET_HARDWARE_REVISION:
        {
            pkt = VN100ReadReg(VN100_REG_HWREV);
            if ((pkt != 0) && (pkt->header.response.errID == VN100_ERR_NONE))
            {
                memcpy(&hardwareRevision, pkt->payload,
                        sizeof(hardwareRevision));
                vn100TaskState = SM_GET_SERIAL_NUMBER;
            }
            break;
        }
        case SM_GET_SERIAL_NUMBER:
        {
            pkt = VN100ReadReg(VN100_REG_SN);
            if ((pkt != 0) && (pkt->header.response.errID == VN100_ERR_NONE))
            {
                memcpy(&serialNumber, pkt->payload,
                        sizeof(serialNumber));
                vn100TaskState = SM_GET_FIRMWARE_VERSION;
            }
            break;
        }
        case SM_GET_FIRMWARE_VERSION:
        {
            pkt = VN100ReadReg(VN100_REG_FWVER);
            if ((pkt != 0) && (pkt->header.response.errID == VN100_ERR_NONE))
            {
                memcpy(&firmwareVersion, pkt->payload,
                        sizeof(firmwareVersion));
                vn100TaskState = SM_INIT_IMU_TIME;
            }
            break;
        }
        case SM_INIT_IMU_TIME:
        {
            imuTime = CoreTime32usGet();
            // No break;
        }
        case SM_GET_IMU_MEASUREMENTS:
        {
            if (CoreTime32usGet() >= imuTime)
            {
                pkt = VN100ReadReg(VN100_REG_IMU);
                if ((pkt != 0) && 
                        (pkt->header.response.errID == VN100_ERR_NONE))
                {
                    memcpy(&imuMeasurements, pkt->payload,
                            sizeof(imuMeasurements));
                    vn100TaskState = SM_ADD_DELAY;
                }
            }
            break;
        }
        case SM_ADD_DELAY:
        {
            imuTime += 1250;    // 1250 us period = 800 Hz
            vn100TaskState = SM_GET_IMU_MEASUREMENTS;
            break;
        }
    }
}


//==============================================================================

static int VN100Init()
{
    int retVal = 1;
    
//    const VN100_COMM_PROTO_CTRL cpc = {
//        .serialCount.val        = 2,    // SYNCIN_TIME
//        .serialStatus.val       = 1,    // VPE Status
//        .spiCount.val           = 2,    // SYNCIN_TIME
//        .spiStatus.val          = 1,    // VPE Status
//        .serialChecksum.val     = 3,    // 16-Bit CRC
//        .spiChecksum.val        = 3,    // 16-Bit CRC
//        .errorMode.val          = 1,    // Send Error
//    };
    
//    
//    TODO: Write configuration to VN100
//    
    
    return retVal;
}


//==============================================================================

//static VN100_SPI_PKT* VN100WriteReg(VN100_REG_E reg, void* regData)
//{
//    VN100_SPI_PKT* retVal = 0;
//
//    static VN100_SPI_PKT pkt;
//    
//    static SPI_XFER vn100spi = {
//        .port = SPI_PORT_SPI2,
//    };
//
//    static enum
//    {
//        SM_REQUEST_START,
//        SM_REQUEST_FINISH,
//        SM_WAIT,
//        SM_RESPONSE_START,
//        SM_RESPONSE_FINISH,
//    } readRegState = SM_REQUEST_START;
//
//    static uint32_t requestTime;
//
//    switch (readRegState)
//    {
//        case SM_REQUEST_START:
//        {
//            pkt.header.request.cmdID = VN100_CMD_WRITE_REG;
//            pkt.header.request.regID = reg;
//            pkt.header.request.zero_0 = 0;
//            pkt.header.request.zero_1 = 0;
//
//            memcpy(pkt.payload, regData, VN100RegSizeGet(reg));
//            
//            vn100spi.rxBuf = 0;
//            vn100spi.txBuf = (uint8_t*)&pkt;
//            vn100spi.length =
//                    sizeof(VN100_SPI_HEADER) + VN100RegSizeGet(reg);
//
//            if (SPIXfer(&vn100spi) == 0)
//            {
//                readRegState = SM_REQUEST_FINISH;
//            }
//            break;
//        }
//        case SM_REQUEST_FINISH:
//        {
//            if (vn100spi.xferDone != 0)
//            {
//                requestTime = CoreTime32usGet();
//                readRegState = SM_WAIT;
//            }
//            break;
//        }
//        case SM_WAIT:
//        {
//            // Per VN-100 user manual, wait at least 50us before
//            // issuing the response packet.
//            if ((CoreTime32usGet() - requestTime) < 100)
//            {
//                break;
//            }
//            else
//            {
//                readRegState = SM_RESPONSE_START;
//            }
//            // No break.
//        }
//        case SM_RESPONSE_START:
//        {
//            vn100spi.rxBuf = (uint8_t*)&pkt;
//            vn100spi.txBuf = 0;
//            vn100spi.length = 
//                    sizeof(VN100_SPI_HEADER) + VN100RegSizeGet(reg);
//
//            if (SPIXfer(&vn100spi) == 0)
//            {
//                readRegState = SM_RESPONSE_FINISH;
//            }
//            break;
//        }
//        case SM_RESPONSE_FINISH:
//        {
//            if (vn100spi.xferDone != 0)
//            {
//                readRegState = SM_REQUEST_START;
//                retVal = &pkt;
//            }
//            break;
//        }
//    }
//    return retVal;
//}


//==============================================================================

static VN100_SPI_PKT* VN100ReadReg(VN100_REG_E reg)
{
    VN100_SPI_PKT* retVal = 0;

    static VN100_SPI_PKT pkt;
    
    static SPI_XFER vn100spi = {
        .port = SPI_PORT_SPI2,
    };

    static enum
    {
        SM_REQUEST_START,
        SM_REQUEST_FINISH,
        SM_WAIT,
        SM_RESPONSE_START,
        SM_RESPONSE_FINISH,
    } readRegState = SM_REQUEST_START;

    static uint32_t requestTime;

    switch (readRegState)
    {
        case SM_REQUEST_START:
        {
            pkt.header.request.cmdID = VN100_CMD_READ_REG;
            pkt.header.request.regID = reg;
            pkt.header.request.zero_0 = 0;
            pkt.header.request.zero_1 = 0;

            vn100spi.rxBuf = 0;
            vn100spi.txBuf = (uint8_t*)&pkt;
            vn100spi.length = sizeof(pkt.header.request);

            if (SPIXfer(&vn100spi) == 0)
            {
                readRegState = SM_REQUEST_FINISH;
            }
            break;
        }
        case SM_REQUEST_FINISH:
        {
            if (vn100spi.xferDone != 0)
            {
                requestTime = CoreTime32usGet();
                readRegState = SM_WAIT;
            }
            break;
        }
        case SM_WAIT:
        {
            // Per VN-100 user manual, wait at least 50us before
            // issuing the response packet.
            if ((CoreTime32usGet() - requestTime) < 100)
            {
                break;
            }
            else
            {
                readRegState = SM_RESPONSE_START;
            }
            // No break.
        }
        case SM_RESPONSE_START:
        {
            vn100spi.rxBuf = (uint8_t*)&pkt;
            vn100spi.txBuf = 0;
            vn100spi.length = 
                    VN100RegSizeGet(reg) + 4;   // Add 4-byte response header.

            if (SPIXfer(&vn100spi) == 0)
            {
                readRegState = SM_RESPONSE_FINISH;
            }
            break;
        }
        case SM_RESPONSE_FINISH:
        {
            if (vn100spi.xferDone != 0)
            {
                readRegState = SM_REQUEST_START;
                retVal = &pkt;
            }
            break;
        }
    }
    return retVal;
}


//==============================================================================

static unsigned int VN100RegSizeGet(VN100_REG_E reg)
{
    int size = 0;

    switch (reg)
    {
        case VN100_REG_TAG:         // ID: 0
            size = 20;
            break;
        case VN100_REG_MODEL:       // ID: 1
            size = 24;
            break;
        case VN100_REG_HWREV:       // ID: 2
            size = 4;
            break;
        case VN100_REG_SN:          // ID: 3
            size = 4;
            break;
        case VN100_REG_FWVER:       // ID: 4
            size = 4;
            break;
        case VN100_REG_SBAUD:       // ID: 5
            size = 4;
            break;
        case VN100_REG_ADOR:        // ID: 6
            size = 4;
            break;
        case VN100_REG_ADOF:        // ID: 7
            size = 4;
            break;
        case VN100_REG_YPR:         // ID: 8
            size = 12;
            break;
        case VN100_REG_QTN:         // ID: 9
            size = 16;
            break;
        case VN100_REG_QTM:         // ID: 10
            break;
        case VN100_REG_QTA:         // ID: 11
            break;
        case VN100_REG_QTR:         // ID: 12
            break;
        case VN100_REG_QMA:         // ID: 13
            break;
        case VN100_REG_QAR:         // ID: 14
            break;
        case VN100_REG_QMR:         // ID: 15
            size = 52;
            break;
        case VN100_REG_DCM:         // ID: 16
            break;
        case VN100_REG_MAG:         // ID: 17
            size = 12;
            break;
        case VN100_REG_ACC:         // ID: 18
            size = 12;
            break;
        case VN100_REG_GYR:         // ID: 19
            size = 12;
            break;
        case VN100_REG_MAR:         // ID: 20
            size = 36;
            break;
        case VN100_REG_REF:         // ID: 21
            size = 24;
            break;
        case VN100_REG_SIG:         // ID: 22
            break;
        case VN100_REG_HSI:         // ID: 23
            size = 48;
            break;
        case VN100_REG_ATP:         // ID: 24
            break;
        case VN100_REG_ACT:         // ID: 25
            size = 48;
            break;
        case VN100_REG_RFR:         // ID: 26
            size = 36;
            break;
        case VN100_REG_YMR:         // ID: 27
            size = 48;
            break;
        case VN100_REG_ACG:         // ID: 28
            break;
        case VN100_REG_PROT:        // ID: 30
            size = 7;
            break;
        case VN100_REG_SYNC:        // ID: 32
            size = 20;
            break;
        case VN100_REG_STAT:        // ID: 33
            size = 12;
            break;
        case VN100_REG_VPE:         // ID: 35
            size = 4;
            break;
        case VN100_REG_VPEMBT:      // ID: 36
            size = 36;
            break;
        case VN100_REG_VPEABT:      // ID: 38
            size = 36;
            break;
        case VN100_REG_MCC:         // ID: 44
            size = 4;
            break;
        case VN100_REG_CMC:         // ID: 47
            size = 48;
            break;
        case VN100_REG_VCM:         // ID: 50
            size = 12;
            break;
        case VN100_REG_VCC:         // ID: 51
            size = 8;
            break;
        case VN100_REG_VCS:         // ID: 52
            size = 8;
            break;
        case VN100_REG_IMU:         // ID: 54
            size = 44;
            break;
        case VN100_REG_BOR1:        // ID: 75
            size = 22;
            break;
        case VN100_REG_BOR2:        // ID: 76
            size = 22;
            break;
        case VN100_REG_BOR3:        // ID: 77
            size = 22;
            break;
        case VN100_REG_DTDV:        // ID: 80
            size = 28;
            break;
        case VN100_REG_DTDVC:       // ID: 82
            size = 6;
            break;
        case VN100_REG_RVC:         // ID: 83
            size = 32;
            break;
        case VN100_REG_GCMP:        // ID: 84
            size = 48;
            break;
        case VN100_REG_IMUF:        // ID: 85
            size = 15;
            break;
        case VN100_REG_YPRTBAAR:    // ID: 239
            size = 36;
            break;
        case VN100_REG_YPRTIAAR:    // ID: 240
            size = 36;
            break;
        case VN100_REG_RAW:         // ID: 251
            break;
        case VN100_REG_CMV:         // ID: 252
            break;
        case VN100_REG_STV:         // ID: 253
            break;
        case VN100_REG_COV:         // ID: 254
            break;
        case VN100_REG_CAL:         // ID: 255
            break;
        default:
            break;
    }

    return size;
}


//==============================================================================

// Calculates the 8-bit checksum for the given byte sequence. 

unsigned char calculateChecksum(unsigned char data[], unsigned int length)
{ 
    unsigned int i; 
    unsigned char cksum = 0;
    
    for (i = 0; i < length; i++)
    {
        cksum ^= data[i];
    }
    
    return cksum;
}


//==============================================================================

// Calculates the 16-bit CRC for the given ASCII or binary message.

unsigned short calculateCRC(unsigned char data[], unsigned int length)
{
    unsigned int i;
    unsigned short crc = 0;
    
    for (i = 0; i < length; i++)
    {
        crc = (unsigned char)(crc >> 8) | (crc << 8);
        crc ^= data[i]; crc ^= (unsigned char)(crc & 0xff) >> 4;
        crc ^= crc << 12; crc ^= (crc & 0x00ff) << 5;
    } 
    
    return crc;
}

