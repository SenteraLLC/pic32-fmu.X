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

static unsigned int VN100RegSizeGet(VN100_REG_E reg);
static VN100_SPI_RESPONSE_PKT* VN100ReadReg(VN100_REG_E reg);


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
        SM_GET_IMU_MEASUREMENTS,
        SM_DELAY,
    } vn100TaskState = SM_INIT;

    VN100_SPI_RESPONSE_PKT* response;

    switch (vn100TaskState)
    {
        case SM_INIT:
        {
            // TODO: Initialize VN-100 module.
            vn100TaskState = SM_GET_MODEL_NUMBER;
            break;
        }
        case SM_GET_MODEL_NUMBER:
        {
            response = VN100ReadReg(VN100_REG_MODEL);
            if (response != 0)
            {
                memcpy(&modelNumber, response->payload,
                        sizeof(modelNumber));
                vn100TaskState = SM_GET_HARDWARE_REVISION;
            }
            break;
        }
        case SM_GET_HARDWARE_REVISION:
        {
            response = VN100ReadReg(VN100_REG_HWREV);
            if (response != 0)
            {
                memcpy(&hardwareRevision, response->payload,
                        sizeof(hardwareRevision));
                vn100TaskState = SM_GET_SERIAL_NUMBER;
            }
            break;
        }
        case SM_GET_SERIAL_NUMBER:
        {
            response = VN100ReadReg(VN100_REG_SN);
            if (response != 0)
            {
                memcpy(&serialNumber, response->payload,
                        sizeof(serialNumber));
                vn100TaskState = SM_GET_FIRMWARE_VERSION;
            }
            break;
        }
        case SM_GET_FIRMWARE_VERSION:
        {
            response = VN100ReadReg(VN100_REG_FWVER);
            if (response != 0)
            {
                memcpy(&firmwareVersion, response->payload,
                        sizeof(firmwareVersion));
                vn100TaskState = SM_GET_IMU_MEASUREMENTS;
            }
            break;
        }
        case SM_GET_IMU_MEASUREMENTS:
        {
            response = VN100ReadReg(VN100_REG_IMU);
            if (response != 0)
            {
                memcpy(&imuMeasurements, response->payload,
                        sizeof(imuMeasurements));
                vn100TaskState = SM_DELAY;
            }
            break;
        }
        case SM_DELAY:
        {
            // TODO: Delay between IMU Measurements reads.
            vn100TaskState = SM_GET_IMU_MEASUREMENTS;
        }
    }
}


//==============================================================================


static VN100_SPI_RESPONSE_PKT* VN100ReadReg(VN100_REG_E reg)
{
    VN100_SPI_RESPONSE_PKT *retVal = 0;

    static VN100_SPI_RESPONSE_PKT response;
    
    static SPI_XFER vn100spi = {
        .port       = SPI_PORT_SPI2,
        .rxBuf      = (uint8_t*)&response,
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
    static uint8_t request[4];

    switch (readRegState)
    {
        case SM_REQUEST_START:
        {
            request[0] = VN100_CMD_READ_REG;
            request[1] = reg;
            request[2] = 0x00;
            request[3] = 0x00;

            vn100spi.txBuf = request;
            vn100spi.length = sizeof(request);

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
                retVal = &response;
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
        case VN100_REG_TAG:
            break;
        case VN100_REG_MODEL:
            size = 24;
            break;
        case VN100_REG_HWREV:
            size = 4;
            break;
        case VN100_REG_SN:
            size = 4;
            break;
        case VN100_REG_FWVER:
            size = 4;
            break;
        case VN100_REG_SBAUD:
            break;
        case VN100_REG_ADOR:
            break;
        case VN100_REG_ADOF:
            break;
        case VN100_REG_YPR:
            break;
        case VN100_REG_QTN:
            break;
        case VN100_REG_QTM:
            break;
        case VN100_REG_QTA:
            break;
        case VN100_REG_QTR:
            break;
        case VN100_REG_QMA:
            break;
        case VN100_REG_QAR:
            break;
        case VN100_REG_QMR:
            break;
        case VN100_REG_DCM:
            break;
        case VN100_REG_MAG:
            break;
        case VN100_REG_ACC:
            break;
        case VN100_REG_GYR:
            break;
        case VN100_REG_MAR:
            break;
        case VN100_REG_REF:
            break;
        case VN100_REG_SIG:
            break;
        case VN100_REG_HSI:
            break;
        case VN100_REG_ATP:
            break;
        case VN100_REG_ACT:
            break;
        case VN100_REG_RFR:
            break;
        case VN100_REG_YMR:
            break;
        case VN100_REG_ACG:
            break;
        case VN100_REG_PROT:
            break;
        case VN100_REG_SYNC:
            break;
        case VN100_REG_STAT:
            break;
        case VN100_REG_VPE:
            break;
        case VN100_REG_VPEMBT:
            break;
        case VN100_REG_VPEABT:
            break;
        case VN100_REG_MCC:
            break;
        case VN100_REG_CMC:
            break;
        case VN100_REG_VCM:
            break;
        case VN100_REG_VCC:
            break;
        case VN100_REG_VCS:
            break;
        case VN100_REG_IMU:
            size = 44;
            break;
        case VN100_REG_BOR1:
            break;
        case VN100_REG_BOR2:
            break;
        case VN100_REG_BOR3:
            break;
        case VN100_REG_DTDV:
            break;
        case VN100_REG_DTDVC:
            break;
        case VN100_REG_RVC:
            break;
        case VN100_REG_GCMP:
            break;
        case VN100_REG_IMUF:
            break;
        case VN100_REG_YPRTBAAR:
            break;
        case VN100_REG_YPRTIAAR:
            break;
        case VN100_REG_RAW:
            break;
        case VN100_REG_CMV:
            break;
        case VN100_REG_STV:
            break;
        case VN100_REG_COV:
            break;
        case VN100_REG_CAL:
            break;
        default:
            break;
    }

    return size;
}

