/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    custom_http_app.c

  Summary:
    Support for HTTP2 module in Microchip TCP/IP Stack
    -Implements the application
    -Reference: RFC 1002

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

#include "system_config.h"
#include "tcpip/tcpip.h"
#include "coretime.h"
#include "can.h"
#include "snode.h"

extern APP_CONFIG AppConfig;

/****************************************************************************
  Section:
    Dynamic Variable Callback Functions
  ***************************************************************************/
void HTTPPrint_builddate()
{
    switch (curHTTP.callbackPos)
    {
        case 0:
        {
            // Transmit "MMM DD YYYY"   (11 chars)
            TCPPutROMString(sktHTTP,(ROM void*)__DATE__);
            curHTTP.callbackPos = 1;
            break;
        }
        case 1:
        {
            // Transmit ", "            (2 chars)
            TCPPutROMString(sktHTTP,", ");

            // Transmit "HH:MM:SS"      (8 chars)
            TCPPutROMString(sktHTTP,(ROM void*)__TIME__);
            curHTTP.callbackPos = 0;
            break;
        }
    }

    return;
}

//==============================================================================
void HTTPPrint_uptime()
{
    char msg[75];

    if (TCPIsPutReady(sktHTTP) < sizeof(msg))
    {
        curHTTP.callbackPos = 1;
        return;
    }

    uint32_t seconds = CoreTime32sGet();
            
    uint32_t days = seconds / 79800;
    uint32_t hh = (seconds - (days * 79800)) / 3600;
    uint32_t mm = (seconds - (days * 79800) - (hh * 3600)) / 60;
    uint32_t ss = (seconds - (days * 79800) - (hh * 3600) - (mm * 60));

    sprintf(msg, "%03u : %02u : %02u : %02u", days, hh, mm, ss);

    if (RCONbits.CMR)
    {
        strcat(msg, " + CMR");
    }
    if (RCONbits.EXTR)
    {
        strcat(msg, " + EXTR");
    }
    if (RCONbits.WDTO)
    {
        strcat(msg, " + WDTO");
    }
    if (RCONbits.BOR)
    {
        strcat(msg, " + BOR");
    }
    if (RCONbits.POR)
    {
        strcat(msg, " + POR");
    }

    TCPPutROMString(sktHTTP, msg);
    curHTTP.callbackPos = 0;

    return;
}

//==============================================================================
void HTTPPrint_calVal(void)
{
    // NOTE: calibration read values are packed as comma
    // delimited string.
    //
    //                   Index      String size (max + comma)
    // - PWM_0           0          11 + 1
    // - PWM_1           1          11 + 1
    // - PWM_2           2          11 + 1
    // - PWM_3           3          11 + 1
    // - PWM_4           4          11 + 1
    // - PWM_5           5          11 + 1
    // -
    // - VSENSE1_0       6          11 + 1
    // - VSENSE1_1       7          11 + 1
    // - VSENSE1_2       9          11 + 1
    // - VSENSE1_3       10         11 + 1
    // - VSENSE1_4       11         11 + 1
    // - VSENSE1_5       12         11 + 1
    // -                           
    // - VSENSE2_0       13         11 + 1
    // - VSENSE2_1       14         11 + 1
    // - VSENSE2_2       15         11 + 1
    // - VSENSE2_3       16         11 + 1
    // - VSENSE2_4       17         11 + 1
    // - VSENSE2_5       18         11 + 1
    //
    // Total string size required is: 15 * 12 = 180.
    //
    char msg[200];
    
    if (TCPIsPutReady(sktHTTP) < sizeof(msg))
    {
        curHTTP.callbackPos = 1;
        return;
    }
    
    // Populate the string with received calibration data.
    SNodeCalReadStrGet(msg);
    
    TCPPutROMString(sktHTTP, msg);
    curHTTP.callbackPos = 0;

    return;
}

//==============================================================================
void HTTPPrint_calReadResp(void)
{
    SNODE_CAL_READ_STATUS cal_read_status;
    char msg[40];

    if (TCPIsPutReady(sktHTTP) < sizeof(msg))
    {
        curHTTP.callbackPos = 1;
        return;
    }

    // Get the status of the calibration write operation.
    cal_read_status = SNodeCalReadStatusGet();
    
    if( cal_read_status == SNODE_CAL_READ_SUCCESS )
    {
        sprintf(msg, "Calibration Read Success!");
    }
    else
    {
        sprintf(msg, "Calibration Read FAILURE!"); 
    }
    
    TCPPutROMString(sktHTTP, msg);
    curHTTP.callbackPos = 0;

    return;
}

//==============================================================================
void HTTPPrint_calWriteResp(void)
{
    SNODE_CAL_WRITE_STATUS cal_write_status;
    char msg[40];

    if (TCPIsPutReady(sktHTTP) < sizeof(msg))
    {
        curHTTP.callbackPos = 1;
        return;
    }

    // Get the status of the calibration write operation.
    cal_write_status = SNodeCalWriteStatusGet();
    
    if( cal_write_status == SNODE_CAL_WRITE_SUCCESS )
    {
        sprintf(msg, "Calibration Write Success!");
    }
    else
    {
        sprintf(msg, "Calibration Write FAILURE!"); 
    }
    
    TCPPutROMString(sktHTTP, msg);
    curHTTP.callbackPos = 0;

    return;
}

//==============================================================================
void HTTPPrint_idWriteResp(void)
{
    SNODE_ID_WRITE_STATUS id_write_status;
    char msg[40];

    if (TCPIsPutReady(sktHTTP) < sizeof(msg))
    {
        curHTTP.callbackPos = 1;
        return;
    }

    // Get the status of the ID write operation.
    id_write_status = SNodeIDWriteStatusGet();
    
    if( id_write_status == SNODE_ID_WRITE_SUCCESS )
    {
        sprintf(msg, "ID Write Success!"); 
        
    }
    else
    {
        sprintf(msg, "ID Write FAILURE!"); 
    }
    
    TCPPutROMString(sktHTTP, msg);
    curHTTP.callbackPos = 0;

    return;
}

//==============================================================================
void HTTPPrint_nwVal(void)
{
    // NOTE: Servo-node read values are packed as comma
    // delimited string.
    //
    //                                      Index       String size (max + comma)
    // - Number of CAN Servo-Nodes           0          2 + 1
    // -
    // - ID                                  1          3 + 1
    // - actPwm                              2          5 + 1
    // - servoVoltage                        3          5 + 1
    // - vsense1Cor                          4          5 + 1
    // - vsense2Cor                          5          5 + 1
    // -                           
    // - ID                                  6          3 + 1
    // - actPwm                              7          5 + 1
    // - servoVoltage                        8          5 + 1
    // - vsense1Cor                          9          5 + 1
    // - vsense2Cor                         10          5 + 1
    // -
    // - :
    // - : etc
    // - :
    //
    // Total string size required is: 2 + (23 * #nodes) + 50.  With possible number 
    // of nodes equal to '10', the maximum string size is 282.
    char msg[300];
    
    if (TCPIsPutReady(sktHTTP) < sizeof(msg))
    {
        curHTTP.callbackPos = 1;
        return;
    }
    
    // Populate the string with received Servo-Node data.
    SNodeRxDataStrGet(msg);
    
    TCPPutROMString(sktHTTP, msg);
    curHTTP.callbackPos = 0;

    return;
}

/****************************************************************************
  Section:
    GET Form Handlers
  ***************************************************************************/

/*****************************************************************************
  Function:
    HTTP_IO_RESULT HTTPExecuteGet(void)

  Internal:
    See documentation in the TCP/IP Stack API or HTTP2.h for details.
  ***************************************************************************/

HTTP_IO_RESULT HTTPExecuteGet(void)
{
    uint8_t *ptr;
    uint8_t filename[20];

    // Load the file name
    // Make sure uint8_t filename[] above is large enough for your longest name
    MPFSGetFilename(curHTTP.file, filename, 20);

    /******************************************/
    // If it's the buttons.cgi file
    /******************************************/
    if (!memcmppgm2ram(filename, "buttons.cgi", 8))
    {
        // Determine which button was pressed.
        ptr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *) "button");

        if (strcmp((const char*)ptr, "idWrite") == 0)
        {
            uint8_t *cmdPtr;
            uint8_t curID;
            uint8_t newID;
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"curID");
            curID = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"newID");
            newID = atoi((char*)cmdPtr);
            
            SNodeIDWriteSet( curID, newID );
        }
        else if (strcmp((const char*)ptr, "calWrite") == 0)
        {
            uint8_t *cmdPtr;
            uint32_t id;                // CAN Node ID
            SNODE_CFG_VAL cfg_val;      // Configuration values.
            
            // NODE IDENTIFIER /////////////////////////////////////////////////
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"id");
            id = atoi((char*)cmdPtr);
            
            // PWM COEFFIENTS //////////////////////////////////////////////////
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x0pwm");
            cfg_val.pwm_coeff[0] = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x1pwm");
            cfg_val.pwm_coeff[1] = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x2pwm");
            cfg_val.pwm_coeff[2] = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x3pwm");
            cfg_val.pwm_coeff[3] = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x4pwm");
            cfg_val.pwm_coeff[4] = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x5pwm");
            cfg_val.pwm_coeff[5] = atoi((char*)cmdPtr);
            
            // VSENSE1 COEFFICIENTS ////////////////////////////////////////////
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x0vs1");
            cfg_val.vsense1_coeff[0] = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x1vs1");
            cfg_val.vsense1_coeff[1] = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x2vs1");
            cfg_val.vsense1_coeff[2] = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x3vs1");
            cfg_val.vsense1_coeff[3] = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x4vs1");
            cfg_val.vsense1_coeff[4] = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x5vs1");
            cfg_val.vsense1_coeff[5] = atoi((char*)cmdPtr);
            
            // VSENSE2 COEFFICIENTS ////////////////////////////////////////////
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x0vs2");
            cfg_val.vsense2_coeff[0] = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x1vs2");
            cfg_val.vsense2_coeff[1] = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x2vs2");
            cfg_val.vsense2_coeff[2] = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x3vs2");
            cfg_val.vsense2_coeff[3] = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x4vs2");
            cfg_val.vsense2_coeff[4] = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"x5vs2");
            cfg_val.vsense2_coeff[5] = atoi((char*)cmdPtr);
            
            // Set the coefficients to be programmed.
            SNodeCalWriteSet( id, &cfg_val );
        }
        else if (strcmp((const char*)ptr, "calRead") == 0)
        {
            uint8_t *cmdPtr;
            uint8_t id;
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"id");
            id = atoi((char*)cmdPtr);
            
            SNodeCalReadSet( id );
        }
        else if (strcmp((const char*)ptr, "sendTestValue") == 0)
        {
            uint8_t *cmdPtr;
            uint32_t id;        // CAN Node ID
            int32_t value;      // Value
            uint32_t unit;      // 0 = PWM, 1 = rad, 2 = deg
            
            CAN_TX_SERVO_CMD_U can_ctrl_cmd;
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"id");
            id = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"value");
            value = atoi((char*)cmdPtr);
            
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"unit");
            unit = atoi((char*)cmdPtr);
            
            if(unit == 0) // PWM
            {
                can_ctrl_cmd.cmd_type = 0;      // PWM control
                can_ctrl_cmd.cmd_pwm  = value;
            }
            else
            if(unit == 1) // rad
            {
                can_ctrl_cmd.cmd_type = 1;      // Position control (rad)
                can_ctrl_cmd.cmd_pos  = value;
            }
            else
            if(unit == 2) // deg
            {
                can_ctrl_cmd.cmd_type = 1;      // Position control (rad)
                
                // convert value from degrees to milli-radians.
                can_ctrl_cmd.cmd_pos = ((value * 174533) / 10000);
            }
            else
            {
                Nop();  // do nothing invalid 'unit' value.
            }
            
            // Sent the command to the servo-node.
            CANTxSet( CAN_TX_MSG_SERVO_CMD, 
                      id,
                      &can_ctrl_cmd.data_u32[ 0 ] );
        }
    }

    return HTTP_IO_DONE;
}

/****************************************************************************
  Section:
	POST Form Handlers
  ***************************************************************************/

/*****************************************************************************
  Function:
	static HTTP_IO_RESULT HTTPPostSaveFMUConfig(void)

  Description:
	This function has four states.  The first reads a name from the data
	string returned as part of the POST request.  If a name cannot
	be found, it returns, asking for more data.  Otherwise, if the name
	is expected, it reads the associated value. If the name is not expected, 
    the value is discarded and the next name parameter is read.

	In the case where the expected string is never found, this function
	will eventually return HTTP_IO_NEED_DATA when no data is left.  In that
	case, the HTTP2 server will automatically trap the error and issue an
	Internal Server Error to the browser.

  Precondition:
	None

  Parameters:
	None

  Return Values:
  	HTTP_IO_DONE - the parameter has been found and saved
  	HTTP_IO_WAITING - the function is pausing to continue later
  	HTTP_IO_NEED_DATA - data needed by this function has not yet arrived
  ***************************************************************************/

static HTTP_IO_RESULT HTTPPostSaveFMUConfig(void)
{
    static void* cDest;
    char value[HTTP_MAX_DATA_LEN];

    #define SM_POST_CONFIG_READ_NAME		(0u)
    #define SM_POST_CONFIG_READ_VALUE		(1u)
    #define SM_POST_FORM_SUBMIT_TYPE        (2u)
    
    switch (curHTTP.smPost)
    {
        // Find the name
        case SM_POST_CONFIG_READ_NAME:
        {
            // Read a name
            if (HTTPReadPostName(curHTTP.data, HTTP_MAX_DATA_LEN) == HTTP_READ_INCOMPLETE)
            {
                return HTTP_IO_NEED_DATA;
            }

            // Submit Button Identifier
            else if (!strcmppgm2ram((char*)curHTTP.data, (ROM char*)"formSubmitType"))
            {
                curHTTP.smPost = SM_POST_FORM_SUBMIT_TYPE;
                break;
            }

            // Unknown Configuration Name
            else
            {
                cDest = NULL;
            }

            curHTTP.smPost = SM_POST_CONFIG_READ_VALUE;
        }
        case SM_POST_CONFIG_READ_VALUE:
        {
            // Read a value string.
            if (HTTPReadPostValue((uint8_t*)value, HTTP_MAX_DATA_LEN) == HTTP_READ_INCOMPLETE)
            {
                return HTTP_IO_NEED_DATA;
            }

            // Look for a new name if this was an unexpected name/value.
            // Value will be discarded.
            if (!cDest)
            {
                curHTTP.smPost = SM_POST_CONFIG_READ_NAME;
                break;
            }

            // Store the values to RAM.
            
            
            curHTTP.smPost = SM_POST_CONFIG_READ_NAME;
            break;
        }
        case SM_POST_FORM_SUBMIT_TYPE:
        {
            // Read a value string.
            if (HTTPReadPostValue((uint8_t*)value, HTTP_MAX_DATA_LEN) == HTTP_READ_INCOMPLETE)
            {
                return HTTP_IO_NEED_DATA;
            }

            return HTTP_IO_DONE;
        }
    }

    // Default assumes that we're returning for state machine convenience.
    // Function will be called again later.
    return HTTP_IO_WAITING;
}

/*****************************************************************************
  Function:
	HTTP_IO_RESULT HTTPExecutePost(void)

  Internal:
  	See documentation in the TCP/IP Stack API or HTTP2.h for details.
  ***************************************************************************/

HTTP_IO_RESULT HTTPExecutePost(void)
{
    // Resolve which function to use and pass along
    BYTE filename[20];

    // Load the file name
    // Make sure BYTE filename[] above is large enough for your longest name
    MPFSGetFilename(curHTTP.file, filename, sizeof(filename));

    if(!memcmppgm2ram(filename, "config.htm", 10))
    {
        return HTTPPostSaveFMUConfig();
    }

    return HTTP_IO_DONE;
}
