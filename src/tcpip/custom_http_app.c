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

void HTTPPrint_feedback()
{
    char msg[16];

    if (TCPIsPutReady(sktHTTP) < sizeof(msg))
    {
        curHTTP.callbackPos = 1;
        return;
    }

    sprintf(msg, "%u", 0);         // TODO: Provide actual feedback value.
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

        if (strcmp((const char*)ptr, "sendCommand") == 0)
        {
            uint8_t *cmdPtr;
            cmdPtr = HTTPGetROMArg(curHTTP.data, (ROM uint8_t *)"cmdString");
            
            float val;
            val = atof((char*)cmdPtr);
            
            Nop();      // TODO: Process command data.
            
            
            
            
            /// TEMP FOR DEBUG /////////////////////////////////////////////////
            uint16_t val_u16;
            val_u16 = (uint16_t) val;
            
            CAN_TX_SERVO_CMD_U cmd_msg;
            
            cmd_msg.cmd_type = 0;       // command PWM to be used.
            cmd_msg.cmd_pwm  = val_u16; // feed through value from ethernet to CAN interface.
            
            // Queue the CAN message for transmission.
            CANTxSet( CAN_TX_MSG_SERVO_CMD, 0x7F, cmd_msg.data_u32 );
            /// END TEMP FOR DEBUG /////////////////////////////////////////////
            
            
            
            
            
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
