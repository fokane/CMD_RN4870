/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "system/debug/sys_debug.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
DRV_HANDLE usartDriverHandle;
DRV_USART_BUFFER_HANDLE usartBufferHandle;
uint8_t usartTxBuffer[USART_TX_BUFFER_SIZE_BYTES];
uint8_t usartRxBuffer[USART_RX_BUFFER_SIZE_BYTES];
volatile uint8_t usartRxBufferIndex;
SYS_TMR_HANDLE sysTmrHandle;
const uint8_t bleMacAddress[] = "001122334455";
bool printAsHex;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void APP_USARTReceiveEventHandler(const SYS_MODULE_INDEX index)
{
    // Byte has been Received. Handle the event.
    // Read byte using DRV_USART_ReadByte ()
    // DRV_USART_ReceiverBufferIsEmpty() function can be used to
    // check if the receiver buffer is empty.
    usartRxBuffer[usartRxBufferIndex++] = DRV_USART_ReadByte(usartDriverHandle);
    
    // check for bounds, reset to 0 if at the max
    if(usartRxBufferIndex == USART_RX_BUFFER_SIZE_BYTES)
    {
        usartRxBufferIndex = 0;
    }
}

void APP_USARTTransmitEventHandler (const SYS_MODULE_INDEX index)
{
    // Byte has been transmitted. Handle the event.
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
static int32_t _APP_Commands(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
int32_t _APP_Commands(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    if(argc < 2)
    {
        return true;
    }
    
    

    switch(*argv[1])
    {
        case '1':
        {
            appData.state = APP_STATE_ENTER_CMD_MODE;
            
            BSP_LEDToggle(BSP_LED_1);
            break;
        }
        case '2':
        {
            appData.state = APP_STATE_START_SCAN;
            
            BSP_LEDToggle(BSP_LED_2);
            break;
        }
        case '3':
        {
            appData.state = APP_STATE_STOP_SCAN;
            
            BSP_LEDToggle(BSP_LED_3);
            break;
        }
        case '4':
        {
            appData.state = APP_STATE_CONNECT;
            
            BSP_LEDToggle(BSP_LED_3);
            break;
        }
        default:
        {
            
            break;
        }
    }
    SYS_MESSAGE("Command.");
    SYS_MESSAGE("\n");
    
    return true;
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
static const SYS_CMD_DESCRIPTOR    appCmdTbl[]=
{
    {"cmd",        _APP_Commands,              ": BLE commands."}
};
/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    usartRxBufferIndex = 0;
    printAsHex = false;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    SYS_CMD_ADDGRP(appCmdTbl, sizeof(appCmdTbl)/sizeof(*appCmdTbl), "app", ": app commands");
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    uint8_t index;
    
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            // open USART driver
            usartDriverHandle = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
            
            // check for valid handle
            if (DRV_HANDLE_INVALID == usartDriverHandle)
            {
                // Unable to open the driver
                // May be the driver is not initialized or the initialization
                // is not complete.
                while(1);
            }
            else
            {
                // Register an event handler with driver. This is done once
                DRV_USART_ByteReceiveCallbackSet(DRV_USART_INDEX_0, APP_USARTReceiveEventHandler);

                // Register an event handler with driver. This is done once
                DRV_USART_ByteTransmitCallbackSet (DRV_USART_INDEX_0, APP_USARTTransmitEventHandler);


                // start CMD processor
                appData.state = APP_STATE_START_CMD_PROCESSOR;
            }
            
            

            break;
        }
        case APP_STATE_START_CMD_PROCESSOR:
        {
            if(SYS_CMD_READY_TO_READ() == true)
            {
                SYS_MESSAGE("Ready to accept command input.");
                SYS_MESSAGE("cmd [n]\r\n");
                SYS_MESSAGE("\t1 - enter cmd mode\r\n");
                SYS_MESSAGE("\t2 - start scan\r\n");
                SYS_MESSAGE("\t3 - stop scan\r\n");
                SYS_MESSAGE("\t4 - connect to stored MAC address\r\n");
                SYS_MESSAGE("\t5 - read last day of solar data\r\n");
                SYS_MESSAGE("\n\n");

                // change state
                appData.state = APP_STATE_IDLE;
            }
            break;
        }
        case APP_STATE_WAIT_1SECS:
        {
            SYS_MESSAGE("waiting 1 sec..\r\n");
            
            // start 1s timer
            sysTmrHandle = SYS_TMR_DelayMS(1000);

            // change state
            appData.state = APP_STATE_CHECK_TIMER;
            break;
        }
        case APP_STATE_WAIT_5SECS:
        {
            SYS_MESSAGE("waiting 5 secs..\r\n");
            
            // start 5s timer
            sysTmrHandle = SYS_TMR_DelayMS(5000);
            
            // change state
            appData.state = APP_STATE_CHECK_TIMER;
            break;
        }
        case APP_STATE_CHECK_TIMER:
        {
            if(SYS_TMR_DelayStatusGet(sysTmrHandle))
            {
                SYS_MESSAGE("time up\r\n");
                
                // print out whatever is in the receive buffer
                if(usartRxBuffer[0] == 0)
                {
                    SYS_MESSAGE("receive buffer empty\r\n");
                }
                else
                {
                    if(printAsHex == false)
                    {
                        SYS_CONSOLE_PRINT("%s\r\n", usartRxBuffer);
                    }
                    else
                    {
                        SYS_CONSOLE_PRINT("%x, ", usartRxBuffer[5]);
                        SYS_CONSOLE_PRINT("%x", usartRxBuffer[6]);
                        SYS_CONSOLE_PRINT("%x", usartRxBuffer[7]);
                        SYS_MESSAGE("\r\n");
                        printAsHex = false;
                    }
                    
                }
                
                
                // clear receive buffer
                memset(usartRxBuffer, 0, sizeof(usartRxBuffer));
            
                // back to idle state
                appData.state = APP_STATE_IDLE;
            }
            
            break;
        }
        case APP_STATE_ENTER_CMD_MODE:
        {
        
            SYS_MESSAGE("entering CMD mode..\r\n");
            
            // reset the read buffer index
            usartRxBufferIndex = 0;
            
            // enter command mode
            usartTxBuffer[0] = '$';
            usartTxBuffer[1] = '$';
            usartTxBuffer[2] = '$';
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[0]);
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[1]);
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[2]);
            
            appData.state = APP_STATE_WAIT_1SECS;
            break;
        }
        case APP_STATE_START_SCAN:
        {
            SYS_MESSAGE("start scan..\r\n");
            
            // reset the read buffer index
            usartRxBufferIndex = 0;
            
            // send scan command
            usartTxBuffer[0] = 'F';
            usartTxBuffer[1] = 0x0D;
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[0]);
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[1]);
        
            appData.state = APP_STATE_WAIT_5SECS;
            break;
        }
        case APP_STATE_STOP_SCAN:
        {
            SYS_MESSAGE("stop scan..\r\n");
            
            // reset the read buffer index
            usartRxBufferIndex = 0;
            
            // send stop scan command
            usartTxBuffer[0] = 'X';
            usartTxBuffer[1] = 0x0D;
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[0]);
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[1]);
            
            appData.state = APP_STATE_WAIT_1SECS;
            break;
        }
        case APP_STATE_CONNECT:
        {
            //SYS_MESSAGE("connecting..\r\n");
            SYS_CMD_PRINT("connecting to %s..\r\n", bleMacAddress);
            
            // reset the read buffer index
            usartRxBufferIndex = 0;
            
            // send stop scan command
            usartTxBuffer[0] = 'C';
            usartTxBuffer[1] = ',';
            usartTxBuffer[2] = '0';
            usartTxBuffer[3] = ',';
            usartTxBuffer[4] = bleMacAddress[0];
            usartTxBuffer[5] = bleMacAddress[1];
            usartTxBuffer[6] = bleMacAddress[2];
            usartTxBuffer[7] = bleMacAddress[3];
            usartTxBuffer[8] = bleMacAddress[4];
            usartTxBuffer[9] = bleMacAddress[5];
            usartTxBuffer[10] = bleMacAddress[6];
            usartTxBuffer[11] = bleMacAddress[7];
            usartTxBuffer[12] = bleMacAddress[8];
            usartTxBuffer[13] = bleMacAddress[9];
            usartTxBuffer[14] = bleMacAddress[10];
            usartTxBuffer[15] = bleMacAddress[11];
            
            for(index = 0; index < 16; index++)
            {
                // check transmitter is not full before writing
                while((DRV_USART_TRANSFER_STATUS_TRANSMIT_FULL & DRV_USART_TransferStatus(usartDriverHandle)));
                DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[index]);
            }
            
            
            appData.state = APP_STATE_WAIT_5SECS;

            break;
        }
        case APP_STATE_READ_SOLAR_DATA:
        {
            SYS_MESSAGE("reading solar data..\r\n");
            
            // reset the read buffer index
            usartRxBufferIndex = 0;
            
            // send command to solar inverter
            usartTxBuffer[0] = 0x11;
            usartTxBuffer[1] = 0x00;
            usartTxBuffer[2] = 0x00;
            usartTxBuffer[3] = 0x00;
            usartTxBuffer[4] = 0x9A;
            usartTxBuffer[5] = 0x01;
            usartTxBuffer[6] = 0x00;
            usartTxBuffer[7] = 0x00;
            usartTxBuffer[8] = 0xAC;
            /*
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[0]);
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[1]);
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[2]);
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[3]);
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[4]);
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[5]);
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[6]);
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[7]);
            */
            for(index = 0; index < 9; index++)
            {
                // check transmitter is not full before writing
                while((DRV_USART_TRANSFER_STATUS_TRANSMIT_FULL & DRV_USART_TransferStatus(usartDriverHandle)));
                DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[index]);
            }
            
            // need to print the solar data as hex, not ASCII
            printAsHex = true;
            
            appData.state = APP_STATE_WAIT_1SECS;
            break;
        }
        case APP_STATE_IDLE:
        {
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
