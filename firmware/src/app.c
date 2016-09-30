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
uint8_t usartRxBuffer[USART_TX_BUFFER_SIZE_BYTES];
volatile uint8_t usartRxBufferIndex;
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
static int32_t _APP_Commands_LED(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
int32_t _APP_Commands_LED(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    if(argc < 2)
    {
        return true;
    }
    
    

    switch(*argv[1])
    {
        case '1':
        {
            // reset the read buffer index
            usartRxBufferIndex = 0;
            
            // enter command mode
            usartTxBuffer[0] = '$';
            usartTxBuffer[1] = '$';
            usartTxBuffer[2] = '$';
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[0]);
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[1]);
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[2]);
            
            BSP_LEDToggle(BSP_LED_1);
            break;
        }
        case '2':
        {
            // reset the read buffer index
            usartRxBufferIndex = 0;
            
            // send scan command
            usartTxBuffer[0] = 'F';
            usartTxBuffer[1] = 0x0D;
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[0]);
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[1]);
            
            BSP_LEDToggle(BSP_LED_2);
            break;
        }
        case '3':
        {
            // reset the read buffer index
            usartRxBufferIndex = 0;
            
            // send stop scan command
            usartTxBuffer[0] = 'X';
            usartTxBuffer[1] = 0x0D;
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[0]);
            DRV_USART_WriteByte(usartDriverHandle, usartTxBuffer[1]);
            
            
            BSP_LEDToggle(BSP_LED_3);
            break;
        }
        default:
        {
            
            break;
        }
    }
    SYS_MESSAGE("LED command.");
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
    {"led",        _APP_Commands_LED,              ": Toggle LED."}
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
                SYS_MESSAGE("\n");

                // change state
                appData.state = APP_STATE_IDLE;
            }
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
