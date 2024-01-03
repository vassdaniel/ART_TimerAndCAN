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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "peripheral/tmr/plib_tmr2.h"
#include "peripheral/gpio/plib_gpio.h"
#include "peripheral/tmr/plib_tmr4.h"
#include "system/time/sys_time.h"
#include "peripheral/can/plib_can1.h"

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

SYS_TIME_HANDLE TimerClients;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void APP_TimerCallback(uint32_t status, uintptr_t context)
{
    if(context == 2)
    {
        TMR2_GPIO_Toggle();
    }
    else if(context == 4)
    {
        TMR4_GPIO_Toggle();
    }
}

void APP_SysTimerCallback ( uintptr_t context )
{
    TIME_GPIO_Toggle();
}

void APP_CanReceiveCallback (uintptr_t contextHandle)
{
    uint32_t id;
    uint8_t length;
    uint8_t data[8];
    uint16_t timestamp;
    CAN_MSG_RX_ATTRIBUTE msgAttr;
    
    if(CAN1_MessageReceive(&id, &length, data, &timestamp, &msgAttr) == true)
    {
        switch(id)
        {
            case 0x77A:
                //Process bytes
                break;
            case 0x77B:
                
                break;
            default:
                ;
        }
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

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



    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
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
            bool appInitialized = true;
            
            TMR2_CallbackRegister(APP_TimerCallback, 2);  
            TMR4_CallbackRegister(APP_TimerCallback, 4);
            
            TMR2_Start();
            TMR4_Start();
            
            TimerClients = SYS_TIME_CallbackRegisterMS(APP_SysTimerCallback, NULL, 100, SYS_TIME_PERIODIC);
            
            //SYS_TIME_TimerDestroy(TimerClients);
            
            CAN1_CallbackRegister(APP_CanReceiveCallback, NULL, 1);
            
            uint8_t txData[8] = {1, 2, 3, 4, 5, 6, 7, 8};
            
            if(CAN1_MessageTransmit(0x77A, 8, txData, 0, CAN_MSG_TX_DATA_FRAME) == true)
            {
                ;
            }
            else
            {
                //Error handling
            }
            
            if (appInitialized)
            {

                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
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
