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

#include <stdbool.h>
#include <stdio.h>
#include "system_definitions.h"
#include "app.h"
#include "uart_handler.h"
#include "FastTransfer.h"
#include "LidarDecoder.h"
#include "timers.h"
#include "lidarCalibrate.h"
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

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
 */
static timer_t configDelay, ms100, secondTimer, spindown, threesecond, foursecond;
timer_t lidar_runtime;

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

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    InitUARTModule(&LidarUart, UART_Lidar);
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

#define MASTER

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */


void APP_Tasks(void) {

    /* Check the application's current state. */
    switch (appData.state) {
            /* Application's initial state. */
        case APP_STATE_INIT:
        {

            if (appInitialized) {
                DRV_OCO_Change_PulseWidth(1800);

                appData.state = APP_STATE_STOP_LIDAR;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            appData.state = APP_STATE_LIDAR;
            break;
        }

        case APP_STATE_STOP_LIDAR:
        {
            DRV_OCO_Change_PulseWidth(0);

            //---------------------Wait a bit-------------------------
            resetTimer(&spindown);
            while (!timerDone(&spindown));

            appData.state++;
            break;
        }
        case APP_STATE_CONFIGURATION_SETUP:
        {

            sweepConfigSettings();

            //--------------Turn on motor--------------
            DRV_OCO_Change_PulseWidth(1800);

            //-------Wait for spin up--------
            resetTimer(&ms100);
            while (!timerDone(&ms100));

            appData.state = APP_STATE_LIDAR;
            break;
        }
        case APP_STATE_LIDAR:
        {
            resetTimer(&lidar_runtime);
            while (appData.state == APP_STATE_LIDAR) {
                if (timerDone(&secondTimer)) {
                    LED6 ^= 1;
                }
                decode_LidarData();
            }

            break;
        }
        case FASTTRANS_TEST:
        {
            if (timerDone(&ms100)) {
                ToSend(0, 0xFFFF, &OutGoing_DataTransBuff);
                sendData(FastTransTestCounter, &OutGoing_DataTransBuff);


                if (FastTransTestCounter > 10) {
                    FastTransTestCounter = 0;
                }
            }
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
