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
#include "Definitions.h"
#include "app.h"
#include "uart_handler.h"
#include "lidarCalibrate.h"
#include "FastTransfer.h"
#include "LidarDecoder.h"
#include "timers.h"
#include "lidarCalibrate.h"
#include "Definitions.h"
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
    DRV_TMR0_Start();
    DRV_TMR1_Start();
    DRV_OC0_Start();
    InitUARTModule(&LidarUart, UART_Lidar);
    InitUARTModule(&LantronixUart, Lantronix);
   // InitFastTransferModule(&LantronixFT, Lantronix, MY_ADDRESS, Send_put, Buffer_Get, Buffer_Size, Buffer_Peek);
    setTimerInterval(&ms100,100);
    LED1 = On;while(!timerDone(&ms100));
    LED2 = On;while(!timerDone(&ms100));
    LED3 = On;while(!timerDone(&ms100));
    LED4 = On;while(!timerDone(&ms100));
    LED5 = On;while(!timerDone(&ms100));
    LED6 = On;while(!timerDone(&ms100));
    LED7 = On;while(!timerDone(&ms100));
    LED8 = On;while(!timerDone(&ms100));
        LED1 = On;while(!timerDone(&ms100));
    LED2 = Off;while(!timerDone(&ms100));
    LED3 = Off;while(!timerDone(&ms100));
    LED4 = Off;while(!timerDone(&ms100));
    LED5 = Off;while(!timerDone(&ms100));
    LED6 = Off;while(!timerDone(&ms100));
    LED7 = Off;while(!timerDone(&ms100));
    LED8 = Off;while(!timerDone(&ms100));
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


void APP_Tasks(void) {

    /* Check the application's current state. */
    switch (appData.state) {
            /* Application's initial state. */
        case APP_STATE_INIT:
        {

                DRV_OCO_Change_PulseWidth(1800);
                setTimerInterval(&secondTimer,1000);
                appData.state = APP_STATE_LIDAR;
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
            setTimerInterval(&spindown,1000);
            resetTimer(&spindown);
            while (!timerDone(&spindown));

            appData.state++;
            break;
        }
        case APP_STATE_CONFIGURATION_SETUP:
        {

           // sweepConfigSettings();

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
            int obj[4];
            int i;
            while (appData.state == APP_STATE_LIDAR) {
                if (timerDone(&secondTimer)) {
                    LED6 ^= 1;
                    int num = runGroundObjectDetection(obj, sizeof(obj));
                    for(i = 0; i < num; i++)
                    {
                        printf("obj%d: %d\n",i,obj[i]);
                    }
                    
                }
                decode_LidarData();
                
                
            }

            break;
        }
        case FASTTRANS_TEST:
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
