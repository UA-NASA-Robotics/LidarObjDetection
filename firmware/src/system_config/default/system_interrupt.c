/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/



// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <xc.h>
#include <sys/attribs.h>
#include "app.h"
#include "system_definitions.h"
#include "../src/uart_handler.h"
#include "FastTransfer.h"
// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

//set up to be a 10ms timer
void __ISR(_TIMER_2_VECTOR, ipl6AUTO) IntHandlerDrvTmrInstance0(void)
{
//    TimerTickFlag = true;    
//    TimerCounter+=10;
    globalTimerTracker();
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
    
}

//lantronix 
void __ISR(_UART1_TX_VECTOR, ipl5AUTO) _IntHandlerDrvUsartTransmitInstance0(void)
{
    /* Clear up the interrupt flag */
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_TRANSMIT);
}
void __ISR(_UART1_RX_VECTOR, ipl6AUTO) _IntHandlerDrvUsartReceiveInstance0(void)
{
    DRV_USART_TasksReceive(sysObj.drvUsart0);
    //SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_RECEIVE);
}

//LIDAR RECIEVE
void __ISR(_UART6_RX_VECTOR, ipl7AUTO) _IntHandlerDrvUsartReceiveInstance1(void)
{
    DRV_USART_TasksReceive(sysObj.drvUsart1);
    //LED8 = OFF;
}

 
void __ISR(_UART6_TX_VECTOR, ipl6AUTO) _IntHandlerDrvUsartTransmitInstance1(void)
{
    /* Clear up the interrupt flag */
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_6_TRANSMIT);
    //DRV_USART_TasksTransmit(sysObj.drvUsart3);
}

//FastTransfer_1 Transmitter UART module
void __ISR(_UART4_TX_VECTOR, ipl6AUTO) _IntHandlerDrvUsartTransmitInstance2(void)
{
   SYS_INT_SourceStatusClear(INT_SOURCE_USART_4_TRANSMIT);
}

void __ISR(_UART4_RX_VECTOR, ipl6AUTO) _IntHandlerDrvUsartReceiveInstance2(void)
{
    DRV_USART_TasksReceive(sysObj.drvUsart2);
}
void __ISR(_UART4_FAULT_VECTOR, ipl0AUTO) _IntHandlerDrvUsartErrorInstance2(void)
{
    DRV_USART_TasksError(sysObj.drvUsart2);
} 

 
void __ISR(_UART5_TX_VECTOR, ipl6AUTO) _IntHandlerDrvUsartTransmitInstance3(void)
{
    /* Clear up the interrupt flag */
    SYS_INT_SourceStatusClear(INT_SOURCE_USART_5_TRANSMIT);
    //DRV_USART_TasksTransmit(sysObj.drvUsart3);
}
void __ISR(_UART5_RX_VECTOR, ipl0AUTO) _IntHandlerDrvUsartReceiveInstance3(void)
{  
    DRV_USART_TasksReceive(sysObj.drvUsart3);
}
void __ISR(_UART5_FAULT_VECTOR, ipl0AUTO) _IntHandlerDrvUsartErrorInstance3(void)
{
    DRV_USART_TasksError(sysObj.drvUsart3);

}
 
 

 
void __ISR(_UART3_TX_VECTOR, ipl5AUTO) _IntHandlerDrvUsartTransmitInstance4(void)
{
    DRV_USART_TasksTransmit(sysObj.drvUsart4);
}
void __ISR(_UART3_RX_VECTOR, ipl1AUTO) _IntHandlerDrvUsartReceiveInstance4(void)
{
    DRV_USART_TasksReceive(sysObj.drvUsart4);
}
void __ISR(_UART3_FAULT_VECTOR, ipl1AUTO) _IntHandlerDrvUsartErrorInstance4(void)
{
    DRV_USART_TasksError(sysObj.drvUsart4);
}
 

     
   
  void __ISR(_TIMER_4_VECTOR, ipl0AUTO) IntHandlerDrvTmrInstance1(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_4);
}
 
/*******************************************************************************
 End of File
*/
