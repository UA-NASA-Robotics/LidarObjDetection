/*******************************************************************************
  System Initialization File

  File Name:
    system_init.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, defines the configuration bits,
    and allocates any necessary global system resources, such as the
    sysObj structure that contains the object handles to all the MPLAB Harmony
    module objects in the system.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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

#include "system_config.h"
#include "system_definitions.h"


// ****************************************************************************
// ****************************************************************************
// Section: Configuration Bits
// ****************************************************************************
// ****************************************************************************
// <editor-fold defaultstate="collapsed" desc="Configuration Bits">

/*** DEVCFG0 ***/

#pragma config DEBUG =      OFF
#pragma config JTAGEN =     OFF
#pragma config ICESEL =     ICS_PGx1
#pragma config TRCEN =      OFF
#pragma config BOOTISA =    MIPS32
#pragma config FECCCON =    OFF_UNLOCKED
#pragma config FSLEEP =     OFF
#pragma config DBGPER =     PG_ALL
#pragma config SMCLR =      MCLR_NORM
#pragma config SOSCGAIN =   GAIN_2X
#pragma config SOSCBOOST =  OFF
#pragma config POSCGAIN =   GAIN_2X
#pragma config POSCBOOST =  OFF
#pragma config EJTAGBEN =   NORMAL
#pragma config CP =         OFF

/*** DEVCFG1 ***/

#pragma config FNOSC =      SPLL
#pragma config DMTINTV =    WIN_127_128
#pragma config FSOSCEN =    OFF
#pragma config IESO =       OFF
#pragma config POSCMOD =    EC
#pragma config OSCIOFNC =   OFF
#pragma config FCKSM =      CSDCMD
#pragma config WDTPS =      PS1048576
#pragma config WDTSPGM =    STOP
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     NORMAL
#pragma config FWDTWINSZ =  WINSZ_25
#pragma config DMTCNT =     DMT31
#pragma config FDMTEN =     OFF
/*** DEVCFG2 ***/

#pragma config FPLLIDIV =   DIV_3
#pragma config FPLLRNG =    RANGE_5_10_MHZ
#pragma config FPLLICLK =   PLL_POSC
#pragma config FPLLMULT =   MUL_50
#pragma config FPLLODIV =   DIV_2
#pragma config UPLLFSEL =   FREQ_24MHZ
/*** DEVCFG3 ***/

#pragma config USERID =     0xffff
#pragma config FMIIEN =     ON
#pragma config FETHIO =     ON
#pragma config PGL1WAY =    ON
#pragma config PMDL1WAY =   ON
#pragma config IOL1WAY =    ON
#pragma config FUSBIDIO =   ON

/*** BF1SEQ0 ***/

#pragma config TSEQ =       0x0000
#pragma config CSEQ =       0xffff
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: Driver Initialization Data
// *****************************************************************************
// *****************************************************************************
// <editor-fold defaultstate="collapsed" desc="DRV_Timer Initialization Data">
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="DRV_USART Initialization Data">

const DRV_USART_INIT drvUsart0InitData =
{
    .moduleInit.value = DRV_USART_POWER_STATE_IDX0,
    .usartID = DRV_USART_PERIPHERAL_ID_IDX0, 
    .mode = DRV_USART_OPER_MODE_IDX0,
    .flags = DRV_USART_INIT_FLAGS_IDX0,
    .brgClock = DRV_USART_BRG_CLOCK_IDX0,
    .lineControl = DRV_USART_LINE_CNTRL_IDX0,
    .baud = DRV_USART_BAUD_RATE_IDX0,
    .handshake = DRV_USART_HANDSHAKE_MODE_IDX0,
    .linesEnable = DRV_USART_LINES_ENABLE_IDX0,
    .interruptTransmit = DRV_USART_XMIT_INT_SRC_IDX0,
    .interruptReceive = DRV_USART_RCV_INT_SRC_IDX0,
    .interruptError = DRV_USART_ERR_INT_SRC_IDX0,
    .dmaChannelTransmit = DMA_CHANNEL_NONE,
    .dmaInterruptTransmit = DRV_USART_XMIT_INT_SRC_IDX0,    
    .dmaChannelReceive = DMA_CHANNEL_NONE,
    .dmaInterruptReceive = DRV_USART_RCV_INT_SRC_IDX0,    
};

const DRV_USART_INIT drvUsart1InitData =
{
    .moduleInit.value = DRV_USART_POWER_STATE_IDX1,
    .usartID = DRV_USART_PERIPHERAL_ID_IDX1, 
    .mode = DRV_USART_OPER_MODE_IDX1,
    .flags = DRV_USART_INIT_FLAGS_IDX1,
    .brgClock = DRV_USART_BRG_CLOCK_IDX1,
    .lineControl = DRV_USART_LINE_CNTRL_IDX1,
    .baud = DRV_USART_BAUD_RATE_IDX1,
    .handshake = DRV_USART_HANDSHAKE_MODE_IDX1,
    .linesEnable = DRV_USART_LINES_ENABLE_IDX1,
    .interruptTransmit = DRV_USART_XMIT_INT_SRC_IDX1,
    .interruptReceive = DRV_USART_RCV_INT_SRC_IDX1,
    .interruptError = DRV_USART_ERR_INT_SRC_IDX1,
    .dmaChannelTransmit = DMA_CHANNEL_NONE,
    .dmaInterruptTransmit = DRV_USART_XMIT_INT_SRC_IDX1,
    .dmaChannelReceive = DMA_CHANNEL_NONE,
    .dmaInterruptReceive= DRV_USART_RCV_INT_SRC_IDX1,
};

const DRV_USART_INIT drvUsart2InitData =
{
    .moduleInit.value = DRV_USART_POWER_STATE_IDX2,
    .usartID = DRV_USART_PERIPHERAL_ID_IDX2, 
    .mode = DRV_USART_OPER_MODE_IDX2,
    .flags = DRV_USART_INIT_FLAGS_IDX2,
    .brgClock = DRV_USART_BRG_CLOCK_IDX2,
    .lineControl = DRV_USART_LINE_CNTRL_IDX2,
    .baud = DRV_USART_BAUD_RATE_IDX2,
    .handshake = DRV_USART_HANDSHAKE_MODE_IDX2,
    .linesEnable = DRV_USART_LINES_ENABLE_IDX2,
    .interruptTransmit = DRV_USART_XMIT_INT_SRC_IDX2,
    .interruptReceive = DRV_USART_RCV_INT_SRC_IDX2,
    .interruptError = DRV_USART_ERR_INT_SRC_IDX2,
    .dmaChannelTransmit = DMA_CHANNEL_NONE,
    .dmaInterruptTransmit = DRV_USART_XMIT_INT_SRC_IDX2,
    .dmaChannelReceive = DMA_CHANNEL_NONE,
    .dmaInterruptReceive = DRV_USART_RCV_INT_SRC_IDX2,
};

const DRV_USART_INIT drvUsart3InitData =
{
    .moduleInit.value = DRV_USART_POWER_STATE_IDX3,
    .usartID = DRV_USART_PERIPHERAL_ID_IDX3, 
    .mode = DRV_USART_OPER_MODE_IDX3,
    .flags = DRV_USART_INIT_FLAGS_IDX3,
    .brgClock = DRV_USART_BRG_CLOCK_IDX3,
    .lineControl = DRV_USART_LINE_CNTRL_IDX3,
    .baud = DRV_USART_BAUD_RATE_IDX3,
    .handshake = DRV_USART_HANDSHAKE_MODE_IDX3,
    .linesEnable = DRV_USART_LINES_ENABLE_IDX3,
    .interruptTransmit = DRV_USART_XMIT_INT_SRC_IDX3,
    .interruptReceive = DRV_USART_RCV_INT_SRC_IDX3,
    .interruptError = DRV_USART_ERR_INT_SRC_IDX3,
    .dmaChannelTransmit = DMA_CHANNEL_NONE,
    .dmaInterruptTransmit = DRV_USART_XMIT_INT_SRC_IDX3,
    .dmaChannelReceive = DMA_CHANNEL_NONE,
    .dmaInterruptReceive = DRV_USART_RCV_INT_SRC_IDX3,
};

const DRV_USART_INIT drvUsart4InitData =
{
    .moduleInit.value = DRV_USART_POWER_STATE_IDX4,
    .usartID = DRV_USART_PERIPHERAL_ID_IDX4, 
    .mode = DRV_USART_OPER_MODE_IDX4,
    .flags = DRV_USART_INIT_FLAGS_IDX4,
    .brgClock = DRV_USART_BRG_CLOCK_IDX4,
    .lineControl = DRV_USART_LINE_CNTRL_IDX4,
    .baud = DRV_USART_BAUD_RATE_IDX4,
    .handshake = DRV_USART_HANDSHAKE_MODE_IDX4,
    .linesEnable = DRV_USART_LINES_ENABLE_IDX4,
    .interruptTransmit = DRV_USART_XMIT_INT_SRC_IDX4,
    .interruptReceive = DRV_USART_RCV_INT_SRC_IDX4,
    .interruptError = DRV_USART_ERR_INT_SRC_IDX4,
    .dmaChannelTransmit = DMA_CHANNEL_NONE,
    .dmaInterruptTransmit = DRV_USART_XMIT_INT_SRC_IDX4,
    .dmaChannelReceive = DMA_CHANNEL_NONE,
    .dmaInterruptReceive = DRV_USART_RCV_INT_SRC_IDX4,
};
// </editor-fold>

// *****************************************************************************
// *****************************************************************************
// Section: System Data
// *****************************************************************************
// *****************************************************************************

/* Structure to hold the object handles for the modules in the system. */
SYSTEM_OBJECTS sysObj;

// *****************************************************************************
// *****************************************************************************
// Section: Module Initialization Data
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Library/Stack Initialization Data
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: System Initialization
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Initialize ( void *data )

  Summary:
    Initializes the board, services, drivers, application and other modules.

  Remarks:
    See prototype in system/common/sys_module.h.
 */

void SYS_Initialize ( void* data )
{
    /* Core Processor Initialization */
    SYS_CLK_Initialize( NULL );
    SYS_DEVCON_Initialize(SYS_DEVCON_INDEX_0, (SYS_MODULE_INIT*)NULL);
    SYS_DEVCON_PerformanceConfig(SYS_CLK_SystemFrequencyGet());
    SYS_PORTS_Initialize();

    /* Initialize Drivers */
    /* Initialize the OC Driver */
    DRV_OC0_Initialize();
    /*Initialize TMR0 */
    DRV_TMR0_Initialize();
    /*Initialize TMR1 */
    DRV_TMR1_Initialize();
 
     sysObj.drvUsart0 = DRV_USART_Initialize(DRV_USART_INDEX_0, (SYS_MODULE_INIT *)&drvUsart0InitData);
    sysObj.drvUsart1 = DRV_USART_Initialize(DRV_USART_INDEX_1, (SYS_MODULE_INIT *)&drvUsart1InitData);
    sysObj.drvUsart2 = DRV_USART_Initialize(DRV_USART_INDEX_2, (SYS_MODULE_INIT *)&drvUsart2InitData);
    sysObj.drvUsart3 = DRV_USART_Initialize(DRV_USART_INDEX_3, (SYS_MODULE_INIT *)&drvUsart3InitData);
    sysObj.drvUsart4 = DRV_USART_Initialize(DRV_USART_INDEX_4, (SYS_MODULE_INIT *)&drvUsart4InitData);
    SYS_INT_VectorPrioritySet(INT_VECTOR_UART1_TX, INT_PRIORITY_LEVEL5);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_UART1_TX, INT_SUBPRIORITY_LEVEL2);
    SYS_INT_VectorPrioritySet(INT_VECTOR_UART1_RX, INT_PRIORITY_LEVEL6);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_UART1_RX, INT_SUBPRIORITY_LEVEL2);
    SYS_INT_VectorPrioritySet(INT_VECTOR_UART1_FAULT, INT_PRIORITY_LEVEL6);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_UART1_FAULT, INT_SUBPRIORITY_LEVEL2);
    SYS_INT_VectorPrioritySet(INT_VECTOR_UART6_TX, INT_DISABLE_INTERRUPT);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_UART6_TX, INT_SUBPRIORITY_LEVEL3);
    SYS_INT_VectorPrioritySet(INT_VECTOR_UART6_RX, INT_PRIORITY_LEVEL7);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_UART6_RX, INT_SUBPRIORITY_LEVEL3);
    SYS_INT_VectorPrioritySet(INT_VECTOR_UART6_FAULT, INT_PRIORITY_LEVEL7);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_UART6_FAULT, INT_SUBPRIORITY_LEVEL3);
    SYS_INT_VectorPrioritySet(INT_VECTOR_UART4_TX, INT_PRIORITY_LEVEL6);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_UART4_TX, INT_SUBPRIORITY_LEVEL2);
    SYS_INT_VectorPrioritySet(INT_VECTOR_UART4_RX, INT_PRIORITY_LEVEL6);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_UART4_RX, INT_SUBPRIORITY_LEVEL2);
    SYS_INT_VectorPrioritySet(INT_VECTOR_UART4_FAULT, INT_PRIORITY_LEVEL6);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_UART4_FAULT, INT_SUBPRIORITY_LEVEL2);
    SYS_INT_VectorPrioritySet(INT_VECTOR_UART5_TX, INT_PRIORITY_LEVEL6);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_UART5_TX, INT_SUBPRIORITY_LEVEL1);
    SYS_INT_VectorPrioritySet(INT_VECTOR_UART5_RX, INT_DISABLE_INTERRUPT);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_UART5_RX, INT_SUBPRIORITY_LEVEL0);
    SYS_INT_VectorPrioritySet(INT_VECTOR_UART5_FAULT, INT_DISABLE_INTERRUPT);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_UART5_FAULT, INT_SUBPRIORITY_LEVEL0);
    SYS_INT_VectorPrioritySet(INT_VECTOR_UART3_TX, INT_PRIORITY_LEVEL5);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_UART3_TX, INT_SUBPRIORITY_LEVEL0);
    SYS_INT_VectorPrioritySet(INT_VECTOR_UART3_RX, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_UART3_RX, INT_SUBPRIORITY_LEVEL0);
    SYS_INT_VectorPrioritySet(INT_VECTOR_UART3_FAULT, INT_PRIORITY_LEVEL1);
    SYS_INT_VectorSubprioritySet(INT_VECTOR_UART3_FAULT, INT_SUBPRIORITY_LEVEL0);

    /* Initialize System Services */

    /*** Interrupt Service Initialization Code ***/
    SYS_INT_Initialize();

    /* Initialize Middleware */

    /* Enable Global Interrupts */
    SYS_INT_Enable();

    /* Initialize the Application */
    APP_Initialize();
}


/*******************************************************************************
 End of File
*/

