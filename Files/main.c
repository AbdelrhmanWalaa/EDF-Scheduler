/*
 * FreeRTOS Kernel V10.3.0
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"

#include <limits.h>
#include <semphr.h>
#include <string.h>

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )



static void prvSetupHardware( void );

/*-----------------------------------------------------------*/
/*-----------------------Macros------------------------------*/

#define BTN_1_PORT								PORT_0
#define BTN_1_PIN									PIN0
#define BTN_2_PORT								PORT_0
#define BTN_2_PIN									PIN1

#define BUTTON_1_PERIOD						50
#define BUTTON_2_PERIOD						50
#define PERIODIC_TASK_PERIOD			100
#define UART_RECEIVER_PERIOD			20
#define LOAD_1_SIM_PERIOD					10
#define LOAD_2_SIM_PERIOD					100

#define COUNTER_5_MS							37400
#define COUNTER_12_MS							89760

/*-----------------------Global Variables---------------------*/

int T1_In_Time, T1_Out_Time, T1_Total_Time;
int T2_In_Time, T2_Out_Time, T2_Total_Time;
int T3_In_Time, T3_Out_Time, T3_Total_Time;
int T4_In_Time, T4_Out_Time, T4_Total_Time;
int T5_In_Time, T5_Out_Time, T5_Total_Time;
int T6_In_Time, T6_Out_Time, T6_Total_Time;
int T7_In_Time, T7_Out_Time, T7_Total_Time;	

TickType_t CurrentSystemTime = NULL;
float Cpu_Load = NULL; 

/*-----------------Task Handlers------------------------------*/

TaskHandle_t Button1TMonitorHandler 		= NULL;
TaskHandle_t Button2TMonitorHandler 		= NULL;
TaskHandle_t PeriodicTransmitterHandler = NULL;
TaskHandle_t UartReceiverHandler 				= NULL;
TaskHandle_t Load1SimulationHandler 		= NULL;
TaskHandle_t Load2SimulationHandler			= NULL;

/*-----------------Queue Handler-------------------------------*/

QueueHandle_t EventQueue;

/*-----------------Tasks Implementation------------------------*/

void vApplicationIdleHook( void )
{
	static int TagInit = NULL;
	
	if( NULL == TagInit )
	{
		vTaskSetApplicationTaskTag( NULL, ( void * ) PIN8 );
		TagInit++;
	}
}

/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	GPIO_write( PORT_0, PIN9, PIN_IS_HIGH );
	GPIO_write( PORT_0, PIN9, PIN_IS_LOW );
	
	CurrentSystemTime = T1TC;
	
	Cpu_Load = ( ( float )( T1_Total_Time + T2_Total_Time + T3_Total_Time + T4_Total_Time + T5_Total_Time + T6_Total_Time ) / ( float ) CurrentSystemTime ) * 100;
}

/*-----------------------------------------------------------*/

void Button_1_Monitor( void *pvParameters ) 
{
	uint8_t u8_PressFlag = pdFALSE;
	uint8_t ButtonState;
	
	const char *eventRisingString = "\n Button 1 Rising Edge\n";
	const char *eventFallingString = "\n Button 1 Falling Edge\n";
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
		ButtonState = GPIO_read( BTN_1_PORT, BTN_1_PIN );
		
		if ( ( ButtonState == pdTRUE ) && ( u8_PressFlag == pdFALSE ) )
		{
			xQueueSend( EventQueue, &eventRisingString, portMAX_DELAY );
			u8_PressFlag = pdTRUE;
		} 
		else if ( ( ButtonState == pdFALSE ) && ( u8_PressFlag == pdTRUE ) )
		{
			xQueueSend( EventQueue, &eventFallingString, portMAX_DELAY );			
			u8_PressFlag = pdFALSE;
		}
		else
		{
			/* Do Nothing */
		}
			
		vTaskDelayUntil( &xLastWakeTime, BUTTON_1_PERIOD );
	}
}

/*--------------------------------------------------------------*/

void Button_2_Monitor( void *pvParameters )
{
	uint8_t u8_PressFlag = pdFALSE;
	uint8_t ButtonState;
	
	const char *eventRisingString = "\n Button 2 Rising Edge\n";
	const char *eventFallingString = "\n Button 2 Falling Edge\n";
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

  for(;;)
	{
		ButtonState = GPIO_read( BTN_2_PORT, BTN_2_PIN );
		
		if ( ( ButtonState == pdTRUE ) && ( u8_PressFlag == pdFALSE ) )
		{			
			xQueueSend(EventQueue, &eventRisingString, portMAX_DELAY);
			u8_PressFlag = pdTRUE;
		}
		else if ( ( ButtonState == pdFALSE ) && ( u8_PressFlag == pdTRUE ) )
		{			
			xQueueSend( EventQueue, &eventFallingString, portMAX_DELAY );			
			u8_PressFlag = pdFALSE;
		}
		else
		{
			/* Do Nothing */
		}
			
		vTaskDelayUntil( &xLastWakeTime, BUTTON_2_PERIOD );
	}
}

/*--------------------------------------------------------------*/

void Periodic_Transmitter( void *pvParameters )
{
	const char *eventString = "Periodic Event Task  ";
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for(;;) 
	{		
		xQueueSend( EventQueue, &eventString, portMAX_DELAY );
		
		vTaskDelayUntil( &xLastWakeTime, PERIODIC_TASK_PERIOD );
	}
}

/*--------------------------------------------------------------*/

void Uart_Receiver( void *pvParameters )
{
	const char *eventString;
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	for(;;) 
	{
		if ( xQueueReceive( EventQueue, &eventString, NULL ) ) 
		{
			vSerialPutString( ( const signed char* ) eventString, strlen( eventString ) );
		}
		
		vTaskDelayUntil( &xLastWakeTime, UART_RECEIVER_PERIOD );
	}
}

/*--------------------------------------------------------------*/

void Load_1_Simulation( void *pvParameters ) 
{
	uint32_t u32_Counter = NULL;

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for(;;) 
	{				
		for( u32_Counter = NULL; u32_Counter < COUNTER_5_MS; u32_Counter++ )
		{
			/*Heavy Load Simulation*/
		}
		
		vTaskDelayUntil( &xLastWakeTime,LOAD_1_SIM_PERIOD );
	}
}

/*--------------------------------------------------------------*/

void Load_2_Simulation( void *pvParameters )
{
	uint32_t u32_Counter = NULL;

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for(;;) 
	{
		for( u32_Counter = NULL; u32_Counter < COUNTER_12_MS; u32_Counter++ )
		{
			/*Heavy Load Simulation*/
		}
		
		vTaskDelayUntil( &xLastWakeTime, LOAD_2_SIM_PERIOD );
	}
}

/*--------------------------------------------------------------*/
/*--------------------------------------------------------------*/
/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	EventQueue = xQueueCreate( 10, sizeof( const char * ) );

    /* Create Tasks here */

		xTaskPeriodicCreate( Button_1_Monitor,
												 "Button1Monitor",
												 configMINIMAL_STACK_SIZE,
												 ( void * ) NULL,
												 1,
												 &Button1TMonitorHandler,
												 BUTTON_1_PERIOD );
								 
								 
		xTaskPeriodicCreate( Button_2_Monitor,
												 "Button2Monitor",
												 configMINIMAL_STACK_SIZE,
												 ( void * ) NULL,
												 1,
												 &Button2TMonitorHandler,
												 BUTTON_2_PERIOD );	
								 
								 
		xTaskPeriodicCreate( Periodic_Transmitter,
												 "PeriodicTransmitter",
												 configMINIMAL_STACK_SIZE,
												 ( void * ) NULL,
												 1,
												 &PeriodicTransmitterHandler,
												 PERIODIC_TASK_PERIOD );	
								 
								 
		xTaskPeriodicCreate( Uart_Receiver,
												 "UARTreceiver",
												 configMINIMAL_STACK_SIZE,
												 ( void * ) NULL,
												 1,
												 &UartReceiverHandler,
												 UART_RECEIVER_PERIOD );

								 
		xTaskPeriodicCreate( Load_1_Simulation,
												 "Load1Simulation",
												 configMINIMAL_STACK_SIZE,
												 ( void * ) NULL,
												 1,
												 &Load1SimulationHandler,
												 LOAD_1_SIM_PERIOD );
								 
								 
		xTaskPeriodicCreate( Load_2_Simulation,
												 "Load2Simulation",
												 configMINIMAL_STACK_SIZE,
												 ( void * ) NULL,
												 1,
												 &Load2SimulationHandler,
												 LOAD_2_SIM_PERIOD );


	vTaskSetApplicationTaskTag( Button1TMonitorHandler, ( void * ) PIN2 );
	vTaskSetApplicationTaskTag( Button2TMonitorHandler, ( void * ) PIN3 );
	vTaskSetApplicationTaskTag( PeriodicTransmitterHandler, ( void * ) PIN4 );
	vTaskSetApplicationTaskTag( UartReceiverHandler, ( void * ) PIN5 );
	vTaskSetApplicationTaskTag( Load1SimulationHandler, ( void * ) PIN6 );
	vTaskSetApplicationTaskTag( Load2SimulationHandler, ( void * ) PIN7 );

	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1( void )
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


