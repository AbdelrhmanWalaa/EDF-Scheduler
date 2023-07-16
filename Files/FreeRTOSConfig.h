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

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <lpc21xx.h>
#include "GPIO.h"

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE. 
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION		1
#define configUSE_IDLE_HOOK			1
#define configUSE_TICK_HOOK			1
#define configCPU_CLOCK_HZ			( ( unsigned long ) 60000000 )	/* =12.0MHz xtal multiplied by 5 using the PLL. */
#define configTICK_RATE_HZ			( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES		( 4 )
#define configMINIMAL_STACK_SIZE	( ( unsigned short ) 200 )
#define configTOTAL_HEAP_SIZE		( ( size_t ) 13 * 1024 )
#define configMAX_TASK_NAME_LEN		( 8 )
#define configUSE_TRACE_FACILITY	0
#define configUSE_16_BIT_TICKS		0
#define configIDLE_SHOULD_YIELD		1
#define configUSE_EDF_SCHEDULER   1
#define configUSE_TIME_SLICING 		1
#define configUSE_APPLICATION_TASK_TAG	1

#define configQUEUE_REGISTRY_SIZE 	0

#define configIDLE_TSK_PERIOD				200

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 		0
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		1
#define INCLUDE_vTaskDelete				1
#define INCLUDE_vTaskCleanUpResources	0
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay				1


#define traceTASK_SWITCHED_IN(){																																					\
																	if( PIN2 == (int)pxCurrentTCB->pxTaskTag )  														\
																	{																																				\
																		T1_In_Time = T1TC;																										\
																	}																																				\
																	else if( PIN3 == (int)pxCurrentTCB->pxTaskTag )													\
																	{																																				\
																		T2_In_Time = T1TC;																										\
																	}																																				\
																	else if( PIN4 == (int)pxCurrentTCB->pxTaskTag )													\
																	{																																				\
																		T3_In_Time = T1TC;																										\
																	}																																				\
																	else if( PIN5 == (int)pxCurrentTCB->pxTaskTag )													\
																	{																																				\
																		T4_In_Time = T1TC;																										\
																	}																																				\
																	else if( PIN6 == (int)pxCurrentTCB->pxTaskTag )													\
																	{																																				\
																		T5_In_Time = T1TC;																										\
																	}																																				\
																	else if( PIN7 == (int)pxCurrentTCB->pxTaskTag )													\
																	{																																				\
																		T6_In_Time = T1TC;																										\
																	}																																				\
																	else{}																																	\
																	GPIO_write(PORT_0, (int)pxCurrentTCB->pxTaskTag, PIN_IS_HIGH );					\
																}																																					\

																
#define traceTASK_SWITCHED_OUT(){																																					\
																	if( PIN2 == (int)pxCurrentTCB->pxTaskTag )  														\
																	{																																				\
																		T1_Out_Time = T1TC;																										\
																		T1_Total_Time += T1_Out_Time - T1_In_Time;														\
																	}																																				\
																	else if( PIN3 == (int)pxCurrentTCB->pxTaskTag )													\
																	{																																				\
																		T2_Out_Time = T1TC;																										\
																		T2_Total_Time += T2_Out_Time - T2_In_Time;														\
																	}																																				\
																	else if( PIN4 == (int)pxCurrentTCB->pxTaskTag )													\
																	{																																				\
																		T3_Out_Time = T1TC;																										\
																		T3_Total_Time += T3_Out_Time - T3_In_Time;														\
																	}																																				\
																	else if( PIN5 == (int)pxCurrentTCB->pxTaskTag )													\
																	{																																				\
																		T4_Out_Time = T1TC;																										\
																		T4_Total_Time += T4_Out_Time - T4_In_Time;														\
																	}																																				\
																	else if( PIN6 == (int)pxCurrentTCB->pxTaskTag )													\
																	{																																				\
																		T5_Out_Time = T1TC;																										\
																		T5_Total_Time += T5_Out_Time - T5_In_Time;														\
																	}																																				\
																	else if( PIN7 == (int)pxCurrentTCB->pxTaskTag )													\
																	{																																				\
																		T6_Out_Time = T1TC;																										\
																		T6_Total_Time += T6_Out_Time - T6_In_Time;														\
																	}																																				\
																	else{}																																	\
																	GPIO_write(PORT_0, (int)pxCurrentTCB->pxTaskTag, PIN_IS_LOW );					\
																}																																					\

extern int T1_In_Time , T1_Out_Time , T1_Total_Time ;
extern int T2_In_Time , T2_Out_Time , T2_Total_Time ;
extern int T3_In_Time , T3_Out_Time , T3_Total_Time ;
extern int T4_In_Time , T4_Out_Time , T4_Total_Time ;
extern int T5_In_Time , T5_Out_Time , T5_Total_Time ;
extern int T6_In_Time , T6_Out_Time , T6_Total_Time ;																
extern int T7_In_Time , T7_Out_Time , T7_Total_Time ;																		
																
#endif /* FREERTOS_CONFIG_H */
