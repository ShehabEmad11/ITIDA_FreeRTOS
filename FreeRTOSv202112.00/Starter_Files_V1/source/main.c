/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */



//Not to be Null if we want handler return
TaskHandle_t task1Handler= NULL;
#define TASK1_PRIORITY				(1u)
#define TASK1_NAME					("Button_1_Monitor")
#define TASK1_STACK_SIZE_WORDS		(100u)
#define TASK1_EXECUTION_TIME 		(1u) //cpu time in ticks
#define TASK1_PERIODICITY 			(50u)


//Not to be Null if we want handler return
TaskHandle_t task2Handler= NULL;
#define TASK2_PRIORITY				(1u)
#define TASK2_NAME					("Button_2_Monitor")
#define TASK2_STACK_SIZE_WORDS		(100u)
#define TASK2_EXECUTION_TIME 		(1u) //cpu time in ticks
#define TASK2_PERIODICITY 			(50u)


//Not to be Null if we want handler return
TaskHandle_t task3Handler= NULL;
#define TASK3_PRIORITY				(1u)
#define TASK3_NAME					("Periodic_Transmitter")
#define TASK3_STACK_SIZE_WORDS		(100u)
#define TASK3_EXECUTION_TIME 		(1u) //cpu time in ticks
#define TASK3_PERIODICITY 			(100u)


//Not to be Null if we want handler return
TaskHandle_t task4Handler= NULL;
#define TASK4_PRIORITY				(1u)
#define TASK4_NAME					("Uart_Receiver")
#define TASK4_STACK_SIZE_WORDS		(100u)
#define TASK4_EXECUTION_TIME 		(1u) //cpu time in ticks
#define TASK4_PERIODICITY 			(20u)


//Not to be Null if we want handler return
TaskHandle_t task5Handler= NULL;
#define TASK5_PRIORITY				(1u)
#define TASK5_NAME					("Load_1_Simulation")
#define TASK5_STACK_SIZE_WORDS		(100u)
#define TASK5_EXECUTION_TIME 		(5u) //cpu time in ticks
#define TASK5_PERIODICITY 			(10u)

//Not to be Null if we want handler return
TaskHandle_t task6Handler= NULL;
#define TASK6_PRIORITY				(1u)
#define TASK6_NAME					("Load_2_Simulation")
#define TASK6_STACK_SIZE_WORDS		(100u)
#define TASK6_EXECUTION_TIME 		(12u) //cpu time in ticks
#define TASK6_PERIODICITY 			(100u)

typedef unsigned short int uint16;
void DisableAllPortALedsExcept(uint16 ledID)
{
	uint16 pinIter;
	for(pinIter=PIN0 ;pinIter<PIN7; pinIter++)
	{
		if(pinIter == ledID)
		{
			GPIO_write(PORT_0, pinIter  ,PIN_IS_HIGH);
		}
		else
		{
			GPIO_write(PORT_0,  pinIter,  PIN_IS_LOW);
		}
	}
}


void vApplicationIdleHook( void )
{
	DisableAllPortALedsExcept(PIN6);
}

void TASK_1 (void *pvParameters)
{
	TickType_t xLastWakeTimeA;
	const TickType_t xFrequency = TASK1_PERIODICITY; //tsk A frequency
	volatile int count = TASK1_EXECUTION_TIME; //tsk A capacity
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTimeA = 0;

	while(1)
	{
		TickType_t xTime = xTaskGetTickCount ();
		TickType_t x;
	
		DisableAllPortALedsExcept(PIN0);
		while(count != 0)
		{
			x=xTaskGetTickCount ();
			if(( x = xTaskGetTickCount () ) > xTime)
			{
				xTime = x;
				count --;
			}
		}
		count = TASK1_EXECUTION_TIME;
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTimeA, xFrequency );
	}
}

void TASK_2 (void *pvParameters)
{
	TickType_t xLastWakeTimeA;
	const TickType_t xFrequency = TASK2_PERIODICITY; //tsk A frequency
	volatile int count = TASK2_EXECUTION_TIME; //tsk A capacity
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTimeA = 0;

	while(1)
	{
		TickType_t xTime = xTaskGetTickCount ();
		TickType_t x;
		DisableAllPortALedsExcept(PIN1);
		while(count != 0)
		{
			x=xTaskGetTickCount ();
			if(( x = xTaskGetTickCount () ) > xTime)
			{
				xTime = x;
				count --;
			}
		}
		count = TASK2_EXECUTION_TIME;
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTimeA, xFrequency );
	}
}


void TASK_3 (void *pvParameters)
{
	TickType_t xLastWakeTimeA;
	const TickType_t xFrequency = TASK3_PERIODICITY; //tsk A frequency
	volatile int count = TASK3_EXECUTION_TIME; //tsk A capacity
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTimeA = 0;

	while(1)
	{
		TickType_t xTime = xTaskGetTickCount ();
		TickType_t x;

		DisableAllPortALedsExcept(PIN2);

		while(count != 0)
		{
			x=xTaskGetTickCount ();
			if(( x = xTaskGetTickCount () ) > xTime)
			{
				xTime = x;
				count --;
			}
		}
		count = TASK3_EXECUTION_TIME;
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTimeA, xFrequency );
	}
}

void TASK_4 (void *pvParameters)
{
	TickType_t xLastWakeTimeA;
	const TickType_t xFrequency = TASK4_PERIODICITY; //tsk A frequency
	volatile int count = TASK4_EXECUTION_TIME; //tsk A capacity
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTimeA = 0;

	while(1)
	{
		TickType_t xTime = xTaskGetTickCount ();
		TickType_t x;
	
		DisableAllPortALedsExcept(PIN3);
		while(count != 0)
		{
			x=xTaskGetTickCount ();
			if(( x = xTaskGetTickCount () ) > xTime)
			{
				xTime = x;
				count --;
			}
		}
		count = TASK4_EXECUTION_TIME;
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTimeA, xFrequency );
	}
}

void TASK_5 (void *pvParameters)
{
	TickType_t xLastWakeTimeA;
	const TickType_t xFrequency = TASK5_PERIODICITY; //tsk A frequency
	volatile int count = TASK5_EXECUTION_TIME; //tsk A capacity
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTimeA = 0;

	while(1)
	{
		TickType_t xTime = xTaskGetTickCount ();
		TickType_t x;

		DisableAllPortALedsExcept(PIN4);

		while(count != 0)
		{
			x=xTaskGetTickCount ();
			if(( x = xTaskGetTickCount () ) > xTime)
			{
				xTime = x;
				count --;
			}
		}
		count = TASK5_EXECUTION_TIME;
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTimeA, xFrequency );
	}
}

void TASK_6 (void *pvParameters)
{
	TickType_t xLastWakeTimeA;
	const TickType_t xFrequency = TASK6_PERIODICITY; //tsk A frequency
	volatile int count = TASK6_EXECUTION_TIME; //tsk A capacity
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTimeA = 0;

	while(1)
	{
		TickType_t xTime = xTaskGetTickCount ();
		TickType_t x;

		DisableAllPortALedsExcept(PIN5);

		while(count != 0)
		{
			x=xTaskGetTickCount ();
			if(( x = xTaskGetTickCount () ) > xTime)
			{
				xTime = x;
				count --;
			}
		}
		count = TASK6_EXECUTION_TIME;
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTimeA, xFrequency );
	}
}
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	DisableAllPortALedsExcept(PIN15+1);

	
    /* Create Tasks here */
	xTaskPeriodicCreate(TASK_1,( const char * ) TASK1_NAME,TASK1_STACK_SIZE_WORDS,( void * ) 1,TASK1_PRIORITY,&task1Handler,TASK1_PERIODICITY);	
	xTaskPeriodicCreate(TASK_2,( const char * ) TASK2_NAME,TASK2_STACK_SIZE_WORDS,( void * ) 1,TASK2_PRIORITY,&task2Handler,TASK2_PERIODICITY);	
	xTaskPeriodicCreate(TASK_3,( const char * ) TASK3_NAME,TASK3_STACK_SIZE_WORDS,( void * ) 1,TASK3_PRIORITY,&task3Handler,TASK3_PERIODICITY);	
	xTaskPeriodicCreate(TASK_4,( const char * ) TASK4_NAME,TASK4_STACK_SIZE_WORDS,( void * ) 1,TASK4_PRIORITY,&task4Handler,TASK4_PERIODICITY);	
	xTaskPeriodicCreate(TASK_5,( const char * ) TASK5_NAME,TASK5_STACK_SIZE_WORDS,( void * ) 1,TASK5_PRIORITY,&task5Handler,TASK5_PERIODICITY);	
	xTaskPeriodicCreate(TASK_6,( const char * ) TASK6_NAME,TASK6_STACK_SIZE_WORDS,( void * ) 1,TASK6_PRIORITY,&task6Handler,TASK6_PERIODICITY);	
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
static void configTimer1(void)
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


