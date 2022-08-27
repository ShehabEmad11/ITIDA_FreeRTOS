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
/*Testing Purposes*/
typedef unsigned char uint8;
uint8 firstCounter=0;
uint8 firstID=100,secondID=100;


//Not to be Null if we want handler return
TaskHandle_t task1Handler= NULL;
#define TASK1_PRIORITY	1
#define TASK1_PRIODICITY 10
#define TASK1_STACK_SIZE_WORDS		(100)

//Not to be Null if we want handler return
TaskHandle_t task2Handler= NULL;
#define TASK2_PRIORITY	1
#define TASK2_PRIODICITY 20
#define TASK2_STACK_SIZE_WORDS		(100)

#define CAPACITY 2 //cpu time in tick
#define A_PERIOD 5 //task A period
#define B_PERIOD 8 //task B period


void vApplicationIdleHook( void )
{
	if(firstCounter==0)
	{
		firstID=3;
		firstCounter++;
	}
	GPIO_write(PORT_0,PIN2,PIN_IS_HIGH);

	GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
}
#if 0
void Task1( void * pvParameters )
{
    /* The parameter value is expected to be 1 as 1 is passed in the
    pvParameters value in the call to xTaskCreate() below. 
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );*/

	TickType_t lastTickVal = xTaskGetTickCount();
	#define PIN_OFF		0
	#define PIN_ON		1
	static unsigned char lastState=PIN_OFF;

	if(firstCounter==0)
	{
		firstID=1;
		firstCounter++;
	}
    for( ;; )
    {
		GPIO_write(PORT_0,PIN0,PIN_IS_HIGH);


		GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
		GPIO_write(PORT_0,PIN2,PIN_IS_LOW);

		vTaskDelayUntil(&lastTickVal,A_PERIOD);
    }
}

void Task2( void * pvParameters )
{
    /* The parameter value is expected to be 1 as 1 is passed in the
    pvParameters value in the call to xTaskCreate() below. 
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );*/

	TickType_t lastTickVal = xTaskGetTickCount();

	if(firstCounter==0)
	{
		firstID=2;
		firstCounter++;
	}

    for( ;; )
    {
		GPIO_write(PORT_0,PIN1,PIN_IS_HIGH);

		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
		GPIO_write(PORT_0,PIN2,PIN_IS_LOW);
		/*Should block according to task periodicity*/
		vTaskDelayUntil(&lastTickVal,B_PERIOD);
    }
}
#endif
void TSK_A (void *pvParameters)
{
	TickType_t xLastWakeTimeA;
	const TickType_t xFrequency = A_PERIOD; //tsk A frequency
	volatile int count = CAPACITY; //tsk A capacity
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTimeA = xTaskGetTickCount();
	if(firstCounter==0)
	{
		firstID=1;
		firstCounter++;
	}
	else if(firstCounter==1)
	{
		secondID=1;
		firstCounter++;
	}
	while(1)
	{
		TickType_t xTime = xTaskGetTickCount ();
		TickType_t x;
		GPIO_write(PORT_0,PIN0,PIN_IS_HIGH);

		GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
		GPIO_write(PORT_0,PIN2,PIN_IS_LOW);
		while(count != 0)
		{
			x=xTaskGetTickCount ();
#if 0
			if(x >= xTime + CAPACITY)
				break;
#endif
			if(( x = xTaskGetTickCount () ) > xTime)
			{
				xTime = x;
				count --;
			}
		}
		count = CAPACITY;
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTimeA, xFrequency );
	}
}

void TSK_B (void *pvParameters)
{
	TickType_t xLastWakeTimeB;
	const TickType_t xFrequency = B_PERIOD; //tsk A frequency
	volatile int count = CAPACITY; //tsk A capacity
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTimeB = xTaskGetTickCount();
	while(1)
	{
		TickType_t xTime = xTaskGetTickCount ();
		TickType_t x;
		if(firstCounter==0)
		{
			firstID=2;
			firstCounter++;
		}
		else if(firstCounter==1)
		{
			secondID=2;
			firstCounter++;
		}
		GPIO_write(PORT_0,PIN1,PIN_IS_HIGH);

		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
		GPIO_write(PORT_0,PIN2,PIN_IS_LOW);
		while(count != 0)
		{
			if(( x = xTaskGetTickCount () ) > xTime)
			{
				xTime = x;
				count --;
			}
		}
		count = CAPACITY;
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTimeB, xFrequency );
	}
}

int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
	GPIO_write(PORT_0,PIN2,PIN_IS_LOW);
	
    /* Create Tasks here */

	xTaskPeriodicCreate(TSK_A,( const char * ) "A",TASK1_STACK_SIZE_WORDS,( void * ) 1,TASK1_PRIORITY,&task1Handler,A_PERIOD);	
	xTaskPeriodicCreate(TSK_B,( const char * ) "B",TASK2_STACK_SIZE_WORDS,( void * ) 1,TASK2_PRIORITY,&task2Handler,B_PERIOD);

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


