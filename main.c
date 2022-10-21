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
#include "semphr.h"
#include "queue.h"

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )
	
/*-------------------------------------------------------------------------*/

#define BUTTON_1_TASK_PER         50
#define BUTTON_2_TASK_PER         50
#define PERIODIC_TRANS_TASK_PER               100
#define UART_RECEIVER_TASK_PER               20
#define LOAD_1_TASK_PER           10
#define LOAD_2_TASK_PER           100

#define BUTTON_1_READ_PIN									PORT_1 , PIN1
#define BUTTON_2_READ_PIN									PORT_1 , PIN2

#define IDLE_HOOK_PIN											PORT_0 , PIN0
#define BUTTON_1_TASK_PIN       					PORT_0 , PIN1  
#define BUTTON_2_TASK_PIN         				PORT_0 , PIN2
#define PERIODIC_TRANS_TASK_PIN           PORT_0 , PIN3    
#define UART_RECEIVER_TASK_PIN            PORT_0 , PIN4  
#define LOAD_1_TASK_PIN           				PORT_0 , PIN5
#define LOAD_2_TASK_PIN           				PORT_0 , PIN6
#define TICK_HOOK_PIN											PORT_0 , PIN7

int Button_1_Task_Time_in, Button_2_Task_Time_out, Button_1_Task_Time_total;
int Button_2_Task_Time_in, Button_2_Task_Time_out, Button_2_Task_Time_total;
int Periodic_Tx_Task_Time_in, Periodic_Tx_Task_Time_out,  Periodic_Tx_Task_Time_total;
int UART_Rx_Task_Time_in, UART_Rx_Task_Time_out,  UART_Rx_Task_Time_total;
int Load_1_Task_Time_in, Load_1_Task_Time_out,  Load_1_Task_Time_total;
int Load_2_Task_Time_in, Load_2_Task_Time_out,  Load_2_Task_Time_total;
int sys_time;
int cpu_load;
	
TaskHandle_t Button_1_Task_Handler = NULL;
TaskHandle_t Button_2_Task_Handler = NULL;
TaskHandle_t Periodic_Transmitter_Task_Handler = NULL;
TaskHandle_t Uart_Receiver_Task_Handler = NULL;
TaskHandle_t Load_1_Task_Handler = NULL;
TaskHandle_t Load_2_Task_Handler = NULL;

QueueHandle_t xQueue1=NULL ;
QueueHandle_t xQueue2=NULL ;
QueueHandle_t xQueue3=NULL ;

void Button_1_Monitor( void * pvParameters );
void Button_2_Monitor( void * pvParameters );
void Task_Transmitter( void * pvParameters );
void Uart_Receiver( void * pvParameters );
void Load_1_Simulation( void * pvParameters );
void Load_2_Simulation( void * pvParameters );

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
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	
	xQueue1 = xQueueCreate( 3,sizeof(char*) );
	xQueue2 = xQueueCreate( 3,sizeof(char*) );	
	xQueue3 = xQueueCreate( 3,sizeof(char*) );

/* Create the task, storing the handle. */
   xTaskPeriodicCreate(
                    Button_1_Monitor,       /* Function that implements the task. */
                    "Button_1",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Button_1_Task_Handler, /* Used to pass out the created task's handle. */
										BUTTON_1_TASK_PER /* Task Period */ );      					
			
										/* Create the task, storing the handle. */
   xTaskPeriodicCreate(
                    Button_2_Monitor,       /* Function that implements the task. */
                    "Button_2",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Button_2_Task_Handler, /* Used to pass out the created task's handle. */
										BUTTON_2_TASK_PER /* Task Period */); 									
											
										/* Create the task, storing the handle. */
   xTaskPeriodicCreate(
                    Task_Transmitter,       /* Function that implements the task. */
                    "Transmitter",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Periodic_Transmitter_Task_Handler, /* Used to pass out the created task's handle. */
										PERIODIC_TRANS_TASK_PER /* Task Period */  );
										
										/* Create the task, storing the handle. */
   xTaskPeriodicCreate(
                    Uart_Receiver,       /* Function that implements the task. */
                    "uart",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Uart_Receiver_Task_Handler, /* Used to pass out the created task's handle. */
										UART_RECEIVER_TASK_PER /* Task Period */  );	
										
                    /* Create the task, storing the handle. */
  xTaskPeriodicCreate(
                    Load_1_Simulation,       /* Function that implements the task. */
                    "Load1",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Load_1_Task_Handler, /* Used to pass out the created task's handle. */
										LOAD_1_TASK_PER /* Task Period */);
										
										/* Create the task, storing the handle. */
   xTaskPeriodicCreate(
                    Load_2_Simulation,       /* Function that implements the task. */
                    "Load2",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Load_2_Task_Handler,	/* Used to pass out the created task's handle. */
										LOAD_2_TASK_PER /* Task Period */ ); 		
											
										
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

void vApplicationIdleHook( void )
{
      GPIO_write(IDLE_HOOK_PIN , PIN_IS_HIGH);
}
void vApplicationTickHook( void )
{
      GPIO_write(TICK_HOOK_PIN , PIN_IS_HIGH);
			GPIO_write(TICK_HOOK_PIN , PIN_IS_LOW);
}



void Button_1_Monitor( void * pvParameters )
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
	char send_to_queue = 0;
	pinState_t new_state;
	pinState_t prev_state = GPIO_read(BUTTON_1_READ_PIN);
  
	vTaskSetApplicationTaskTag( NULL, ( void * ) 1 );
	
	for( ;; )
	{
		new_state = GPIO_read(BUTTON_1_READ_PIN);
		if (new_state != prev_state )
		{
			if ( new_state == PIN_IS_HIGH ) 
			{
				send_to_queue = 1 ;
			}
			else 
			{
				send_to_queue = 2 ;
			}
      if(xQueue1 != 0)
      {	
        xQueueSend( xQueue1, ( void * )&send_to_queue, (TickType_t) 0 );
      }
		}
		prev_state = new_state;

		vTaskDelayUntil( &xLastWakeTime , BUTTON_1_TASK_PER); 
		
		/*IDLE task pin*/
		GPIO_write(IDLE_HOOK_PIN,PIN_IS_LOW);
	}
}


void Button_2_Monitor( void * pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	char send_to_queue = 0;
	pinState_t new_state;
	pinState_t prev_state = GPIO_read(BUTTON_2_READ_PIN);
  
	vTaskSetApplicationTaskTag( NULL, ( void * ) 2 );
	
	for( ;; )
	{
		new_state = GPIO_read(BUTTON_2_READ_PIN);
		if (new_state != prev_state )
		{
			if ( new_state == PIN_IS_HIGH ) 
			{
				send_to_queue = 1 ;
			}
			else 
			{
				send_to_queue = 2 ;
			}
      if(xQueue2 != 0)
      {	
        xQueueSend( xQueue2, ( void * )&send_to_queue, (TickType_t) 0 );
      }
		}
		prev_state = new_state;
		
		vTaskDelayUntil( &xLastWakeTime , BUTTON_2_TASK_PER); 
		
		/*IDLE task pin*/
		GPIO_write(IDLE_HOOK_PIN,PIN_IS_LOW);
	}
}


void Task_Transmitter( void * pvParameters )
{
	char send_to_queue =0;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	vTaskSetApplicationTaskTag( NULL, ( void * ) 3 );
	
	for( ;; )
	{
    if(xQueue3 != 0)
			{
        xQueueSend( xQueue3, ( void * )&send_to_queue, (TickType_t) 0 );
      }

		vTaskDelayUntil( &xLastWakeTime , PERIODIC_TRANS_TASK_PER);

		/*IDLE task pin*/
		GPIO_write(IDLE_HOOK_PIN,PIN_IS_LOW);
	}
}

void Uart_Receiver( void * pvParameters )
{	
	char receive_from_queue = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	vTaskSetApplicationTaskTag( NULL, ( void * ) 4 );
	
  for(;;)
  {
        if( xQueue1 != NULL )
        {
            if ((xQueueReceive(xQueue1,(void *) &(receive_from_queue), (TickType_t)0))==pdPASS )
            {
                if(receive_from_queue == 1) 
								{
									vSerialPutString((const signed char * const)"Button 1 Rising Edge\n", 21);
								}
                else if (receive_from_queue == 2) 
								{
									vSerialPutString((const signed char * const)"Button 1 Falling Edge\n", 22);
								}
            }
				}	
				 
        if(xQueue2!=NULL)
        {
            if ((xQueueReceive(xQueue2,(void *) &(receive_from_queue), (TickType_t)0))==pdPASS )
            {
                if(receive_from_queue == 1) 
								{
									vSerialPutString((const signed char * const)"Button 2 Rising Edge\n", 21);
								}
                else if (receive_from_queue == 2) 
								{
									vSerialPutString((const signed char * const)"Button 2 Falling Edge\n", 22);
								}
            }
				}
        if(xQueue3!=NULL)
        {
            if ((xQueueReceive(xQueue3,(void *) &(receive_from_queue), (TickType_t)0))==pdPASS )
            {
                if (receive_from_queue == 0) 
								{
									vSerialPutString((const signed char * const)"Periodic Message\n", 17);
								}
            }
				}					
		
		vTaskDelayUntil( &xLastWakeTime , UART_RECEIVER_TASK_PER); 
				
		/*IDLE task pin*/
		GPIO_write(IDLE_HOOK_PIN,PIN_IS_LOW);
	}
}


void Load_1_Simulation ( void * pvParameters )
{
	int i = 0 ;
	
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	vTaskSetApplicationTaskTag( NULL, ( void * ) 5 );	
	for( ; ; )
	{		
		for( i=0 ; i <= 33225 ; i++)
		{
			i = i ;
		}

		vTaskDelayUntil( &xLastWakeTime , LOAD_1_TASK_PER);
	
		/*IDLE task pin*/
		GPIO_write(IDLE_HOOK_PIN,PIN_IS_LOW);
	}
}


void Load_2_Simulation ( void * pvParameters )
{
	int i = 0 ;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	vTaskSetApplicationTaskTag( NULL, ( void * ) 6 );
	for( ; ; )
	{		
		for( i=0 ; i <= 79730 ; i++)
		{
			i = i ;
		}

		vTaskDelayUntil( &xLastWakeTime , LOAD_2_TASK_PER);
			
		/*IDLE task pin*/
		GPIO_write(IDLE_HOOK_PIN,PIN_IS_LOW);
	}
}