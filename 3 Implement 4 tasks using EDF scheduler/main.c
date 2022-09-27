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

/*the macro of period of all tasks*/
#define Button_1_PERIOD         50
#define Button_2_PERIOD         50
#define Transmitte_PERIOD       100
#define Receive_PERIOD          20
#define LOAD_1_PERIOD           10
#define LOAD_2_PERIOD           100

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )
	
/*the handler of the task in the system */

TaskHandle_t Button_1_TaskHandler = NULL;
TaskHandle_t Button_2_TaskHandler = NULL;
TaskHandle_t Transmitter_TaskHandler = NULL;
TaskHandle_t Uart_TaskHandler = NULL;
TaskHandle_t Load_1_TaskHandler = NULL;
TaskHandle_t Load_2_TaskHandler = NULL;
QueueHandle_t xQueue=NULL;

int button_1_TaskInTime = 0, button_1_TaskOutTime = 0, button_1_TaskTotalTime;
int button_2_TaskInTime = 0, button_2_TaskOutTime = 0, button_2_TaskTotalTime;
int periodic_TaskInTime = 0, periodic_TaskOutTime = 0, periodic_TaskTotalTime;
int uart_TaskInTime     = 0, uart_TaskOutTime     = 0, uart_TaskTotalTime;
int load_1_TaskInTime   = 0, load_1_TaskOutTime   = 0, load_1_TaskTotalTime;
int load_2_TaskInTime   = 0, load_2_TaskOutTime   = 0, load_2_TaskTotalTime;

int system_Time = 0;
int cpu_Load = 0;


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/
/* HOOK task */
void vApplicationIdleHook( void );
void vApplicationTickHook( void );

/*  the Task 1 to check the rising and falling edge */
void Button_1_Monitor( void * pvParameters );

/*  the Task 2 to check the rising and falling edge */
void Button_2_Monitor( void * pvParameters );

/* the Task 3 transimtion periodic string  */
void Task_Transmitter( void * pvParameters );

/* the Task 4 Uart Receiver  */
void Uart_Receiver( void * pvParameters );

/* the Task 5 load 1 5ms  */
void Load_1_Simulation( void * pvParameters );

/* the Task 6 load 2 12ms  */
void Load_2_Simulation( void * pvParameters );

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	xQueue = xQueueCreate( 3,sizeof(char*) );

/* Task 1: ""Button_1_Monitor"", {Periodicity: 50, Deadline: 50} */
   xTaskPeriodicCreate(
                    Button_1_Monitor,       /* Function that implements the task. */
                    "Button_1",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Button_1_TaskHandler,
										Button_1_PERIOD/*period task */ );      /* Used to pass out the created task's handle. */					
			
/* Task 2: ""Button_2_Monitor"", {Periodicity: 50, Deadline: 50} */	
   xTaskPeriodicCreate(
                    Button_2_Monitor,       /* Function that implements the task. */
                    "Button_2",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Button_2_TaskHandler,
										Button_2_PERIOD /*period task */);      /* Used to pass out the created task's handle. */											
											
	/* Task 3: ""Periodic_Transmitter"", {Periodicity: 100, Deadline: 100} */									
   xTaskPeriodicCreate(
                    Task_Transmitter,       /* Function that implements the task. */
                    "Transmitter",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Transmitter_TaskHandler,
										Transmitte_PERIOD /*period task */  );      /* Used to pass out the created task's handle. */		
										
	/* Task 4: ""Uart_Receiver"", {Periodicity: 20, Deadline: 20}		*/							
   xTaskPeriodicCreate(
                    Uart_Receiver,       /* Function that implements the task. */
                    "uart",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Uart_TaskHandler,
										Receive_PERIOD /*period task */  );      /* Used to pass out the created task's handle. */		
										
/* Task 5: ""Load_1_Simulation"", {Periodicity: 10, Deadline: 10}, Execution time: 5ms
 */	
  xTaskPeriodicCreate(
                    Load_1_Simulation,       /* Function that implements the task. */
                    "Load1",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Load_1_TaskHandler,
										LOAD_1_PERIOD /*period task */);      /* Used to pass out the created task's handle. */	
										
/* Task 6: ""Load_2_Simulation"", {Periodicity: 100, Deadline: 100}, Execution time: 12ms */																		
   xTaskPeriodicCreate(
                    Load_2_Simulation,       /* Function that implements the task. */
                    "Load2",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Load_2_TaskHandler,	
										LOAD_2_PERIOD /*period task */ );      /* Used to pass out the created task's handle. */			
											

/*
	while(1){
		
		GPIO_write(PORT_0, PIN0, PIN_IS_HIGH);
		
		GPIO_write(PORT_0, PIN0, PIN_IS_LOW);
		
	}*/										

										
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

/* Read T1TC value */
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
/* tick hook callback function */
void vApplicationIdleHook( void )
{
      GPIO_write(PORT_0,PIN0,PIN_IS_HIGH);
}
/* idle task callback function */
void vApplicationTickHook( void )
{
      GPIO_write(PORT_0,PIN1,PIN_IS_HIGH);
			GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
}



/*
This task will monitor rising and falling edge on button 1 and send this event to the consumer task. 
(Note: The rising and failling edges are treated as separate events, hence they have separate strings)
*/
void Button_1_Monitor( void * pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	pinState_t Button_1_State;
	pinState_t PrevState = GPIO_read(PORT_0 , PIN8);//read initial value the frite
  //empty message string
	char* button_1_message = NULL;
	
	/* This task is going to be represented by a voltage scale of 1 . */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 1 );
	
	for( ;; )
	{
		/* Read GPIO Input pin value high or low  */
		Button_1_State = GPIO_read(PORT_0 , PIN8);
		
		/*Check for Edges stutes */
		if (Button_1_State == PIN_IS_LOW && PrevState == PIN_IS_HIGH)
		{
			// falling
			button_1_message ="\n Button 1: OFF";
			
			if(xQueue != 0)
          {
						xQueueSend( xQueue, ( void * )&button_1_message, (TickType_t) 0 );
          }
		}
		else if( Button_1_State == PIN_IS_HIGH && PrevState == PIN_IS_LOW)
		{
			// rising 
			button_1_message ="\n Button 1: ON";
			 
       if(xQueue != 0)
          {
						xQueueSend( xQueue, ( void * )&button_1_message, (TickType_t) 0 );
          }
		}
		
		/*save the ststue new that  value in Button_1_State in the old  PrevState */
		PrevState = Button_1_State;
		
		/*Periodicity  of the task 1 equal 50*/
		vTaskDelayUntil( &xLastWakeTime , Button_1_PERIOD); 
		
		/*pin indection of idel task*/
		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	}
}

/*
This task will monitor rising and falling edge on button 2 and send this event to the consumer task. 
(Note: The rising and failling edges are treated as separate events, hence they have separate strings)
*/
void Button_2_Monitor( void * pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	pinState_t Button_2_State;
	pinState_t PrevState = GPIO_read(PORT_0 , PIN9);//read initial value the frite
	char* button_2_message =NULL;
	
	
	/* This task is going to be represented by a voltage scale of 2 . */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 2 );
	
	for( ;; )
	{
		/* Read GPIO Input pin value high or low  */
		Button_2_State = GPIO_read(PORT_0 , PIN9);
		
		/*Check for Edges stutes */
		if (Button_2_State == PIN_IS_LOW && PrevState == PIN_IS_HIGH)
		{
			
			// falling
			button_2_message ="\n Button 2: OFF";
			
       if(xQueue!= 0)
          {
            xQueueSend( xQueue, ( void * )&button_2_message, (TickType_t) 0 );
          }
		}
		else if( Button_2_State == PIN_IS_HIGH && PrevState == PIN_IS_LOW)
		{
			// rising 
			button_2_message = "\n Button 2: ON";
			
       if(xQueue!= 0)
          {
            xQueueSend( xQueue, ( void * )&button_2_message, (TickType_t) 0 );
          }
		}
		
		/*save the ststue new that  value in Button_1_State in the old  PrevState */
		PrevState = Button_2_State;

		/*Periodicity  of the task 1 equal 50*/
		vTaskDelayUntil( &xLastWakeTime , Button_2_PERIOD); 
		
		/*pin indection of idel task*/
		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	}
}

/* This task will send preiodic string every 100ms to the consumer task */
void Task_Transmitter( void * pvParameters )
{
	volatile char*	periodic_message =NULL;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	
	/* This task is going to be represented by a voltage scale of 3 . */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 3 );
	
	for( ;; )
	{
		periodic_message = "\n periodic Task";
		
				if(xQueue != 0)
          {
						xQueueSend( xQueue, ( void * )&periodic_message, (TickType_t) 0 );
          }
					
		/*Periodicity  of the task 3 equal 100*/
		vTaskDelayUntil( &xLastWakeTime , Transmitte_PERIOD);

		/*pin indection of idel task*/
		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	}
}

/* This is the consumer task which will write on UART any received string from other tasks */
void Uart_Receiver( void * pvParameters )
{	
	
	char* receive_message = NULL;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	
	/* This task is going to be represented by a voltage scale of 4 . */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 4 );
	
  for(;;)
  {
				 //check the Queue
        if(xQueue!=NULL)
        {
            if ((xQueueReceive(xQueue,(void *) &(receive_message), (TickType_t)0))==pdPASS )
            {
							//send state via Uart
							vSerialPutString((const signed char * const)receive_message, 16);
            }
				}		
				
		vTaskDelayUntil( &xLastWakeTime , Receive_PERIOD); 
				
		/*pin indection of idel task*/
		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	}
}

/*
These two tasks shall be implemented as en empty loop that loops X times. 
You shall determine the X times to achieve the required execution time mentioned above. 
(Hint: In run-time use GPIOs and logic analyzer to determine the execution time)"
*/


void Load_1_Simulation ( void * pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint32_t i=0, x1Tick = 12000*5;  /* (XTAL / 1000U)*time_in_ms  */
	
	
	/* This task is going to be represented by a voltage scale of 5 . */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 5 );
	
	for( ; ; )
	{		
		for( i=0 ; i <= x1Tick; i++){/*5 ms delay*/}
	
		/*Periodicity  of the task 5 equal 10*/
		vTaskDelayUntil( &xLastWakeTime , LOAD_1_PERIOD);
	
		/*pin indection of idel task*/
		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	}
}

void Load_2_Simulation ( void * pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint32_t i=0, x1Tick = 12000*12;  /* (XTAL / 1000U)*time_in_ms  */
	
	
	/* This task is going to be represented by a voltage scale of 6 . */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 6 );
	 
	for( ; ; )
	{		
		for( i=0 ; i <= x1Tick; i++){/*12 ms delay*/}
			
		vTaskDelayUntil( &xLastWakeTime , LOAD_2_PERIOD);
			
		/*pin indection of idel task*/
		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	}
}
