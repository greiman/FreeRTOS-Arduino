/*
  FreeRTOS.org V5.0.4 - Copyright (C) 2003-2008 Richard Barry.

  This file is part of the FreeRTOS.org distribution.

  FreeRTOS.org is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  FreeRTOS.org is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with FreeRTOS.org; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

  A special exception to the GPL can be applied should you wish to distribute
  a combined work that includes FreeRTOS.org, without being obliged to provide
  the source code for any proprietary components.  See the licensing section
  of http://www.FreeRTOS.org for full details of how and when the exception
  can be applied.

    ***************************************************************************
    ***************************************************************************
    *                                                                         *
    * SAVE TIME AND MONEY!  We can port FreeRTOS.org to your own hardware,    *
    * and even write all or part of your application on your behalf.          *
    * See http://www.OpenRTOS.com for details of the services we provide to   *
    * expedite your project.                                                  *
    *                                                                         *
    ***************************************************************************
    ***************************************************************************

  Please ensure to read the configuration and relevant port sections of the
  online documentation.

  http://www.FreeRTOS.org - Documentation, latest information, license and
  contact details.

  http://www.SafeRTOS.com - A version that is certified for use in safety
  critical systems.

  http://www.OpenRTOS.com - Commercial support, development, porting,
  licensing and training services.
*/

/* FreeRTOS.org includes. */
#include "FreeRTOS_ARM.h"
//#include "task.h"
//#include "queue.h"
//#include "portasm.h"

/* Demo includes. */
#include "basic_io_arm.h"

/* Compiler includes. */
//#include <dos.h>

/* The tasks to be created. */
static void vIntegerGenerator( void *pvParameters );
static void vStringPrinter( void *pvParameters );

/* The service routine for the interrupt.  This is the interrupt that the task
will be synchronized with. */
static void vExampleInterruptHandler( void );


unsigned long ulNext = 0;
unsigned long ulCount;
unsigned long ul[ 100 ];

/*-----------------------------------------------------------*/

/* Declare two variables of type QueueHandle_t.  One queue will be read from
within an ISR, the other will be written to from within an ISR. */
QueueHandle_t xIntegerQueue, xStringQueue;

// pins to generate interrupts - they must be connected
const uint8_t inputPin = 2;
const uint8_t outputPin = 3;

void setup( void )
{
  Serial.begin(9600);

  /* Before a queue can be used it must first be created.  Create both queues
  used by this example.  One queue can hold variables of type unsigned long,
  the other queue can hold variables of type char*.  Both queues can hold a
  maximum of 10 items.  A real application should check the return values to
  ensure the queues have been successfully created. */
  xIntegerQueue = xQueueCreate( 10, sizeof( unsigned long ) );
  xStringQueue = xQueueCreate( 10, sizeof( char * ) );

  /* Create the task that uses a queue to pass integers to the interrupt service
  routine.  The task is created at priority 1. */
  xTaskCreate( vIntegerGenerator, "IntGen", 200, NULL, 1, NULL );

  /* Create the task that prints out the strings sent to it from the interrupt
  service routine.  This task is created at the higher priority of 2. */
  xTaskCreate( vStringPrinter, "String", 200, NULL, 2, NULL );
  
  /* Install the interrupt handler. */
  pinMode(inputPin, INPUT);
  pinMode(outputPin, OUTPUT);
  digitalWrite(outputPin, HIGH);
  bool tmp = digitalRead(inputPin);
  digitalWrite(outputPin, LOW);
  if (digitalRead(inputPin) || !tmp) {
    Serial.println("pin 2 must be connected to pin 3");
    while(1);
  }
  attachInterrupt(inputPin, vExampleInterruptHandler, RISING);
  
  /* Start the scheduler so the created tasks start executing. */
  vTaskStartScheduler();

    /* If all is well we will never reach here as the scheduler will now be
    running the tasks.  If we do reach here then it is likely that there was
    insufficient heap memory available for a resource to be created. */
  for( ;; );
//  return 0;
}
/*-----------------------------------------------------------*/

static void vIntegerGenerator( void *pvParameters )
{
TickType_t xLastExecutionTime;
unsigned portLONG ulValueToSend = 0;
int i;

  /* Initialize the variable used by the call to vTaskDelayUntil(). */
  xLastExecutionTime = xTaskGetTickCount();

  for( ;; )
  {
    /* This is a periodic task.  Block until it is time to run again.
    The task will execute every 200ms. */
    vTaskDelayUntil( &xLastExecutionTime, 200 / portTICK_PERIOD_MS );

    /* Send an incrementing number to the queue five times.  These will be
    read from the queue by the interrupt service routine.  A block time is
    not specified. */
    for( i = 0; i < 5; i++ )
    {
      xQueueSendToBack( xIntegerQueue, &ulValueToSend, 0 );
      ulValueToSend++;
    }

    /* Force an interrupt so the interrupt service routine can read the
    values from the queue. */
    vPrintString( "Generator task - About to generate an interrupt.\r\n" );
    
    digitalWrite(outputPin, LOW);
    digitalWrite(outputPin, HIGH);

    vPrintString( "Generator task - Interrupt generated.\r\n\r\n\r\n" );
  }
}
/*-----------------------------------------------------------*/

static void vStringPrinter( void *pvParameters )
{
char *pcString;

  for( ;; )
  {
    /* Block on the queue to wait for data to arrive. */
    xQueueReceive( xStringQueue, &pcString, portMAX_DELAY );

    /* Print out the string received. */
    vPrintString( pcString );
  }
}
/*-----------------------------------------------------------*/

static void vExampleInterruptHandler( void )
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  static unsigned long ulReceivedNumber;

  /* The strings are declared static const to ensure they are not allocated to the
  interrupt service routine stack, and exist even when the interrupt service routine
  is not executing. */
  static const char *pcStrings[] =
  {
    "String 0\r\n",
    "String 1\r\n",
    "String 2\r\n",
    "String 3\r\n"
  };


  /* Loop until the queue is empty. */
  while( xQueueReceiveFromISR( xIntegerQueue, &ulReceivedNumber, &xHigherPriorityTaskWoken ) != errQUEUE_EMPTY )
  {
    /* Truncate the received value to the last two bits (values 0 to 3 inc.), then
    send the string that corresponds to the truncated value to the other
    queue. */
    ulReceivedNumber &= 0x03;
    xQueueSendToBackFromISR( xStringQueue, &pcStrings[ ulReceivedNumber ], &xHigherPriorityTaskWoken );
  }
  /* xHigherPriorityTaskWoken was initialised to pdFALSE.  It will have then
  been set to pdTRUE only if reading from or writing to a queue caused a task
  of equal or greater priority than the currently executing task to leave the
  Blocked state.  When this is the case a context switch should be performed.
  In all other cases a context switch is not necessary.

  NOTE: The syntax for forcing a context switch within an ISR varies between
  FreeRTOS ports.  The portEND_SWITCHING_ISR() macro is provided as part of
  the Cortex-M3 port layer for this purpose.  taskYIELD() must never be called
  from an ISR! */
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
//------------------------------------------------------------------------------
void loop() {}