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
//#include "semphr.h"

/* Compiler includes. */
//#include <stdlib.h>
//#include <stdio.h>

/* The task that sends messages to the stdio gatekeeper.  Two instances of this
task are created. */
static void prvPrintTask( void *pvParameters );

/* The gatekeeper task itself. */
static void prvStdioGatekeeperTask( void *pvParameters );

/* Define the strings that the tasks and interrupt will print out via the gatekeeper. */
static char *pcStringsToPrint[] =
{
  "Task 1 ****************************************************\r\n",
  "Task 2 ----------------------------------------------------\r\n",
  "Message printed from the tick hook interrupt ##############\r\n"
};

/*-----------------------------------------------------------*/

/* Declare a variable of type QueueHandle_t.  This is used to send messages from
the print tasks to the gatekeeper task. */
QueueHandle_t xPrintQueue;


void setup( void )
{
  Serial.begin(9600);
    /* Before a queue is used it must be explicitly created.  The queue is created
  to hold a maximum of 5 character pointers. */
    xPrintQueue = xQueueCreate( 5, sizeof( char * ) );

  /* The tasks are going to use a pseudo random delay, seed the random number
  generator. */
  srand( 567 );

  /* Check the queue was created successfully. */
  if( xPrintQueue != NULL )
  {
    /* Create two instances of the tasks that send messages to the gatekeeper.
    The index to the string they attempt to write is passed in as the task
    parameter (4th parameter to xTaskCreate()).  The tasks are created at
    different priorities so some pre-emption will occur. */
    xTaskCreate( prvPrintTask, "Print1", 200, ( void * ) 0, 1, NULL );
    xTaskCreate( prvPrintTask, "Print2", 200, ( void * ) 1, 2, NULL );

    /* Create the gatekeeper task.  This is the only task that is permitted
    to access standard out. */
    xTaskCreate( prvStdioGatekeeperTask, "Gatekeeper", 200, NULL, 0, NULL );

    /* Start the scheduler so the created tasks start executing. */
    vTaskStartScheduler();
  }

    /* If all is well we will never reach here as the scheduler will now be
    running the tasks.  If we do reach here then it is likely that there was
    insufficient heap memory available for a resource to be created. */
  for( ;; );
//  return 0;
}
/*-----------------------------------------------------------*/

static void prvStdioGatekeeperTask( void *pvParameters )
{
char *pcMessageToPrint;

  /* This is the only task that is allowed to write to the terminal output.
  Any other task wanting to write to the output does not access the terminal
  directly, but instead sends the output to this task.  As only one task
  writes to standard out there are no mutual exclusion or serialization issues
  to consider within this task itself. */
  for( ;; )
  {
    /* Wait for a message to arrive. */
    xQueueReceive( xPrintQueue, &pcMessageToPrint, portMAX_DELAY );

    /* There is no need to check the return value as the task will block
    indefinitely and only run again when a message has arrived.  When the
    next line is executed there will be a message to be output. */
    //printf( "%s", pcMessageToPrint );
    //fflush( stdout );
    Serial.print(pcMessageToPrint );
    Serial.flush();
    if (Serial.available()) {
    	vTaskEndScheduler();
    }
    /* Now simply go back to wait for the next message. */
  }
}
/*-----------------------------------------------------------*/
extern "C"{  // FreeRTOS expects C linkage
void vApplicationTickHook( void )
{
static int iCount = 0;
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  /* Print out a message every 200 ticks.  The message is not written out
  directly, but sent to the gatekeeper task. */
  iCount++;
  if( iCount >= 200 )
  {
    /* In this case the last parameter (xHigherPriorityTaskWoken) is not
    actually used but must still be supplied. */
    xQueueSendToFrontFromISR( xPrintQueue, &( pcStringsToPrint[ 2 ] ), &xHigherPriorityTaskWoken );

    /* Reset the count ready to print out the string again in 200 ticks
    time. */
    iCount = 0;
  }
}
}
/*-----------------------------------------------------------*/

static void prvPrintTask( void *pvParameters )
{
int iIndexToString;

  /* Two instances of this task are created so the index to the string the task
  will send to the gatekeeper task is passed in the task parameter.  Cast this
  to the required type. */
  iIndexToString = ( int ) pvParameters;

  for( ;; )
  {
    /* Print out the string, not directly but by passing the string to the
    gatekeeper task on the queue.  The queue is created before the scheduler is
    started so will already exist by the time this task executes.  A block time
    is not specified as there should always be space in the queue. */
    xQueueSendToBack( xPrintQueue, &( pcStringsToPrint[ iIndexToString ] ), 0 );

    /* Wait a pseudo random time.  Note that rand() is not necessarily
    re-entrant, but in this case it does not really matter as the code does
    not care what value is returned.  In a more secure application a version
    of rand() that is known to be re-entrant should be used - or calls to
    rand() should be protected using a critical section. */
    vTaskDelay( ( rand() & 0x1FF ) );
  }
}
//------------------------------------------------------------------------------
void loop() {}

