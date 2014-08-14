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
//#include <stdio.h>
//#include <conio.h>
#include <Arduino.h>
#include "FreeRTOS_ARM.h"
//#include "task.h"

void vPrintString( const char *pcString )
{
	/* Print the string, suspending the scheduler as method of mutual
	exclusion. */
	vTaskSuspendAll();
	{
      Serial.print(pcString);
      Serial.flush();
	//	printf( "%s", pcString );
	//	fflush( stdout );
	}
	xTaskResumeAll();

    /* Allow any key to stop the application running.  A real application that
    actually used the key value should protect access to the keyboard too. */
	if( Serial.available() )
	{
		vTaskEndScheduler();
	}
}
/*-----------------------------------------------------------*/

void vPrintStringAndNumber( const char *pcString, unsigned portLONG ulValue )
{
	/* Print the string, suspending the scheduler as method of mutual
	exclusion. */
	vTaskSuspendAll();
	{
//		printf( "%s %lu\r\n", pcString, ulValue );
//		fflush( stdout );
      Serial.print(pcString);
      Serial.write(' ');
      Serial.println(ulValue);
      Serial.flush();
	}
	xTaskResumeAll();

	/* Allow any key to stop the application running. */
	if( Serial.available() )
	{
		vTaskEndScheduler();
	}
}


