/**
 * \file
 * \brief FreeRTOS for AVR Arduino
 */
#include <Arduino.h>
#include <FreeRTOS_AVR.h>
#include <stdlib.h>

/** current begining of heap */
extern char *__brkval;

#if defined(CORE_TEENSY) || ARDUINO == 104
extern char __bss_end;
/** \return free heap size */
size_t freeHeap() {
  return (char*)RAMEND - (__brkval ? __brkval : &__bss_end)+ 1;
}
#else  // CORE_TEENSY
/** \return free heap size */
size_t freeHeap() {
  return (char*)RAMEND - (__brkval ? __brkval : __malloc_heap_start) + 1;
}
#endif  // CORE_TEENSY

//------------------------------------------------------------------------------
/** calibration factor for delayMS */
#define CAL_FACTOR (F_CPU/20000)
/** delay between led error flashes
 * \param[in] millis milliseconds to delay
 */
static void delayMS(uint32_t millis) {
  uint32_t iterations = millis * CAL_FACTOR;
  uint32_t i;
  for(i = 0; i < iterations; ++i) {
    asm volatile("nop\n\t");
  }
}
//------------------------------------------------------------------------------
/** Blink error pattern
 *
 * \param[in] n  number of short pulses
 */
static void errorBlink(int n) {

  noInterrupts();
  pinMode(13, OUTPUT);
  for (;;) {
    int i;
    for (i = 0; i < n; i++) {
      digitalWrite(13, 1);
      delayMS(300);
      digitalWrite(13, 0);
      delayMS(300);
    }
    delayMS(2000);
  }
}
//------------------------------------------------------------------------------
/** assertBlink
 * Blink one short pulse every two seconds if configASSERT fails.
*/
void assertBlink() {
  errorBlink(1);
}
//------------------------------------------------------------------------------
	/** vApplicationMallocFailedHook()
   Blink two short pulses if malloc fails.

	will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
void vApplicationMallocFailedHook() {
  errorBlink(2);
}
/*-----------------------------------------------------------*/
	/**  Blink three short pulses if stack overflow is detected.
	Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.
  \param[in] pxTask Task handle
  \param[in] pcTaskName Task name
  */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
	(void) pcTaskName;
	(void) pxTask;
//  void assertMsg(const char* file, int line);
//  assertMsg(pcTaskName, 0);
	errorBlink(3);
}