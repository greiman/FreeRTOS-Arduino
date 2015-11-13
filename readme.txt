This is a port of FreeRTOS as Arduino libraries.

The documentation for FreeRTOS is located here:

http://www.freertos.org/

FreeRTOS is version 8.2.3

These libraries were tested with Arduino 1.6.5 for AVR boards, Arduino 1.6.4
for Due and the 1.26 version of the Teensy 3 software.

To install these libraries and run the FreeRTOS examples, copy the
included libraries to your libraries folder.

A version of SdFat is included.  For the current version go to:

https://github.com/greiman/SdFat

Please read FreeRTOS.html for more information.

See FreeRTOS_API.html for API documentation.

__malloc_heap_end must be defined to use dynamic memory in AVR tasks.
Add this as the first line of setup():

  // Insure malloc works in tasks
  __malloc_heap_end = (char*)RAMEND;

See: http://www.nongnu.org/avr-libc/user-manual/malloc.html
