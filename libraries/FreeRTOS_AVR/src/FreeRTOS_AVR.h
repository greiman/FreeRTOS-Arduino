/**
 * \file
 * \brief FreeRTOS for AVR Arduino
 */
#ifndef FreeRTOS_AVR_h
#define FreeRTOS_AVR_h

#if !defined(ARDUINO) || ARDUINO < 100
#error Arduino 1.0 or greater required
#endif  // ARDUINO

#ifndef __AVR__
#error avr based board required
#else  // __AVR__

#include "utility/FreeRTOS.h"
#include "utility/task.h"
#include "utility/queue.h"
#include "utility/semphr.h"

#ifdef __cplusplus
extern "C"{
#endif  //  __cplusplus

//------------------------------------------------------------------------------
/** FreeROS_AVR version YYYYMMDD */
#define FREE_RTOS_AVR_VERSION 20151112
//------------------------------------------------------------------------------

size_t freeHeap();

#ifdef __cplusplus
} // extern "C"
#endif  // __cplusplus
#endif  // __AVR__
#endif  // FreeRTOS_AVR_h