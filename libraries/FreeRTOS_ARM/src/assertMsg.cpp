#include <Arduino.h>

extern "C" {
/**
 *  Print file and line when configASSERT is defied like this.
 *
 * #define configASSERT( x ) if( ( x ) == 0 ) {assertMsg(__FILE__,__LINE__);}
 */
void assertMsg(const char* file, int line) {
    interrupts();
    Serial.print(file);
    Serial.write('.');
    Serial.println(line);
    Serial.flush();
    noInterrupts();
    for (;;) {}
}
}  // extern "C"