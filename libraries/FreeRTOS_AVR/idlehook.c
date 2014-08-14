void loop();
void __attribute__((weak)) vApplicationIdleHook() {
  loop();
}
void __attribute__((weak)) vApplicationTickHook() {
}