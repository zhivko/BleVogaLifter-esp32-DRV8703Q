void workLoad (void) ;                // prototype is required
void Task1( void * parameter )
{
  int start = millis();
 for (;;) {
   unsigned long start = millis();   // ref: https://github.com/espressif/arduino-esp32/issues/384
   workLoad();
   Serial.printf("Task 1 complete running on Core %d time %u ms.\n", xPortGetCoreID(), (millis() - start));
   delay(10) ;
 }
}

void workLoad()
{
  delay(5);
}
