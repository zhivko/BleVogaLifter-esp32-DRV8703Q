#include "FDC2212.h"

void i2cTask_func( void * parameter )
{
	Serial.println("Setting up FDC2212...");
	FDC2212 fdc2212 = FDC2212();
	Serial.println("Setting up FDC2212. Done.");
	if(fdc2212.begin()){
		long i=0;
		for (;;) {
			//uint32_t result = fdc2212.getReading();
			//Serial.printf("%lu FDC2212 read: %ui\n", i, result);
			uint32_t result =0 ;
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			Serial.printf("%lu FDC2212 read: %ui\n", i, result);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			i++;
		}
	}
}
