#include <driver/adc.h>

#include "MiniPID.h"

#include <Preferences.h>

#include "AiEsp32RotaryEncoder.h"

extern AiEsp32RotaryEncoder rotaryEncoder2;
extern AiEsp32RotaryEncoder rotaryEncoder1;

extern Preferences preferences;
extern int16_t encoder1_value;
extern int16_t encoder2_value;
extern double output1, output2;

MiniPID pid1=MiniPID(0.0,0.0,0.0);
MiniPID pid2=MiniPID(0.0,0.0,0.0);

int16_t pwmValueMax = 1024;

extern double target1, target2;
extern bool pidEnabled;
extern int16_t pwm1, pwm2;

void Task1( void * parameter )
{
  /*
  preferences.begin("settings", false);
  encoder1_value=preferences.getInt("encoder1_value");
  encoder2_value=preferences.getInt("encoder2_value");
  preferences.end();
  */

	rotaryEncoder2.begin();
  rotaryEncoder2.setup([]{rotaryEncoder2.readEncoder_ISR();});
  
  rotaryEncoder1.begin();
  rotaryEncoder1.setup([]{rotaryEncoder1.readEncoder_ISR();});

  rotaryEncoder1.setBoundaries(-10000,10000,false);
  rotaryEncoder2.setBoundaries(-10000,10000,false);

  //pid1.setOutputRampRate(10);
  //pid2.setOutputRampRate(10);
  pid1.setDirection(true);
  pid2.setDirection(true);
  pid1.setPID(1,0.1,0.0,0);
  pid2.setPID(1,0.1,0.0,0 );

  pid1.setOutputLimits(-1024.0, 1024.0);
  pid2.setOutputLimits(-1024.0, 1024.0);
//  pid1.setOutputFilter(1);
//  pid2.setOutputFilter(1);


  unsigned long start;
  long delta;
  for (;;) {
    //int an1 = analogRead( ADC1_CHANNEL_6_GPIO_NUM  );
    //int an2 = analogRead( ADC1_CHANNEL_7_GPIO_NUM  );

    // save encoder position if position changed
  
    encoder1_value = rotaryEncoder1.readEncoder();
    output1=pid1.getOutput((float)rotaryEncoder1.readEncoder(), target1);
    /*
    if(rotaryEncoder1.encoderChanged()>0)
    {
      encoder1_value = rotaryEncoder1.readEncoder();
      start = micros();   // ref: https://github.com/espressif/arduino-esp32/issues/384
      preferences.begin("settings", false);
      preferences.putInt("encoder1_value", rotaryEncoder1.readEncoder());
      preferences.end();
      delta = micros() - start;
      if(delta>20)
        Serial.printf("Preferences save completed in %u us.\n", delta);
    }
    */

    encoder2_value = rotaryEncoder2.readEncoder();
    output2=pid2.getOutput((float)rotaryEncoder2.readEncoder(), target2);
    /*
    if(rotaryEncoder2.encoderChanged()>0)
    {
      encoder2_value = rotaryEncoder2.readEncoder();
      start = micros();   // ref: https://github.com/espressif/arduino-esp32/issues/384
      preferences.begin("settings", false);
      preferences.putInt("encoder2_value", rotaryEncoder2.readEncoder());
      preferences.end();
      delta = micros() - start;
      if(delta>20)
        Serial.printf("Preferences save completed in %lu us.\n", delta);

      output2=pid2.getOutput((float)rotaryEncoder2.readEncoder(), target2);
    }
    */

  if(pidEnabled)
  {
    pwm1 = (int)output1;
    pwm2 = (int)output2;
  }

  vTaskDelay(100 / portTICK_PERIOD_MS);
 }
}


