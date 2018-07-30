#include <driver/adc.h>
#include "AiEsp32RotaryEncoder.h"
#include <Preferences.h>
#include "MiniPID.h"

#define ROTARY_ENCODER1_A_PIN 16
#define ROTARY_ENCODER1_B_PIN 4
#define ROTARY_ENCODER2_A_PIN 17
#define ROTARY_ENCODER2_B_PIN 5

AiEsp32RotaryEncoder rotaryEncoder1 = AiEsp32RotaryEncoder(ROTARY_ENCODER1_A_PIN, ROTARY_ENCODER1_B_PIN, -1, -1);
AiEsp32RotaryEncoder rotaryEncoder2 = AiEsp32RotaryEncoder(ROTARY_ENCODER2_A_PIN, ROTARY_ENCODER2_B_PIN, -1, -1);
Preferences preferences;
int16_t encoder1_value;
int16_t encoder2_value;

MiniPID pid1=MiniPID(1,0,0);
MiniPID pid2=MiniPID(1,0,0);

int16_t pwmValueMax = 1024;

double target1, target2;
double output1, output2;


void workLoad()
{
  /*
  long l=0;
  double r;
  for(long j=0;j<10000;j=j+1)
  {
    l=l+1;
    r= random(0,100) * random(0,100) / random(0,100); 
  }*/
  int an1 = analogRead( ADC1_CHANNEL_6_GPIO_NUM  );
  int an2 = analogRead( ADC1_CHANNEL_7_GPIO_NUM  );
  //Serial.printf("%u %u %d %d\n", an1, an2, rotaryEncoder1.readEncoder(), rotaryEncoder2.readEncoder());
}


void Task1( void * parameter )
{
  preferences.begin("Logger", false);
  //preferences.putInt("encoder1_value", 0);
  //preferences.putInt("encoder2_value", 0);
  encoder1_value=preferences.getInt("encoder1_value");
  encoder2_value=preferences.getInt("encoder2_value");
  preferences.end();

  rotaryEncoder1.begin();
  rotaryEncoder2.begin();

  pid1.setOutputRampRate(0.2);
  pid2.setOutputRampRate(0.2);
//  pid1.setOutputFilter(1);
//  pid2.setOutputFilter(1);


  unsigned long start;
  long delta;
  for (;;) {
    workLoad();

    // save encoder position if position changed
    if(rotaryEncoder1.encoderChanged()>0)
    {
      start = micros();   // ref: https://github.com/espressif/arduino-esp32/issues/384
      preferences.begin("Logger", true);
      preferences.putInt("encoder1_value", rotaryEncoder1.readEncoder());
      preferences.end();
      delta = micros() - start;
      if(delta>20)
        Serial.printf("Preferences save completed in %lu us.\n", delta);

      output1=pid1.getOutput((float)rotaryEncoder1.readEncoder(), target1);
    }

    if(rotaryEncoder2.encoderChanged()>0)
    {
      start = micros();   // ref: https://github.com/espressif/arduino-esp32/issues/384
      preferences.begin("Logger", true);
      preferences.putInt("encoder2_value", rotaryEncoder2.readEncoder());
      preferences.end();
      delta = micros() - start;
      if(delta>20)
        Serial.printf("Preferences save completed in %lu us.\n", delta);

      output2=pid2.getOutput((float)rotaryEncoder2.readEncoder(), target2);
    }

    vTaskDelay(1);
 }
}


