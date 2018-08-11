#include <driver/adc.h>

#include "MiniPID.h"

#include <Preferences.h>

#include "AiEsp32RotaryEncoder.h"

// #define min(a,b) ((a)<(b)?(a):(b))
// #define max(a,b) ((a)>(b)?(a):(b))

extern AiEsp32RotaryEncoder rotaryEncoder2;
extern AiEsp32RotaryEncoder rotaryEncoder1;

extern Preferences preferences;
extern volatile int16_t encoder1_value, encoder2_value;
extern volatile double output1, output2;
extern volatile double target1, target2;
extern volatile int16_t pwm1, pwm2;

uint8_t pwmPositionDelta = 150;
uint8_t maxPositionDelta = 10;

extern MiniPID pid1;
extern MiniPID pid2;

int16_t pwmValueMax = 1024;

extern volatile bool pidEnabled;

/*
int8_t clip(int8_t n, int8_t lower, int8_t upper) {
  return max(lower, min(n, upper));
}
*/

static void IRAM_ATTR readEncoder1_ISR()
{
	portENTER_CRITICAL_ISR(&(rotaryEncoder1.mux));
	boolean A = digitalRead(rotaryEncoder1.encoderAPin);
	boolean B = digitalRead(rotaryEncoder1.encoderBPin);

	if((A==HIGH)&&(B==HIGH)) rotaryEncoder1.phase =1;
	if((A==HIGH)&&(B==LOW))  rotaryEncoder1.phase =2;
	if((A==LOW)&&(B==LOW))   rotaryEncoder1.phase =3;
	if((A==LOW)&&(B==HIGH))  rotaryEncoder1.phase =4;

	switch(rotaryEncoder1.phase){

		case 1:
		{
			if(rotaryEncoder1.phasep == 2)    rotaryEncoder1.encoder0Pos--;
			if(rotaryEncoder1.phasep == 4)    rotaryEncoder1.encoder0Pos++;
			break;
		}

		case 2:
		{
			if(rotaryEncoder1.phasep == 1)    rotaryEncoder1.encoder0Pos++;
			if(rotaryEncoder1.phasep == 3)    rotaryEncoder1.encoder0Pos--;
			break;
		}

		case 3:
		{
			if(rotaryEncoder1.phasep == 2)    rotaryEncoder1.encoder0Pos++;
			if(rotaryEncoder1.phasep == 4)    rotaryEncoder1.encoder0Pos--;
			break;
		}

		default:
		{
			if(rotaryEncoder1.phasep == 1)    rotaryEncoder1.encoder0Pos--;
			if(rotaryEncoder1.phasep == 3)    rotaryEncoder1.encoder0Pos++;
			break;
		}
	}


	rotaryEncoder1.phasep=rotaryEncoder1.phase;

	portEXIT_CRITICAL_ISR(&(rotaryEncoder1.mux));
}

static void IRAM_ATTR readEncoder2_ISR()
{
	portENTER_CRITICAL_ISR(&(rotaryEncoder2.mux));

	boolean A = digitalRead(rotaryEncoder2.encoderAPin);
	boolean B = digitalRead(rotaryEncoder2.encoderBPin);

	if((A==HIGH)&&(B==HIGH)) rotaryEncoder2.phase =1;
	if((A==HIGH)&&(B==LOW))  rotaryEncoder2.phase =2;
	if((A==LOW)&&(B==LOW))   rotaryEncoder2.phase =3;
	if((A==LOW)&&(B==HIGH))  rotaryEncoder2.phase =4;

	switch(rotaryEncoder2.phase){

		case 1:
		{
			if(rotaryEncoder2.phasep == 2)    rotaryEncoder2.encoder0Pos--;
			if(rotaryEncoder2.phasep == 4)    rotaryEncoder2.encoder0Pos++;
			break;
		}

		case 2:
		{
			if(rotaryEncoder2.phasep == 1)    rotaryEncoder2.encoder0Pos++;
			if(rotaryEncoder2.phasep == 3)    rotaryEncoder2.encoder0Pos--;
			break;
		}

		case 3:
		{
			if(rotaryEncoder2.phasep == 2)    rotaryEncoder2.encoder0Pos++;
			if(rotaryEncoder2.phasep == 4)    rotaryEncoder2.encoder0Pos--;
			break;
		}

		default:
		{
			if(rotaryEncoder2.phasep == 1)    rotaryEncoder2.encoder0Pos--;
			if(rotaryEncoder2.phasep == 3)    rotaryEncoder2.encoder0Pos++;
			break;
		}
	}


	rotaryEncoder2.phasep=rotaryEncoder2.phase;

	portEXIT_CRITICAL_ISR(&(rotaryEncoder2.mux));
}

void Task1( void * parameter )
{
	rotaryEncoder2.begin();
  rotaryEncoder1.begin();
  attachInterrupt(digitalPinToInterrupt(rotaryEncoder2.encoderAPin), readEncoder2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotaryEncoder2.encoderBPin), readEncoder2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotaryEncoder1.encoderAPin), readEncoder1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotaryEncoder1.encoderBPin), readEncoder1_ISR, CHANGE);


  for (;;) {
    //int an1 = analogRead( ADC1_CHANNEL_6_GPIO_NUM  );
    //int an2 = analogRead( ADC1_CHANNEL_7_GPIO_NUM  );

    // save encoder position if position changed
  
    encoder1_value = rotaryEncoder1.readEncoder();
    encoder2_value = rotaryEncoder2.readEncoder();



    if(pidEnabled)
    {
      int16_t encoderDelta = encoder1_value - encoder2_value;
      pid1.setPositionDiff(encoderDelta);
      pid2.setPositionDiff(-encoderDelta);
      pid1.setSetpoint(target1);
      pid2.setSetpoint(target2);
      output1=pid1.getOutput((float)encoder1_value, target1);    
      output2=pid2.getOutput((float)encoder2_value, target2);      

      pwm1 = (int)(output1); //- (deltaPos*1.0 / maxPositionDelta * pwmPositionDelta));
      pwm2 = (int)(output2); //+ (deltaPos*1.0 / maxPositionDelta * pwmPositionDelta));


/*
      if(abs(encoder1_value-target1)<=1 && abs(encoder2_value-target2)<=1)
      {
        //pwm1 = 0;
        //pwm2 = 0;
        //pid1.reset();
        //pid2.reset();
      }
      else
      {
        pwm1 = (int)(output1); //- (deltaPos*1.0 / maxPositionDelta * pwmPositionDelta));
        pwm2 = (int)(output2); //+ (deltaPos*1.0 / maxPositionDelta * pwmPositionDelta));
      }
*/
    }
    vTaskDelay(3 / portTICK_PERIOD_MS);
 }



}


