#include <driver/adc.h>

#include "MiniPID.h"

#include <Preferences.h>

#include "AiEsp32RotaryEncoder.h"

// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0 0
#define LEDC_CHANNEL_1 1
#define LEDC_CHANNEL_2 2
#define LEDC_CHANNEL_3 3
extern volatile uint16_t pwmValueMax;



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

extern uint16_t an1_max, an2_max;
extern uint16_t stop2_top, stop2_bottom;
extern uint16_t stop1_top, stop1_bottom;
extern String status;

extern volatile bool pidEnabled;

extern uint16_t an1, an2;
uint16_t lastAn1_fast, lastAn1_slow;
uint16_t lastAn2_fast, lastAn2_slow;

extern float an1_fast, an1_slow;
extern float an2_fast, an2_slow;


float fastFilter = 0.01; 
float slowFilter = 0.001;

extern long searchTopMilis;
extern String previousPercent_str_1, previousPercent_str_2;
extern void setOutputPercent(String percent_str, int i);

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = pwmValueMax) {
  /*
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * _min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
  */
  ledcWrite(channel, _min(value, valueMax));
}

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
		an1 = analogRead( ADC1_CHANNEL_6_GPIO_NUM  );
		an2 = analogRead( ADC1_CHANNEL_7_GPIO_NUM  );

		an1_fast = lastAn1_fast*fastFilter+an1*(1-fastFilter);
		an1_slow = lastAn1_slow*slowFilter+an1*(1-slowFilter);
		an2_fast = lastAn2_fast*fastFilter+an2*(1-fastFilter);
		an2_slow = lastAn2_slow*slowFilter+an2*(1-slowFilter);
		lastAn1_fast = an1_fast;
		lastAn1_slow = an1_slow;
		lastAn2_fast = an2_fast;
		lastAn2_slow = an2_slow;

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
		}

		if(status.equals("searchtop") || status.equals("searchbottom"))
		{
			// because of higher startup current we do check 1 sec after start of searchtop or searchbottom
			if((an1_slow >= 700.0 || an2_slow >= 700.0) && millis()>(searchTopMilis+1000))
			{
				Serial.print("Current Stop an1_slow: ");
				Serial.print(an1_slow);
				Serial.print("an2_slow: ");
				Serial.println(an2_slow);
				target1 = encoder1_value;
				target2 = encoder2_value;

				if(status.equals("searchtop"))
				{
					stop1_top = encoder1_value;
					stop2_top = encoder2_value;
					preferences.begin("settings", false);
					preferences.putInt("stop1_top", stop1_top);
					preferences.end();
				}
				else
				{
					stop1_bottom = encoder1_value;
					stop2_bottom = encoder2_value;
					preferences.begin("settings", false);
					preferences.putInt("stop1_bottom", stop1_bottom);
					preferences.end();
				}
				status = "";
				setOutputPercent(previousPercent_str_1, 1);
				setOutputPercent(previousPercent_str_2, 2);
			}
		}


		if(pwm1 > 0)
		{
			ledcAnalogWrite(LEDC_CHANNEL_0, pwm1, pwmValueMax);
			ledcAnalogWrite(LEDC_CHANNEL_1, 0, pwmValueMax);
			//Serial.printf("pwm1: %d\n", pwm1);
		}
		else if(pwm1 < 0)
		{
			ledcAnalogWrite(LEDC_CHANNEL_1, abs(pwm1), pwmValueMax);
			ledcAnalogWrite(LEDC_CHANNEL_0, 0, pwmValueMax);
			//Serial.printf("pwm1: %d\n", pwm1);
		}
		if(pwm2>=0)
		{
			ledcAnalogWrite(LEDC_CHANNEL_2, pwm2, pwmValueMax);
			ledcAnalogWrite(LEDC_CHANNEL_3, 0, pwmValueMax);
			//Serial.printf("pwm2: %d\n", pwm2);
		}
		else if(pwm2<0)
		{
			ledcAnalogWrite(LEDC_CHANNEL_3, abs(pwm2), pwmValueMax);
			ledcAnalogWrite(LEDC_CHANNEL_2, 0, pwmValueMax);
			//Serial.printf("pwm2: %d\n", pwm2);
		}		

		vTaskDelay(3 / portTICK_PERIOD_MS);
	}
}
