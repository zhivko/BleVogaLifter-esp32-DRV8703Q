#include <WiFi.h>
#include <FS.h>
#include "SPIFFS.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPI.h>
#include <ArduinoOTA.h>


const char* ssid = "AndroidAP";
const char* password =  "Doitman1";
AsyncWebServer server(80);


static const int spiClk = 1000000; // 1 MHz
uint16_t toTransfer;
//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_10_BIT  10
// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 5000
// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define PWM1_PIN 12
#define PWM2_PIN 14
#define PWM3_PIN 26
#define PWM4_PIN 27
// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0 0
#define LEDC_CHANNEL_1 1
#define LEDC_CHANNEL_2 2
#define LEDC_CHANNEL_3 3
#define SS1 33
#define pwmValueMax  2000

int brightness = 0; // how bright the LED is
int fadeAmount = 1; // how many points to fade the LED by

String status1;
String status2;


void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 1024) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * _min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}


void testSpi()
{
  //SPI_MODE0, ..., SPI_MODE3
  vspi->setDataMode(SPI_MODE1);
  vspi->setHwCs(false);

  usleep(1);
  digitalWrite(SS1, LOW);
  // SPI WRITE
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));

  // http://www.ti.com/product/drv8702-q1?qgpn=drv8702-q1
  // datasheet: http://www.ti.com/lit/gpn/drv8702-q1
  // page 42


  byte data_read = B10000000;  // READ OPERATION
  byte data_address = B00010000; // ADDRES 02x MAIN REGISTER
  byte data = data_read | data_address;
  byte lowbyte = B0;
  uint16_t data_int = data << 8 | lowbyte;

  uint16_t reply = vspi->transfer16(data_int);  // should return 0x18 B00011000
  vspi->endTransaction();
  digitalWrite(SS1, HIGH);
  usleep(1);
  if(reply == B00011000)
  {
    status1 = "DRV8703Q is ON (Not locked).";
  }

  usleep(1);
  digitalWrite(SS1, LOW);
  // SPI WRITE
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  // fault bytes:
  uint8_t FAULT_FAULT = B1 << 7;     // FAULT R 0b Logic OR of the FAULT status register excluding the OTW bit
  uint8_t FAULT_WDFLT = B1 << 6;     // WDFLT R 0b Watchdog time-out fault
  uint8_t FAULT_GDF = B1 << 5;       // GDF R 0b Indicates gate drive fault condition
  uint8_t FAULT_OCP = B1 << 4;       // OCP R 0b Indicates VDS monitor overcurrent fault condition
  uint8_t FAULT_VM_UVFL = B1 << 3;   // VM_UVFL R 0b Indicates VM undervoltage lockout fault condition
  uint8_t FAULT_VCP_UVFL = B1 << 2;  // VCP_UVFL R 0b Indicates charge-pump undervoltage fault condition
  uint8_t FAULT_OTSD = B1 << 1;      // OTSD R 0b Indicates overtemperature shutdown
  uint8_t FAULT_OTW = B1 << 0;       // OTW R 0b Indicates overtemperature warning

  data_read = B10000000;  // READ OPERATION
  data_address = B00000000; // ADDRES 0x FAULT REGISTER
  data = data_read | data_address;
  lowbyte = B0;
  data_int = data << 8 | lowbyte;

  reply = vspi->transfer16(data_int);  // should return 0x18
  vspi->endTransaction();
  digitalWrite(SS1, HIGH);
  usleep(1);
  //Serial.println(reply);

  if((reply & FAULT_FAULT) > 0)
  {
    //Serial.println("fault");
  }
  if((reply & FAULT_WDFLT) > 0)
  {
    status1 = "Watchdog time-out fault";
  }
  if((reply & FAULT_GDF) > 0)
  {
    status1 = "Gate drive fault";
  }
  if((reply & FAULT_OCP) > 0)
  {
    status1 = "VDS monitor overcurrent fault";
  }
  if((reply & FAULT_VM_UVFL) > 0)
  {
    status1 = "VM undervoltage lockout fault";
  }
  if((reply & FAULT_VCP_UVFL) > 0)
  {
    status1 = "Charge-pump undervoltage fault";
  }
  if((reply & FAULT_OTSD) > 0)
  {
    status1 = "Overtemperature shutdown";
  }
  if((reply & FAULT_OTW) > 0)
  {
    status1 = "Overtemperature warning";
  }
}

void pwmUp(){
  if(brightness<=(pwmValueMax-1))
  {
    brightness = brightness + 1;
    ledcAnalogWrite(LEDC_CHANNEL_0, brightness, pwmValueMax);
  }
}

void pwmDown()
{
  if(brightness>=1)
  {
   brightness = brightness - 1;
   ledcAnalogWrite(LEDC_CHANNEL_0, brightness, pwmValueMax);
  }
  Serial.print("duty= ");
  Serial.println(brightness);
}

void setup(){
  Serial.begin(115200);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println(WiFi.localIP());

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("/status");
    testSpi();
    request->send(200, "text/plain", status1);
  });
  server.on("/up", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("/up");
    pwmUp();
    request->send(200, "text/plain", "Hello World");
  });
  server.on("/down", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("/down");
    request->send(200, "text/plain", "Hello World");
  });

  if(!SPIFFS.begin()){
      Serial.println("SPIFFS Mount Failed");
      return;
  }  

  server.serveStatic("/", SPIFFS, "/www/");

  server.begin();
  ArduinoOTA.begin();
}

void loop(){
  ArduinoOTA.handle();
  usleep(10);
}
