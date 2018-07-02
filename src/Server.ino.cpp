# 1 "c:\\users\\klemen\\appdata\\local\\temp\\tmpfu9vqc"
#include <Arduino.h>
# 1 "C:/Users/klemen/Dropbox/Voga/BleVogaLifter-esp32-DRV8703Q/src/Server.ino"




#include <WiFi.h>

#include <FS.h>

#include "SPIFFS.h"

#include <AsyncTCP.h>

#include <ESPAsyncWebServer.h>

#include <SPI.h>

#include <ArduinoOTA.h>

#include <stdint.h>





const char* ssid = "AndroidAP";

const char* password = "Doitman1";

AsyncWebServer server(80);

AsyncWebSocket ws("/ws");





static const int spiClk = 1000000;

uint16_t toTransfer;



SPIClass * vspi = NULL;





#define LEDC_TIMER_10_BIT 10



#define LEDC_BASE_FREQ 5000



#define PWM1_PIN 12

#define PWM2_PIN 14

#define PWM3_PIN 26

#define PWM4_PIN 27



#define LEDC_CHANNEL_0 0

#define LEDC_CHANNEL_1 1

#define LEDC_CHANNEL_2 2

#define LEDC_CHANNEL_3 3

#define SS1 33

#define pwmValueMax 2000



int brightness = 0;

int fadeAmount = 1;



String status1;

String status2;





void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 1024) {



  uint32_t duty = (8191 / valueMax) * _min(value, valueMax);





  ledcWrite(channel, duty);

}





void listDir(fs::FS &fs, const char * dirname, uint8_t levels){

    Serial.printf("Listing directory: %s\r\n", dirname);



    File root = fs.open(dirname);

    if(!root){

        Serial.println("- failed to open directory");

        return;

    }

    if(!root.isDirectory()){

        Serial.println(" - not a directory");

        return;

    }



    File file = root.openNextFile();

    while(file){

        if(file.isDirectory()){

            Serial.print("  DIR : ");

            Serial.println(file.name());

            if(levels){

                listDir(fs, file.name(), levels -1);

            }

        } else {

            Serial.print("  FILE: ");

            Serial.print(file.name());

            Serial.print("\tSIZE: ");

            Serial.println(file.size());

        }

        file = root.openNextFile();

    }

}
void testSpi();
void pwmUp();
void pwmDown();
void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);
void setup();
void loop();
#line 175 "C:/Users/klemen/Dropbox/Voga/BleVogaLifter-esp32-DRV8703Q/src/Server.ino"
void testSpi()

{



  vspi->setDataMode(SPI_MODE1);

  vspi->setHwCs(false);



  usleep(1);

  digitalWrite(SS1, LOW);



  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
# 207 "C:/Users/klemen/Dropbox/Voga/BleVogaLifter-esp32-DRV8703Q/src/Server.ino"
  byte data_read = B10000000;

  byte data_address = B00010000;

  byte data = data_read | data_address;

  byte lowbyte = B0;

  uint16_t data_int = data << 8 | lowbyte;



  uint16_t reply = vspi->transfer16(data_int);

  vspi->endTransaction();

  digitalWrite(SS1, HIGH);

  usleep(1);

  if(reply == B00011000)

  {

    status1 = "DRV8703Q is ON (Not locked).";

  }



  usleep(1);

  digitalWrite(SS1, LOW);



  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));



  uint8_t FAULT_FAULT = B1 << 7;

  uint8_t FAULT_WDFLT = B1 << 6;

  uint8_t FAULT_GDF = B1 << 5;

  uint8_t FAULT_OCP = B1 << 4;

  uint8_t FAULT_VM_UVFL = B1 << 3;

  uint8_t FAULT_VCP_UVFL = B1 << 2;

  uint8_t FAULT_OTSD = B1 << 1;

  uint8_t FAULT_OTW = B1 << 0;



  data_read = B10000000;

  data_address = B00000000;

  data = data_read | data_address;

  lowbyte = B0;

  data_int = data << 8 | lowbyte;



  reply = vspi->transfer16(data_int);

  vspi->endTransaction();

  digitalWrite(SS1, HIGH);

  usleep(1);





  if((reply & FAULT_FAULT) > 0)

  {



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



void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){

  if(type == WS_EVT_CONNECT){



    printf("ws[%s][%u] connect\n", server->url(), client->id());

    client->printf("Hello Client %u :)", client->id());

    client->ping();

  } else if(type == WS_EVT_DISCONNECT){



    printf("ws[%s][%u] disconnect: %u\n", server->url(), client->id());

  } else if(type == WS_EVT_ERROR){



    printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);

  } else if(type == WS_EVT_PONG){



    printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len)?(char*)data:"");

  } else if(type == WS_EVT_DATA){



    AwsFrameInfo * info = (AwsFrameInfo*)arg;

    printf("Data came...");

    if(info->final && info->index == 0 && info->len == len){



      printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT)?"text":"binary", info->len);

      if(info->opcode == WS_TEXT){

        printf("Data11: ");

        data[len] = 0;

        printf("%s\n", (char*)data);

      } else {

        for(size_t i=0; i < info->len; i++){

          printf("%02x ", data[i]);

        }

        printf("\n");

      }

      if(info->opcode == WS_TEXT)

        client->text("I got your text message");

      else

        client->binary("I got your binary message");

    } else {



      if(info->index == 0){

        if(info->num == 0)

          printf("ws[%s][%u] %s-message start\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");

        printf("ws[%s][%u] frame[%u] start[%llu]\n", server->url(), client->id(), info->num, info->len);

      }



      printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT)?"text":"binary", info->index, info->index + len);

      if(info->message_opcode == WS_TEXT){

        printf("%s\n", (char*)data);

      } else {

        for(size_t i=0; i < len; i++){

          printf("%02x ", data[i]);

        }

        printf("\n");

      }



      if((info->index + len) == info->len){

        printf("ws[%s][%u] frame[%u] end[%llu]\n", server->url(), client->id(), info->num, info->len);

        if(info->final){

          printf("ws[%s][%u] %s-message end\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");

          if(info->message_opcode == WS_TEXT)

            client->text("I got your text message");

          else

            client->binary("I got your binary message");

        }

      }

    }

  }

}



void setup(){

  Serial.begin(115200);

  Serial.print("ESP ChipSize:");

  Serial.println(ESP.getFlashChipSize());



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



  listDir(SPIFFS, "/", 0);



  ws.onEvent(onEvent);

  server.addHandler(&ws);



  server.serveStatic("/", SPIFFS, "/");







  server.begin();

  ArduinoOTA.begin();

}



void loop(){

  ArduinoOTA.handle();

  sleep(1);

}