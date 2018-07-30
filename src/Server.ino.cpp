# 1 "c:\\users\\klemen\\appdata\\local\\temp\\tmpd3iv37"
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



#include <Ticker.h>

#include "TaskCore0.h"



#include <ArduinoJson.h>



String ssid("AndroidAP");

String password("Doitman1");



char* softAP_ssid = "MLIFT";

char* softAP_password = "Doitman1";



AsyncWebServer server(80);

AsyncWebSocket ws("/ws");

const char * mysystem_event_names[] = { "WIFI_READY", "SCAN_DONE", "STA_START", "STA_STOP", "STA_CONNECTED", "STA_DISCONNECTED", "STA_AUTHMODE_CHANGE", "STA_GOT_IP", "STA_LOST_IP", "STA_WPS_ER_SUCCESS", "STA_WPS_ER_FAILED", "STA_WPS_ER_TIMEOUT", "STA_WPS_ER_PIN", "AP_START", "AP_STOP", "AP_STACONNECTED", "AP_STADISCONNECTED", "AP_PROBEREQRECVED", "GOT_IP6", "ETH_START", "ETH_STOP", "ETH_CONNECTED", "ETH_DISCONNECTED", "ETH_GOT_IP", "MAX"};



int NO_AP_FOUND_count = 0;



static const int spiClk = 1000000;

uint16_t toTransfer;



SPIClass * vspi = NULL;



int shouldStopM1 = 0;

int shouldStopM2 = 0;

int shouldPwm_M1_left = 0;

int shouldPwm_M1_right = 0;

int shouldPwm_M2_left = 0;

int shouldPwm_M2_right = 0;





#define LEDC_TIMER_10_BIT 10



#define LEDC_BASE_FREQ 5000



#define PWM1_PIN GPIO_NUM_12

#define PWM2_PIN GPIO_NUM_14

#define PWM3_PIN GPIO_NUM_26

#define PWM4_PIN GPIO_NUM_27

#define LED_PIN GPIO_NUM_2



#define LEDC_RESOLUTION LEDC_TIMER_10_BIT

#define pwmValueMax 1024

#define pwmDelta 5





#define LEDC_CHANNEL_0 0

#define LEDC_CHANNEL_1 1

#define LEDC_CHANNEL_2 2

#define LEDC_CHANNEL_3 3

#define SS1 33

#define SS2 25







const char* NTP_SERVER0 = "0.si.pool.ntp.org";

const char* NTP_SERVER1 = "1.si.pool.ntp.org";

const char* NTP_SERVER2 = "2.si.pool.ntp.org";

const char* TZ_INFO2 = "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00";

time_t now;

struct tm info;





int pwm1 = 0;

int pwm2 = 0;

int fadeAmount = 1;



String status1;

String status2;

String gdfVds1;

String gdfVds2;

int motor1_pos;

int motor2_pos;



Ticker mover;



TaskHandle_t TaskA;



void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = pwmValueMax) {
# 195 "C:/Users/klemen/Dropbox/Voga/BleVogaLifter-esp32-DRV8703Q/src/Server.ino"
  ledcWrite(channel, _min(value, valueMax));

}
# 207 "C:/Users/klemen/Dropbox/Voga/BleVogaLifter-esp32-DRV8703Q/src/Server.ino"
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
void clearFault();
void testSpi();
void gdfVdsStatus();
void pwmStop();
String scanNetworks();
String getToken(String data, char separator, int index);
void processWsData(char *data, AsyncWebSocketClient* client);
void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);
String processor(const String& var);
void waitForIp();
void blink(int i);
void setup();
void loop();
void move();
#line 269 "C:/Users/klemen/Dropbox/Voga/BleVogaLifter-esp32-DRV8703Q/src/Server.ino"
void clearFault()

{

  digitalWrite(SS1, LOW);



  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));



  byte data_read = B00000000;

  byte data_address = B00010000;

  byte data = data_read | data_address;



  uint16_t data_int = data << 8 | B00000001;
# 299 "C:/Users/klemen/Dropbox/Voga/BleVogaLifter-esp32-DRV8703Q/src/Server.ino"
  uint16_t reply = vspi->transfer16(data_int);

  vspi->endTransaction();

  digitalWrite(SS1, HIGH);

}



void testSpi()

{

  usleep(1);

  digitalWrite(SS1, LOW);



  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
# 333 "C:/Users/klemen/Dropbox/Voga/BleVogaLifter-esp32-DRV8703Q/src/Server.ino"
  byte data_read = B10000000;

  byte data_address = B00010000;

  byte data = data_read | data_address;

  byte lowbyte = B0;

  uint16_t data_int = data << 8 | lowbyte;



  uint16_t reply = vspi->transfer16(data_int);

  vspi->endTransaction();

  digitalWrite(SS1, HIGH);

  usleep(1);

  status1 = "";

  status2 = "";

  if(reply == B00011000)

  {

    status1.concat("DRV8703Q is ON (Not locked)\n");



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

    Serial.print("SPI reply: ");

    Serial.println(reply, BIN);



    if((reply & FAULT_WDFLT) > 0)

    {

      status1.concat("Watchdog time-out fault\n");

    }

    if((reply & FAULT_GDF) > 0)

    {

      status1.concat("Gate drive fault\n");

    }

    if((reply & FAULT_OCP) > 0)

    {

      status1.concat("VDS monitor overcurrent fault\n");

    }

    if((reply & FAULT_VM_UVFL) > 0)

    {

      status1.concat("VM undervoltage lockout fault\n");

    }

    if((reply & FAULT_VCP_UVFL) > 0)

    {

      status1.concat("Charge-pump undervoltage fault\n");

    }

    if((reply & FAULT_OTSD) > 0)

    {

      status1.concat("Overtemperature shutdown\n");

    }

    if((reply & FAULT_OTW) > 0)

    {

      status1.concat("Overtemperature warning\n ");

    }



  }

  else

  {

    status1.concat("DRV8703Q is NOT ON\n");

  }





}





void gdfVdsStatus()

{

  usleep(1);

  digitalWrite(SS1, LOW);



  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
# 521 "C:/Users/klemen/Dropbox/Voga/BleVogaLifter-esp32-DRV8703Q/src/Server.ino"
  byte data_read = B10000000;

  byte data_address = B00001000;

  byte data = data_read | data_address;

  byte lowbyte = B0;

  uint16_t data_int = data << 8 | lowbyte;



  uint16_t reply = vspi->transfer16(data_int);

  vspi->endTransaction();

  digitalWrite(SS1, HIGH);

  usleep(1);

  gdfVds1 = "";





  uint8_t H2_GDF = B1 << 7;

  uint8_t L2_GDF = B1 << 6;

  uint8_t H1_GDF = B1 << 5;

  uint8_t L1_GDF = B1 << 4;

  uint8_t H2_VDS = B1 << 3;

  uint8_t L2_VDS = B1 << 2;

  uint8_t H1_VDS = B1 << 1;

  uint8_t L1_VDS = B1 << 0;



  digitalWrite(SS1, HIGH);

  usleep(1);

  Serial.print("SPI reply: ");

  Serial.println(reply, BIN);



  if((reply & H2_GDF) > 0)

  {

    gdfVds1.concat(" Gate drive fault on the high-side FET of half bridge 2\n");

  }

  if((reply & L2_GDF) > 0)

  {

    gdfVds1.concat("Gate drive fault on the low-side FET of half bridge 2\n");

  }

  if((reply & H1_GDF) > 0)

  {

    gdfVds1.concat("Gate drive fault on the high-side FET of half bridge 1\n");

  }

  if((reply & L1_GDF) > 0)

  {

    gdfVds1.concat("Gate drive fault on the low-side FET of half bridge 1\n");

  }

  if((reply & H2_VDS) > 0)

  {

    gdfVds1.concat("VDS monitor overcurrent fault on the high-side FET of half bridge 2\n");

  }

  if((reply & L2_VDS) > 0)

  {

    gdfVds1.concat("VDS monitor overcurrent fault on the low-side FET of half bridge 2\n");

  }

  if((reply & H1_VDS) > 0)

  {

    gdfVds1.concat("VDS monitor overcurrent fault on the high-side FET of half bridge 1\n ");

  }

  if((reply & L1_VDS) > 0)

  {

    gdfVds1.concat("VDS monitor overcurrent fault on the low-side FET of half bridge 1\n ");

  }

}





void pwmStop()

{

  pwm1 = 0;

  ledcAnalogWrite(LEDC_CHANNEL_0, pwm1, pwmValueMax);

}



String scanNetworks(){

  String ret;

  int n = WiFi.scanNetworks();

  Serial.println("scan done");

  if (n == 0) {

    ret.concat("no networks found");

  } else {

    for (int i = 0; i < n; ++i) {



        ret.concat("wifi ");

        ret.concat(WiFi.SSID(i));

        ret.concat(" (");

        ret.concat(WiFi.RSSI(i));

        ret.concat(") ");

        ret.concat((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?"OPEN":"PASS");

        ret.concat("\n");

    }

  }

  Serial.println(ret.c_str());

  return ret;

}



String getToken(String data, char separator, int index)

{

  int found = 0;

  int strIndex[] = {0, -1};

  int maxIndex = data.length()-1;



  for(int i=0; i<=maxIndex && found<=index; i++){

    if(data.charAt(i)==separator || i==maxIndex){

        found++;

        strIndex[0] = strIndex[1]+1;

        strIndex[1] = (i == maxIndex) ? i+1 : i;

    }

  }



  String ret("");

  if(found>index)

  {

    ret = data.substring(strIndex[0], strIndex[1]);

  }



  return ret;

}



void processWsData(char *data, AsyncWebSocketClient* client)

{

  String input;

  input.concat(data);

  printf("received: %s",input.c_str());

  if(input.equals(String("status")))

  {

    testSpi();

    client->printf("motor1_pos %i", motor1_pos);

    client->printf("motor2_pos %i", motor2_pos);

    client->printf("status: %s", status1.c_str());

    client->printf("shouldPwm_M1_left: %d", shouldPwm_M1_left);

    client->printf("shouldPwm_M1_right: %d", shouldPwm_M1_right);

    client->printf("shouldstop_M1: %d", shouldStopM1);

    client->printf("shouldPwm_M2_left: %d", shouldPwm_M2_left);

    client->printf("shouldPwm_M2_right: %d", shouldPwm_M2_right);

    client->printf("shouldstop_M2: %d", shouldStopM2);



  }else if(input.startsWith("gdfvdsstatus")){

    gdfVdsStatus();

    client->printf("gdfvdsstatus: %s", gdfVds1.c_str());

  }else if(input.startsWith("clrflt")){

    clearFault();

    client->printf("Clear Fault done.");

  }else if(input.startsWith("wificonnect")){

    ssid = getToken(input, ' ', 1);

    password = getToken(input, ' ', 2);

    NO_AP_FOUND_count = 0;

    printf("wificonnect %s %s", ssid.c_str(), password.c_str());

    WiFi.mode(WIFI_STA);

    WiFi.begin(ssid.c_str(), password.c_str());

    waitForIp();

  }else if(input.startsWith("pwm_m1_left")){

    shouldStopM1 = 0;

    shouldPwm_M1_left = 1;

    shouldPwm_M1_right = 0;

  }else if(input.startsWith("pwm_m1_right")){

    shouldStopM1 = 0;

    shouldPwm_M1_left = 0;

    shouldPwm_M1_right = 1;

  }else if(input.startsWith("stop pwm_m1")){

    shouldStopM1 = 1;

    shouldPwm_M1_left = 0;

    shouldPwm_M1_right = 0;

  }else if(input.startsWith("pwm_m2_left")){

    shouldStopM2 = 0;

    shouldPwm_M2_left = 1;

    shouldPwm_M2_right = 0;

  }else if(input.startsWith("pwm_m2_right")){

    shouldStopM2 = 0;

    shouldPwm_M2_left = 0;

    shouldPwm_M2_right = 1;

  }else if(input.startsWith("stop pwm_m2")){

    shouldStopM2 = 1;

    shouldPwm_M2_left = 0;

    shouldPwm_M2_right = 0;

  }else if(input.startsWith("scan")){

    printf("scan");

    client->printf(scanNetworks().c_str());

  }

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

    if(info->final && info->index == 0 && info->len == len){



      printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT)?"text":"binary", info->len);

      if(info->opcode == WS_TEXT){

        data[len] = 0;

        printf("%s\n", (char*)data);

        processWsData((char*)data, client);

      } else {

        printf("not text\n");

        for(size_t i=0; i < info->len; i++){

          printf("%02x ", data[i]);

        }

        printf("\n");

      }
# 947 "C:/Users/klemen/Dropbox/Voga/BleVogaLifter-esp32-DRV8703Q/src/Server.ino"
    } else {

      printf("multi frames\n");



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
# 1005 "C:/Users/klemen/Dropbox/Voga/BleVogaLifter-esp32-DRV8703Q/src/Server.ino"
        }

      }

    }

  }

}



String processor(const String& var)

{

  if(var == "encoder1_value")

    return F("Hello world!");

  return String();

}



void waitForIp()

{

  while (WiFi.status() != WL_CONNECTED) {



    delay(1000);

    Serial.print("SSID: ");

    Serial.println(ssid);

    Serial.print("password: ");

    Serial.println(password);

    Serial.print("status: ");

    Serial.println(WiFi.status());



    Serial.print("no ap count count: ");

    Serial.println(NO_AP_FOUND_count);



    if(NO_AP_FOUND_count >= 2)

    {

      WiFi.mode(WIFI_AP);

      if(WiFi.softAP(softAP_ssid, softAP_password))

      {

        Serial.println("Wait 100 ms for AP_START...");

        delay(100);

        Serial.println("");

        IPAddress Ip(192, 168, 1, 1);

        IPAddress NMask(255, 255, 255, 0);

        WiFi.softAPConfig(Ip, Ip, NMask);

        IPAddress myIP = WiFi.softAPIP();

        Serial.println("Network " + String(softAP_ssid) + " is running.");

        Serial.print("AP IP address: ");

        Serial.println(myIP);

      }

      else

      {

        Serial.println("Starting AP failed.");

      }

      break;

    }

  }



  Serial.print("status:");

  Serial.println(WiFi.status());



  Serial.print("WiFi local IP:");

  Serial.println(WiFi.localIP());



}



void blink(int i)

{

  for(int j=0; j<i; j++)

  {

    digitalWrite(LED_PIN, HIGH);

    usleep(50000);

    digitalWrite(LED_PIN, LOW);

    usleep(40000);

  }

}







void setup(){

  blink(1);



  Serial.begin(115200);

  Serial.print("ESP ChipSize:");

  Serial.println(ESP.getFlashChipSize());



  pinMode(LED_PIN, OUTPUT);

  pinMode(SS1, OUTPUT);

  pinMode(SS2, OUTPUT);





  Serial.println("initialise vspi with default pins 1...");

  vspi = new SPIClass(VSPI);





  vspi->begin(18,19,23,SS1);



  vspi->setDataMode(SPI_MODE1);

  vspi->setHwCs(false);



  Serial.println("initialise vspi with default pins 3...");



  delay(10);

  Serial.println("initialise ledc...");

  ledcSetup(LEDC_CHANNEL_0, 20000, LEDC_RESOLUTION);

  ledcSetup(LEDC_CHANNEL_1, 20000, LEDC_RESOLUTION);

  ledcSetup(LEDC_CHANNEL_2, 20000, LEDC_RESOLUTION);

  ledcSetup(LEDC_CHANNEL_3, 20000, LEDC_RESOLUTION);

  ledcAttachPin(PWM1_PIN, LEDC_CHANNEL_0);

  ledcAttachPin(PWM2_PIN, LEDC_CHANNEL_1);

  ledcAttachPin(PWM3_PIN, LEDC_CHANNEL_2);

  ledcAttachPin(PWM4_PIN, LEDC_CHANNEL_3);



  WiFiEventId_t eventID = WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info){

          Serial.print("WiFi lost connection. Reason: ");

          Serial.println(info.disconnected.reason);



          String msg;

          msg.concat(info.disconnected.reason);



          if(msg.indexOf("201")>=0){

            NO_AP_FOUND_count++;

          }



  }, WiFiEvent_t::SYSTEM_EVENT_STA_DISCONNECTED);



  Serial.println("Configuring adc1");

  adc1_config_width(ADC_WIDTH_BIT_10);
# 1257 "C:/Users/klemen/Dropbox/Voga/BleVogaLifter-esp32-DRV8703Q/src/Server.ino"
  WiFi.begin(ssid.c_str(), password.c_str());

  waitForIp();



  Serial.println("Config ntp time...");

  configTzTime(TZ_INFO2, NTP_SERVER0, NTP_SERVER1, NTP_SERVER2);



  time(&now);

  localtime_r(&now, &info);



  Serial.println(&info, "%Y-%m-%d %H:%M:%S");

  Serial.println("Config ntp time...DONE.");



  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){

    Serial.println("/");

    Serial.println("redirecting to /index.html");

    request->redirect("/index.html");

  });



  if(!SPIFFS.begin()){

      Serial.println("SPIFFS Mount Failed");

  }



  listDir(SPIFFS, "/", 0);



  ws.onEvent(onEvent);

  server.addHandler(&ws);



  server.serveStatic("/", SPIFFS, "/").setCacheControl("max-age=40").setDefaultFile("index.html").setTemplateProcessor(processor);



  server.on("/json", HTTP_POST, [](AsyncWebServerRequest *request){



  }, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){

    DynamicJsonBuffer jsonBuffer;

    JsonObject& root = jsonBuffer.parseObject((const char*)data);

    if (root.success()) {



      if (root.containsKey("cmd")) {

        Serial.println(root["cmd"].asString());

      }

      if (root.containsKey("cmd1")) {

        Serial.println(root["cmd1"].asString());

      }



      AsyncResponseStream *response = request->beginResponseStream("application/json");

      DynamicJsonBuffer jsonBuffer;

      JsonObject &root = jsonBuffer.createObject();

      root["heap"] = ESP.getFreeHeap();

      root["ssid"] = WiFi.SSID();

      root.printTo(*response);

      request->send(response);



    } else {

      request->send(404, "text/plain", "");

    }

  });



  server.begin();

  ArduinoOTA.begin();



  ArduinoOTA.onStart([]() {



    SPIFFS.end();





    ws.enable(false);





    ws.textAll("OTA Update Started");





    ws.closeAll();

  });
# 1423 "C:/Users/klemen/Dropbox/Voga/BleVogaLifter-esp32-DRV8703Q/src/Server.ino"
  mover.attach_ms(5, move);





 xTaskCreatePinnedToCore(

   Task1,

   "Workload1",

   4096,

   NULL,

   1,

   &TaskA,

   0);



  blink(5);



}



void loop(){

  ArduinoOTA.handle();

  sleep(5);

  vTaskDelay(1);

}



void move(){





  if(shouldPwm_M1_left == 1 && shouldStopM1 != 1)

  {

    if(pwm1<=(pwmValueMax-pwmDelta))

    {

      pwm1 = pwm1 + pwmDelta;

    }

  }

  else if(shouldPwm_M1_right == 1 && shouldStopM1 != 1)

  {

    if(pwm1>=(-pwmValueMax+pwmDelta))

    {

      pwm1 = pwm1 - pwmDelta;

    }

  }else if(shouldStopM1 == 1)

  {

    if(pwm1>0)

      pwm1 = pwm1 - pwmDelta;

    else if(pwm1<0)

      pwm1 = pwm1+pwmDelta;

    else

      shouldStopM1 = 0;

  }





  if(pwm1 > 0)

  {

      ledcAnalogWrite(LEDC_CHANNEL_0, pwm1, pwmValueMax);



  }

  else if(pwm1<0)

  {

      ledcAnalogWrite(LEDC_CHANNEL_1, abs(pwm1), pwmValueMax);



  }





  if(shouldPwm_M2_left == 1 && shouldStopM2 != 1)

  {

    if(pwm2<=(pwmValueMax-pwmDelta))

    {

      pwm2=pwm2+pwmDelta;

    }

  }

  else if(shouldPwm_M2_right == 1 && shouldStopM2 != 1)

  {

    if(pwm2>=(-pwmValueMax+pwmDelta))

    {

      pwm2=pwm2-pwmDelta;

    }

  }else if(shouldStopM2 == 1)

  {

    if(pwm2>0)

      pwm2=pwm2-pwmDelta;

    else if(pwm2<0)

      pwm2=pwm2+pwmDelta;

    else

      shouldStopM2 = 0;

  }



  if(pwm2>=0)

  {

      ledcAnalogWrite(LEDC_CHANNEL_2, pwm2, pwmValueMax);



  }

  else if(pwm2<0)

  {

      ledcAnalogWrite(LEDC_CHANNEL_3, abs(pwm2), pwmValueMax);



  }



}