#include <WiFi.h>
#include <SPI.h>
#include <ArduinoOTA.h>

// SET PLATFORMIO_BUILD_FLAGS='-DWIFI_PASS="My password"' '-DWIFI_SSID="My ssid name"'
/*
#ifndef WIFI_PASS
#define WIFI_PASS "Defined in environment variable PLATFORMIO_BUILD_FLAGS"
#endif
#ifndef WIFI_SID
#define WIFI_SSID  "Defined in environment variable PLATFORMIO_BUILD_FLAGS"
#endif
*/

WiFiServer server(80);

static const int spiClk = 1000000; // 1 MHz
uint16_t toTransfer;

//uninitalised pointers to SPI objects
SPIClass * hspi = NULL;

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


int brightness = 0; // how bright the LED is
int fadeAmount = 1; // how many points to fade the LED by

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 1024) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * _min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}

void setup()
{


    Serial.begin(115200);
    pinMode(PWM1_PIN, OUTPUT);
    pinMode(PWM2_PIN, OUTPUT);
    pinMode(PWM3_PIN, OUTPUT);
    pinMode(PWM4_PIN, OUTPUT);

    //initialise vspi with default pins
    Serial.println("initialise vspi with default pins 1...");
    hspi = new SPIClass(HSPI);
    Serial.println("initialise vspi with default pins 2...");
    // VSPI - SCLK = 18, MISO = 19, MOSI = 23, SS = 5
    // begin(int8_t sck=-1, int8_t miso=-1, int8_t mosi=-1, int8_t ss=-1);
    hspi->begin(18,19,23,33);
    Serial.println("initialise vspi with default pins 3...");

    pinMode(33, OUTPUT); //VSPI SS

    delay(10);


    Serial.println("initialise ledc...");
    ledcSetup(LEDC_CHANNEL_0, 20000, LEDC_TIMER_10_BIT);
    ledcSetup(LEDC_CHANNEL_1, 20000, LEDC_TIMER_10_BIT);
    ledcSetup(LEDC_CHANNEL_2, 20000, LEDC_TIMER_10_BIT);
    ledcSetup(LEDC_CHANNEL_3, 20000, LEDC_TIMER_10_BIT);
    ledcAttachPin(PWM1_PIN, LEDC_CHANNEL_0);
    ledcAttachPin(PWM2_PIN, LEDC_CHANNEL_1);
    ledcAttachPin(PWM3_PIN, LEDC_CHANNEL_2);
    ledcAttachPin(PWM4_PIN, LEDC_CHANNEL_3);

    // We start by connecting to a WiFi network
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    server.begin();

    ArduinoOTA.onStart([]() {
         String type;
         if (ArduinoOTA.getCommand() == U_FLASH)
           type = "sketch";
         else // U_SPIFFS
           type = "filesystem";

         // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
         Serial.println("Start updating " + type);
       });
       ArduinoOTA.onEnd([]() {
         Serial.println("\nEnd");
       });
       ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
         Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
       });
       ArduinoOTA.onError([](ota_error_t error) {
         Serial.printf("Error[%u]: ", error);
         if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
         else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
         else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
         else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
         else if (error == OTA_END_ERROR) Serial.println("End Failed");
       });
       ArduinoOTA.begin();

}

int value = 0;

void checkWifiClient()
{
  WiFiClient client = server.available();   // listen for incoming clients

   if (client) {                             // if you get a client,
     Serial.println("New Client.");           // print a message out the serial port
     String currentLine = "";                // make a String to hold incoming data from the client
     while (client.connected()) {            // loop while the client's connected
       if (client.available()) {             // if there's bytes to read from the client,
         char c = client.read();             // read a byte, then
         Serial.write(c);                    // print it out the serial monitor
         if (c == '\n') {                    // if the byte is a newline character

           // if the current line is blank, you got two newline characters in a row.
           // that's the end of the client HTTP request, so send a response:
           if (currentLine.length() == 0) {
             // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
             // and a content-type so the client knows what's coming, then a blank line:
             client.println("HTTP/1.1 200 OK");
             client.println("Content-type:text/html");
             client.println();

             // the content of the HTTP response follows the header:
             client.print("Click <a href=\"/H\">here</a> to turn the LED on pin 5 on.<br>");
             client.print("Click <a href=\"/L\">here</a> to turn the LED on pin 5 off.<br>");
             client.print("Click <a href=\"/UP\">faster</a> to pwm duty UP<br>");
             client.print("Click <a href=\"/DOWN\">here</a>  to pwm duty DOWN<br>");

             // The HTTP response ends with another blank line:
             client.println();
             // break out of the while loop:
             break;
           } else {    // if you got a newline, then clear currentLine:
             currentLine = "";
           }
         } else if (c != '\r') {  // if you got anything else but a carriage return character,
           currentLine += c;      // add it to the end of the currentLine
         }

         // Check to see if the client request was "GET /H" or "GET /L":
         if (currentLine.endsWith("GET /H")) {
           digitalWrite(33, HIGH);               // GET /H turns the LED on
           digitalWrite(25, HIGH);
           Serial.println("\nHigh.");
         }
         if (currentLine.endsWith("GET /L")) {
           Serial.println("\nSPI Start.");
           //digitalWrite(33, LOW);
           usleep(10);
           // SPI WRITE
           hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
           //byte stuff = 0b11001100;
           //hspi->transfer(stuff);
           digitalWrite(33, LOW);
           toTransfer = 0b11001100 << 8;
           toTransfer |= 0b11001100 ;
           hspi->transfer16(toTransfer);
           digitalWrite(33, HIGH);
           hspi->endTransaction();

           usleep(10);

           //digitalWrite(33, HIGH);

           Serial.println("SPI Done.");
         }
         if (currentLine.endsWith("GET /UP")) {
           if(brightness>=10)
           {
            brightness = brightness - 10;
            ledcAnalogWrite(LEDC_CHANNEL_0, brightness, 1024);
            Serial.print("duty= ");
            Serial.println(brightness);
           }
         }
         if (currentLine.endsWith("GET /DOWN")) {
           if(brightness<=(1024-10))
           {
            brightness = brightness + 10;
            ledcAnalogWrite(LEDC_CHANNEL_0, brightness, 1024);
            Serial.print("duty= ");
            Serial.println(brightness);
          }
         }

       }
     }
     // close the connection:
     client.stop();
     Serial.println("Client Disconnected.");
}
}

void loop(){

/*
  // set the brightness on LEDC channel 0
  ledcAnalogWrite(LEDC_CHANNEL_0, brightness);

  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (brightness <= 0 || brightness >= 1024) {
    fadeAmount = -fadeAmount;
  }
*/
  checkWifiClient();
  ArduinoOTA.handle();


}
