#include <WiFi.h>
#include <SPI.h>

const char* ssid     = "***";
const char* password = "***";

WiFiServer server(80);

static const int spiClk = 1000000; // 1 MHz
uint16_t toTransfer;

//uninitalised pointers to SPI objects
SPIClass * hspi = NULL;

void setup()
{


    Serial.begin(115200);
    pinMode(33, OUTPUT);
    pinMode(25, OUTPUT);

    //initialise vspi with default pins
    Serial.println("initialise vspi with default pins 1...");
    hspi = new SPIClass(HSPI);
    Serial.println("initialise vspi with default pins 2...");
    hspi->begin(5,6,7,8);
    // VSPI - SCLK = 18, MISO = 19, MOSI = 23, SS = 5
    // begin(int8_t sck=-1, int8_t miso=-1, int8_t mosi=-1, int8_t ss=-1);
    hspi->begin(18,19,23,5);
    Serial.println("initialise vspi with default pins 3...");

    delay(10);

    // We start by connecting to a WiFi network

    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    server.begin();

}

int value = 0;

void loop(){
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
          Serial.println("High.");
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(33, LOW);                // GET /L turns the LED off


          // SPI WRITE
          hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
          digitalWrite(33, LOW);
          //byte stuff = 0b11001100;
          //hspi->transfer(stuff);


          toTransfer = 0b11001100 << 8;
          toTransfer |= 0b11001100 ;
          hspi->transfer16(toTransfer);

          digitalWrite(15, HIGH);
          hspi->endTransaction();

          Serial.println("Done.");
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }
}
