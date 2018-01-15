/*
  WiFi Web Server LED Blink

 A simple web server that lets you blink an LED via the web.
 This sketch will print the IP address of your WiFi Shield (once connected)
 to the Serial monitor. From there, you can open that address in a web browser
 to turn on and off the LED on pin 9.

 If the IP address of your shield is yourAddress:
 http://yourAddress/H turns the LED on
 http://yourAddress/L turns it off

 This example is written for a network using WPA encryption. For
 WEP or WPA, change the Wifi.begin() call accordingly.

 Circuit:
 * WiFi shield attached
 * LED attached to pin 5

 Error codes from WiFiType.h:
 WL_NO_SHIELD        = 255,   // for compatibility with WiFi Shield library
 WL_IDLE_STATUS      = 0,
 WL_NO_SSID_AVAIL    = 1,
 WL_SCAN_COMPLETED   = 2,
 WL_CONNECTED        = 3,
 WL_CONNECT_FAILED   = 4,
 WL_CONNECTION_LOST  = 5,
 WL_DISCONNECTED     = 6

 created 25 Nov 2012
 by Tom Igoe
 */
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

/*=========================================================================
 I2C ADDRESS/BITS/SETTINGS
 -----------------------------------------------------------------------*/
#define BMP280_ADDRESS                (0x76)
#define BMP280_CHIPID                 (0x58)
/*=========================================================================*/

Adafruit_BMP280 bmp; // I2C

const char* name = "sensor01";      // 
const char* ssid = "DLFT";          // your network SSID (name)
const char* pass = "gtr5rtg5rtg";   // your network password
boolean debug = false;

#ifdef __cplusplus
extern "C" {
#endif

uint8_t temprature_sens_read();
//uint8_t g_phyFuns;

#ifdef __cplusplus
}
#endif

WiFiServer server(80);

void setup()
{
    esp_log_level_set("*", ESP_LOG_VERBOSE);
    Serial.begin(115200);
    pinMode(5, OUTPUT);      // set the LED pin mode
    delay(10);

    // We start by connecting to a WiFi network
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.disconnect(true);
    delay(1000);
    WiFi.enableSTA(true);   //Without this sometimes blocked when arriving at WiFi.begin ()
    WiFi.onEvent(WiFiEvent);
    WiFi.begin(ssid, pass);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("connected!");

    server.begin();

    if (!bmp.begin(BMP280_ADDRESS, BMP280_CHIPID)) {  
      Serial.println(F("Could not find a valid BMP280 sensor, check address and wiring!"));
    }
}

void printWifiStatus() {
  Serial.print("Sensor name: ");
  Serial.println(name);
 
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  String mac = WiFi.macAddress();
  Serial.print("MAC Address: ");
  Serial.println(mac);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  int channel = WiFi.channel();
  Serial.print("channel: ");
  Serial.println(channel);
  
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

float getTemperatureInternal() {
  uint8_t temp_farenheit= temprature_sens_read();
  float temp_celsius = ( temp_farenheit - 32 ) / 1.8;
  return temp_celsius;
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d  - ", event);
  switch(event) {
  case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      printWifiStatus();
      break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      WiFi.begin();
      break;
  case SYSTEM_EVENT_STA_START:
      Serial.println("ESP32 station start");
      break;
  case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("ESP32 station connected to AP");
      break;
  }
}

//Global variables
int value = 0;
byte stap12 = 0;                // teller voor het aantal 12 seconden perioden
float temperatureSample, temperatureSum, temperatureAverage;

void loop(){
  stap12 += 1;
  if (debug) {
      Serial.print("Stap12 = ");
      Serial.println(stap12);
  }

  temperatureSample = getTemperatureInternal();
  temperatureSum += temperatureSample; 
  temperatureAverage = temperatureSum / stap12;
  Serial.print("CPU temperature sample = ");
  Serial.print(temperatureSample);
  Serial.print(" average = ");
  Serial.println(temperatureAverage);
  Serial.print("External sensor1 temperature sample = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
  Serial.print("External sensor1 pressure sample = ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  if (stap12 > 70) {
     //cyclus10();
     stap12 = 0;
     temperatureSum = 0;
     Serial.println("Reset the average counter after 70 intervals of 12 seconds");
  }

  while (millis() % 12000 < 11500) {
     if (debug) {
        Serial.print("DEBUG Inloop!!!");
        Serial.println(millis() % 12000);
     }
     opvragen(); //Do WiFi
     delay(500); //Change into deep-sleep??
  }
  delay(500);
}

void opvragen() {
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("New Client.");          // print a message out the serial port
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
            client.println("Content-type:text/plain");
            client.println("Cache-Control: no-cache");
            client.println();

            // the content of the HTTP response follows the header:
            //client.print("Click <a href=\"/H\">here</a> to turn the LED on pin 5 on.<br>");
            //client.print("Click <a href=\"/L\">here</a> to turn the LED on pin 5 off.<br>");

            client.print("Temperature internal sensor in celsius: ");
            client.println(temperatureAverage);
            client.print("Temperature external sensor in celsius: ");
            client.println(bmp.readTemperature());
            
            Serial.print("Going to reset the average counter at stap12 = ");
            Serial.println(stap12);
            stap12 = 0;
            temperatureSum = 0;
            Serial.println("Reset stap12 done");
                        
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
          digitalWrite(5, HIGH);               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(5, LOW);                // GET /L turns the LED off
        }
        if (currentLine.endsWith("GET /status.txt")) {
          //client.print(statusPage());
        }
      }
    }
    // close the connection:
    delay(1);
    client.stop();
    Serial.println("Client Disconnected.");
  }
}

