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
#include "Adafruit_BME680.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C

int ledPin = 5;   // onboard GPIO5

const char* sensorName = "se02.bocuse.nl";      // 
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
    if (debug) {
      esp_log_level_set("*", ESP_LOG_VERBOSE);
    }
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);

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
    }

    server.begin();

  Serial.print("During setup searching BME680 sensor...");
  if (!bme.begin()) {
    Serial.println("ERROR!");
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
  } else {
    Serial.println("FOUND!");
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  }
}

float getTemperatureInternal() {
  uint8_t temp_farenheit= temprature_sens_read();
  float temp_celsius = ( temp_farenheit - 32 ) / 1.8;
  return temp_celsius;
}

String outputWiFiStatus() {
  String out = "";
  out += "\nSensor name            : ";
  out += sensorName;
  out += "\nIP Address             : ";
  out += WiFi.localIP().toString();
  out += "\nMAC Address            : ";
  out += WiFi.macAddress();
  out += "\nSSID                   : ";
  out += WiFi.SSID();
  out += "\nSignal strength (RSSI) : ";
  out += WiFi.RSSI();
  out += " dBm";
  out += "\nChannel                : ";
  out += WiFi.channel();
  return out;
}

String outputInfluxdbFormat(float averageT1, float averageT2, float averageP1, float averageH1, float averageG1) {
  // Creating out like: HomeSensors,sensor=se01,uomT1="",descriptionT1="internal",uomT2="",descriptionT2="" averageT1=12.02,averageT2=12.02 timestamp

  String out = "HomeSensors,sensor=";
  out += sensorName;
  out += ",uomT1=°C,descriptionT1=\"internal\"";
  out += ",uomT2=°C,descriptionT2=\"external\"";
  out += ",uomP1=hPa,descriptionP1=\"external\"";
  out += ",uomH1=%,descriptionH1=\"external\"";
  out += ",uomG1=KOhms,descriptionG1=\"external\"";
  out += " ";
  out += "averageT1=";
  out += String(round(averageT1* 10.0) / 10.0, 1);  //Round and display one digit
  out += ",averageT2=";
  out += String(round(averageT2* 10.0) / 10.0, 1);
  out += ",averageP1=";
  out += String(round(averageP1* 10.0) / 10.0, 1);
  out += ",averageH1=";
  out += String(round(averageH1* 10.0) / 10.0, 1);
  out += ",averageG1=";
  out += String(round(averageG1* 10.0) / 10.0, 1);
  return out;
}

String outputWeerschipFormat(float averageT1, float averageT2, float averageP1, float averageH1, float averageG1) {
  // Creating out like: T1|T2|P1|H1|G1

  String out = "";
  out += String(round(averageT1* 10.0) / 10.0, 1);  //Round and display one digit
  out += "|";
  out += String(round(averageT2* 10.0) / 10.0, 1);
  out += "|";
  out += String(round(averageP1* 10.0) / 10.0, 1);
  out += "|";
  out += String(round(averageH1* 10.0) / 10.0, 1);
  out += "|";
  out += String(round(averageG1* 10.0) / 10.0, 1);

  return out;
}

String outputWeerschipSampleFormat(float sampleT1, float sampleT2, float sampleP1, float sampleH1, float sampleG1) {
  // Creating out like: T1|T2|P1

  String out = "";
  out += String(sampleT1, 4);  //Display four digits
  out += "|";
  out += String(sampleT2, 4);
  out += "|";
  out += String(sampleP1, 4);
  out += "|";
  out += String(sampleH1, 4);
  out += "|";
  out += String(sampleG1, 4);

  return out;
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d  - ", event);
  switch(event) {
  case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println(outputWiFiStatus());
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
float sampleT1, sampleT2, sampleP1, sampleH1, sampleG1;
float summaryT1, summaryT2, summaryP1, summaryH1, summaryG1;
float averageT1, averageT2, averageP1, averageH1, averageG1;

void loop(){
  stap12 += 1;
  if (debug) {
      Serial.print("Stap12 = ");
      Serial.println(stap12);
  }

  sampleT1 = getTemperatureInternal();
  summaryT1 += sampleT1;
  averageT1 = summaryT1 / stap12;

  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
  } else {
    sampleT2 = bme.temperature;
    summaryT2 += sampleT2;
    averageT2 = summaryT2 / stap12;
  
    sampleP1 = bme.pressure / 100;
    summaryP1 += sampleP1;
    averageP1 = summaryP1 / stap12;
  
    sampleH1 = bme.humidity;
    summaryH1 += sampleH1;
    averageH1 = summaryH1 / stap12;
  
    sampleG1 = bme.gas_resistance / 1000.0;
    summaryG1 += sampleG1;
    averageG1 = summaryG1 / stap12;
  }

  //Flash GPIO5 led measurement taken
  digitalWrite(ledPin, HIGH);
  delay(10);
  digitalWrite(ledPin, LOW);

  if (debug) {
    Serial.println(outputInfluxdbFormat(averageT1, averageT2, averageP1, averageH1, averageG1));
  }
  
  if (stap12 > 70) {
     //cyclus10();
     stap12 = 0;
     summaryT1 = 0; summaryT2 = 0; summaryP1 = 0; summaryH1 = 0; summaryG1 = 0;
     Serial.println("Reset the average counter after 70 intervals of 12 seconds");
  }

  while (millis() % 12000 < 11750) {
     if (debug) {
        Serial.print("DEBUG Inloop: ");
        Serial.println(millis() % 12000);
     }
     opvragen(); //Do WiFi
     delay(250); //Change into deep-sleep later to realy power down
  }
  delay(250);
}

void opvragen() {
  String request = "";
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("New Client.");          // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        if (debug) {
          Serial.write(c);                  // print it out the serial monitor
        }

        if (c == '\n' && currentLine.length() == 0) {                    // if the byte is a newline character
          // if you've gotten to the end of the line (received a newline
          // character) and the line is blank, the http request has ended,
          // so you can send a reply
               
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type: text/plain; charset=utf-8");
            client.println("Cache-Control: no-cache");
            client.println();

            if (request == "/data.txt") {
                //send header200
                client.print(outputWeerschipFormat(averageT1, averageT2, averageP1, averageH1, averageG1));
                Serial.print("Going to reset the average counter at stap12 = ");
                Serial.println(stap12);
                stap12 = 0; summaryT1 = 0; summaryT2 = 0; summaryP1 = 0; summaryH1 = 0; summaryG1 = 0;
            } else if (request == "/influxdb.txt")  {
                //send header200
                client.print(outputInfluxdbFormat(averageT1, averageT2, averageP1, averageH1, averageG1));
                Serial.print("Going to reset the average counter at stap12 = ");
                Serial.println(stap12);
                stap12 = 0; summaryT1 = 0; summaryT2 = 0; summaryP1 = 0; summaryH1 = 0; summaryG1 = 0;
            } else if (request == "/") {
                client.println(outputWiFiStatus());
                client.println();
                client.print("Sensor type            : ");
                client.println();
                client.print("Sensor status          : ");
                client.println();
                client.println();
                client.print("Data format            : T1|T2|P1|H1|G1");
                client.println();
                client.print("Data averages          : ");
                client.println(outputWeerschipFormat(averageT1, averageT2, averageP1, averageH1, averageG1));
                client.print("Data last sample raw   : ");
                client.println(outputWeerschipSampleFormat(sampleT1, sampleT2, sampleP1, sampleH1, sampleG1));
                client.println();
                client.print("Data Weerschip format  : http://");
                client.print(WiFi.localIP().toString());
                client.println("/data.txt");
                client.print("Data InfluxDB format   : http://");
                client.print(WiFi.localIP().toString());
                client.println("/influxdb.txt");
                client.println();
            }
            else {
                //send header404 Not FOUND
                //client.println("NOT FOUND");
                Serial.println("ERROR 404: Page NOT FOUND");
            }
            
            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
        }
        if (c == '\n') {   
          // if you got a newline, then clear currentLine:
            currentLine = "";
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
          // Check to see if the client request was
          if (currentLine.endsWith("GET /data.txt ")) {
              Serial.println("GET /data.txt request received");
              request = "/data.txt";
          } else if (currentLine.endsWith("GET /influxdb.txt ")) {
              Serial.println("GET /influxdb.txt request received");
              request = "/influxdb.txt";         
          } else if (currentLine.endsWith("GET / ")) {
              Serial.println("GET / request received");
              request = "/";
          } else {
              //ERROR URL not found
          }
       }       
      }
    }
    // close the connection:
    delay(1);
    client.stop();
    Serial.println("Client Disconnected.");
  }
}

