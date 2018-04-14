/*
  WiFi Web Server LED Blink

 A simple web server that lets you blink an LED via the web.
 This sketch will print the IP address of your WiFi Shield (once connected)
 to the Serial monitor. From there, you can open that address in a web browser
 to turn on and off the LED on pin 9.

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
#include "Adafruit_HDC1000.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme;
Adafruit_HDC1000 hdc = Adafruit_HDC1000();
HardwareSerial MHZ19Serial(2);                  // RX, TX . You can choose other pins if you prefer.
#define MHZ19SERIAL_RXPIN 16 
#define MHZ19SERIAL_TXPIN 17 

int ledPin = 13;   // onboard LED on GPIO13

const char* sensorName = "se01.bocuse.nl";
const char* ssid = "DLFT";
const char* pass = "gtr5rtg5rtg";
boolean debug = false;
String sensorType = "";

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

  Serial.print("Searching for BME680 sensor...");
  if (!bme.begin()) {
    Serial.println("NOT FOUND!");
  } else {
    Serial.println("FOUND!");
    sensorType += "BME680 ";
    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  }

  Serial.print("Searching for HDC1080 sensor...");
  if (!hdc.begin()) {
    Serial.println("NOT FOUND!");
  } else {
    sensorType += "HDC1080 ";
    Serial.println("FOUND!");
  }

  Serial.print("Searching for MH-Z19 sensor...");
  MHZ19Serial.begin(9600, SERIAL_8N1, MHZ19SERIAL_RXPIN, MHZ19SERIAL_TXPIN);
  delay(1000);
  if (readCO2() != -1) {
    sensorType += "MH-Z19 ";
    Serial.println("FOUND!");
  }
  else {
    Serial.println("NOT FOUND!");
  }

  Serial.println(outputWiFiStatus());
}

float getTemperatureInternalESP32() {
  uint8_t temp_farenheit= temprature_sens_read();
  float temp_celsius = ( temp_farenheit - 32 ) / 1.8;
  return temp_celsius;
}

long readCO2()
{
  while (MHZ19Serial.read()!=-1) {};  //clear serial buffer  

  char response[9]; // for answer
  byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  
  // Command to ask for data.
  MHZ19Serial.write(cmd, 9); //request PPM CO2 

  // Then for 1 second listen for 9 bytes of data.
  MHZ19Serial.readBytes(response, 9);

  if (debug) {
    Serial.print(response[0], HEX);   
    Serial.print(" - ");   
    Serial.print(response[1], HEX);   
    Serial.print(" - ");    
    Serial.print(response[2], HEX);   
    Serial.print(" - ");          
    Serial.print(response[3], HEX);   
    Serial.print(" - ");          
    Serial.print(response[4], HEX);   
    Serial.print(" - ");          
    Serial.print(response[5], HEX);   
    Serial.print(" - ");        
    Serial.print(response[6], HEX);   
    Serial.print(" - ");   
    Serial.print(response[7], HEX);   
    Serial.print(" - ");      
    Serial.print(response[8], HEX); 
    Serial.println(" - END");
  }

  if (response[0] != 0xFF) {
    Serial.println("Wrong starting byte from co2 sensor! (should be FF)");
    return -1;
  }

  if (response[1] != 0x86) {
    Serial.println("Wrong command from co2 sensor! (should be 86)");
    return -1;
  }

  int responseHigh = (int) response[2];
  int responseLow = (int) response[3];
  int ppm = (256 * responseHigh) + responseLow;
  
  return ppm;
}

String outputWiFiStatus() {
  String out = "";
  out += "\nSensor name            : ";
  out += sensorName;
  out += "\nSensor type(s)         : ";
  out += sensorType;
  out += "\n\nIP Address             : ";
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
  out += "\n";
  return out;
}

String outputInfluxdbFormat(float averageT1, float averageT2, float averageH1, float averageP1 = -1, float averageG1 = -1, float averageC1 = -1) {
  // Creating out like: HomeSensors,sensor=se01,uomT1="",descriptionT1="internal",uomT2="",descriptionT2="" averageT1=12.02,averageT2=12.02 timestamp

  String out = "HomeSensors,sensor=";
  out += sensorName;
  out += ",uomT1=°C,descriptionT1=\"internal\"";
  out += ",uomT2=°C,descriptionT2=\"external\"";
  out += ",uomH1=%,descriptionH1=\"external\"";
  if (averageP1 != -1) {
    out += ",uomP1=hPa,descriptionP1=\"external\"";
  }
  if (averageG1 != -1) {
    out += ",uomG1=KOhms,descriptionG1=\"external\"";
  }
  if (averageC1 != -1) {
    out += ",uomC1=PPM,descriptionG1=\"external\"";
  }
  out += " ";
  out += "averageT1=";
  out += String(round(averageT1 * 10.0) / 10.0, 1);  //Round and display one digit
  out += ",averageT2=";
  out += String(round(averageT2 * 10.0) / 10.0, 1);
  out += ",averageH1=";
  out += String(round(averageH1 * 10.0) / 10.0, 1);
  if (averageP1 != -1) {
    out += ",averageP1=";
    out += String(round(averageP1 * 10.0) / 10.0, 1);
  }
  if (averageG1 != -1) {
    out += ",averageG1=";
    out += String(round(averageG1 * 10.0) / 10.0, 1);
  }
  if (averageC1 != -1) {
    out += ",averageC1=";
    out += String(round(averageC1));
  }
  return out;
}

String outputWeerschipFormat(float averageT1, float averageT2, float averageH1, float averageP1 = -1, float averageG1 = -1, float averageC1 = -1) {
  // Creating out like: T1|T2|P1|H1|G1

  String out = "";
  out += String(round(averageT1* 10.0) / 10.0, 1);  //Round and display one digit
  out += "|";
  out += String(round(averageT2* 10.0) / 10.0, 1);
  out += "|";
  out += String(round(averageH1* 10.0) / 10.0, 1);
  if (averageP1 != -1) {  
    out += "|";
    out += String(round(averageP1* 10.0) / 10.0, 1);
  }
  if (averageG1 != -1) {  
    out += "|";
    out += String(round(averageG1* 10.0) / 10.0, 1);
  }
  if (averageC1 != -1) {  
    out += "|";
    out += String(round(averageC1));
  }
  return out;
}

String outputWeerschipSampleFormat(float sampleT1, float sampleT2, float sampleH1, float sampleP1 = -1, float sampleG1 = -1, long sampleC1 = -1) {
  // Creating out like: T1|T2|H1 optional P1|G1|C1

  String out = "";
  out += String(sampleT1, 4);  //Display four digits
  out += "|";
  out += String(sampleT2, 4);
  out += "|";
  out += String(sampleH1, 4);
  if (sampleP1 != -1) {
    out += "|";
    out += String(sampleP1, 4);
  }
  if (sampleG1 != -1) {
    out += "|";
    out += String(sampleG1, 4);
  }
  if (sampleC1 != -1) {
    out += "|";
    out += String(sampleC1);
  }
  return out;
}

String outputWeerschipQuantity(float sampleT1, float sampleT2, float sampleH1, float sampleP1 = -1, float sampleG1 = -1, long sampleC1 = -1) {
  // Creating out like: T1|T2|H1 optional P1|G1
  String out = "";
  out += "T1|T2|H1";
  if (sampleP1 != -1) {
    out += "|P1";
  }
  if (sampleG1 != -1) {
    out += "|G1";
  }
  if (sampleC1 != -1) {
    out += "|C1";
  }
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
float sampleT1, sampleT2, sampleH1, sampleP1 = -1, sampleG1 = -1;
long sampleC1 = -1;
float summaryT1, summaryT2, summaryH1, summaryP1, summaryG1, summaryC1;
float averageT1, averageT2, averageH1, averageP1 = -1, averageG1 = -1, averageC1 = -1;

void loop(){
  stap12 += 1;
  if (debug) {
      Serial.print("Stap12 = ");
      Serial.println(stap12);
  }

  sampleT1 = getTemperatureInternalESP32();
  summaryT1 += sampleT1;
  averageT1 = summaryT1 / stap12;

  if (sensorType.indexOf("BME680") != -1) {
    if (! bme.performReading()) {
      Serial.println("ERROR failed reading from BME680 check hardware");
    } else {
      sampleT2 = bme.temperature;
      summaryT2 += sampleT2;
      averageT2 = summaryT2 / stap12;

      sampleH1 = bme.humidity;
      summaryH1 += sampleH1;
      averageH1 = summaryH1 / stap12;

      sampleP1 = bme.pressure / 100;
      summaryP1 += sampleP1;
      averageP1 = summaryP1 / stap12;
    
      sampleG1 = bme.gas_resistance / 1000.0;
      summaryG1 += sampleG1;
      averageG1 = summaryG1 / stap12;
    }
  }

  if (sensorType.indexOf("HDC1080") != -1) {
      sampleT2 = hdc.readTemperature();
      summaryT2 += sampleT2;
      averageT2 = summaryT2 / stap12;
    
      sampleH1 = hdc.readHumidity();
      summaryH1 += sampleH1;
      averageH1 = summaryH1 / stap12;
  }

  if (sensorType.indexOf("MH-Z19") != -1) {
    sampleC1 = readCO2();
    if (sampleC1 != -1 ) {
      summaryC1 += sampleC1;
      averageC1 = summaryC1 / stap12;
      Serial.println("Average Co2 - PPM = " + String(averageC1));
    }
    else {
      Serial.println("ERROR failed reading from MH-Z19 check hardware");
    }

    Serial.println("Co2 - PPM = " + String(sampleC1));
  }


  //Flash GPIO5 led measurement taken
  digitalWrite(ledPin, HIGH);
  delay(10);
  digitalWrite(ledPin, LOW);

  if (debug) {
    Serial.println(outputInfluxdbFormat(averageT1, averageT2, averageH1, averageP1, averageG1, averageC1));
  }
    
  if (stap12 > 70) {
     //cyclus10();
     stap12 = 0;
     summaryT1 = 0; summaryT2 = 0; summaryH1 = 0; summaryP1 = 0; summaryG1 = 0; summaryC1 = 0;
     Serial.println("INFO Reset the average counter after 70 intervals of 12 seconds");
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

            if (request == "/influxdb.txt")  {
                //send header200
                client.print(outputInfluxdbFormat(averageT1, averageT2, averageH1, averageP1, averageG1, averageC1));
                Serial.print("Going to reset the average counter at stap12 = ");
                Serial.println(stap12);
                stap12 = 0; summaryT1 = 0; summaryT2 = 0; summaryH1 = 0; summaryP1 = 0; summaryG1 = 0; summaryC1 = 0;
            } else if (request == "/") {
                client.println(outputWiFiStatus());
                client.print("Data format            : ");
                client.println(outputWeerschipQuantity(averageT1, averageT2, averageH1, averageP1, averageG1, averageC1));
                client.print("Data averages          : ");
                client.println(outputWeerschipFormat(averageT1, averageT2, averageH1, averageP1, averageG1, averageC1));
                client.print("Data last sample raw   : ");
                client.println(outputWeerschipSampleFormat(sampleT1, sampleT2, sampleH1, sampleP1, sampleG1, sampleC1));
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

