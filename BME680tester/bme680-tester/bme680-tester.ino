/***************************************************************************
  This is a library for the BME680 gas, humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/3660

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

Adafruit_BME680 bme; // I2C

int ledPin = 13;                 // onboard LED op pin 13

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("BME680 test"));
  
  pinMode(ledPin, OUTPUT);
  
  Wire.begin();      
  delay(1000);
  
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop() {
  digitalWrite(ledPin, HIGH); delay(10); digitalWrite(ledPin, LOW);
  
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("T = ");
  Serial.print(bme.temperature, 1);
//  Serial.print(" *C"); 

  Serial.print("   RH = ");
  Serial.print(bme.humidity, 1);
//  Serial.print(" %");

  Serial.print("   P = ");
  Serial.print((bme.pressure / 100), 1);
//  Serial.print(" hPa");

  Serial.print("   Gas = ");
  Serial.println((bme.gas_resistance / 1000), 1);
//  Serial.println(" KOhms");

//  Serial.println();
  delay(10000);
}
