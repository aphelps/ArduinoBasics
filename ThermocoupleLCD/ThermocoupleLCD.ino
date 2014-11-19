/*************************************************** 
  This is an example for the Adafruit Thermocouple Sensor w/MAX31855K

  Designed specifically to work with the Adafruit Thermocouple Sensor
  ----> https://www.adafruit.com/products/269

  These displays use SPI to communicate, 3 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_MAX31855.h"
#include "Wire.h"
#include "SerialCLI.h"

//#define ADAFRUIT_LIQUID
#if ADAFRUIT_LIQUID
  #include "LiquidCrystal.h"
  //LiquidCrystal lcd(0);
#else
  // This claims to be much faster: http://forums.adafruit.com/viewtopic.php?f=19&t=21586&p=113177
  #include <LiquidTWI.h>
  LiquidTWI lcd(0);
#endif

int thermoDO = 2;
int thermoCS = 3;
int thermoCLK = 6;

int trigLED = 9;
int heatPin = 12;
int debugLED = 13;

int targetF = 300;

Adafruit_MAX31855 thermocouple(thermoCLK, thermoCS, thermoDO);
  // Connect via i2c, default address #0 (A0-A2 not jumpered)

SerialCLI serialcli(128, cliHandler);

void setup() {
  Serial.begin(9600);

  
  pinMode(trigLED, OUTPUT);
  pinMode(heatPin, OUTPUT);
  pinMode(debugLED, OUTPUT);
  
  // set up the LCD's number of rows and columns: 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("hello, world!");
  lcd.setCursor(0, 1);

  
  Serial.println("MAX31855 test");
  // wait for MAX chip to stabilize
  delay(500);
}

void loop() {
  serialcli.checkSerial();

  // basic readout test, just print the current temp
  Serial.print("Internal Temp = ");
  Serial.print(thermocouple.readInternal());

   double c = thermocouple.readCelsius();
   if (isnan(c)) {
     Serial.println("Something wrong with thermocouple!");
     c = -0;
   } else {
     Serial.print(" C = "); 
     Serial.print(c);
   }
   Serial.print(" F = ");

   int tempF = thermocouple.readFarenheit();
   Serial.println(tempF);

   if (tempF > 75) {
     digitalWrite(debugLED, HIGH);
     digitalWrite(trigLED, HIGH);
   } else {
     digitalWrite(debugLED, LOW);
     digitalWrite(trigLED, LOW);
   }

   lcd.clear();
   lcd.setCursor(0, 0);  
   lcd.print("Temp F:");
   lcd.print(tempF);
   lcd.print(" C:");
   lcd.print((int)c);

   lcd.setCursor(0, 1);
   if (tempF <= targetF) {
     digitalWrite(heatPin, HIGH);
     lcd.print("Heat is ON!");
   } else {
     digitalWrite(heatPin, LOW);
     lcd.print("Heat is off");
   }
   
   delay(1000);
}

void cliHandler(char **tokens, byte numtokens) {
  switch (tokens[0][0]) {
    case 'f': {
      if (numtokens < 2) return;
      int val = atoi(tokens[1]);
      Serial.print("Setting temp to F=");
      Serial.println(val);
      targetF = val;
      break;
    }
  
  }
}
