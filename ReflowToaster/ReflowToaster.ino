/*
 * Adam Phelps
 *
 * Reflow Toaster Control code
 */

#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL DEBUG_HIGH
#endif
#include <Debug.h>

#include <PID_v1.h>
#include <Adafruit_MAX31855.h>
#include <Wire.h>
#include <SerialCLI.h>
#include <SPI.h>

// TODO: Look at LiquidTWI2
#include <LiquidTWI.h>
LiquidTWI lcd(0);

int thermoDO = 3;
int thermoCS = 5;
int thermoCLK = 6;
Adafruit_MAX31855 thermocouple(thermoCLK, thermoCS, thermoDO);

int trigLED = 9;
int heatPin = 12;
int debugLED = 13;


double targetTemp, thermocoupleTemp, pidOutput;

// TODO: Figure out proper tuning parameters
//double Kp=2, Ki=5, Kd=1;
//double Kp=1, Ki=0.05, Kd=0.25;
double Kp=1, Ki=1, Kd=0.1;
PID myPID(&thermocoupleTemp, &pidOutput, &targetTemp, Kp, Ki, Kd, DIRECT);

SerialCLI serialcli(128, cliHandler);

void setup() {
  Serial.begin(9600);
  
  pinMode(trigLED, OUTPUT);
  pinMode(heatPin, OUTPUT);
  pinMode(debugLED, OUTPUT);
  
  // Setup the LCD display
  lcd.begin(16, 2);
  lcd.clear();

  // Print a message to the LCD.
  lcd.print("AMP Reflow!");
  lcd.setCursor(0, 1);
  
  // wait for MAX chip to stabilize
  delay(500);

  // Initialized the PID controller and turn it on
  thermocoupleTemp = thermocouple.readCelsius();
  targetTemp = 100;
  myPID.SetMode(AUTOMATIC);

  DEBUG1_PRINTLN("AMP's Reflow Toaster initialized");
}

void loop() {
  // Check for serial commands
  serialcli.checkSerial();

  // Read the current temperature
  thermocoupleTemp = thermocouple.readCelsius();
  if (isnan(thermocoupleTemp)) {
    DEBUG_ERR("Something wrong with thermocouple!");
    thermocoupleTemp = -0;
  }

  // Update pid outout
  myPID.Compute();
  analogWrite(heatPin, pidOutput);

  update_LCD();
  update_serial();
}

#define SERIAL_REFRESH_PERIOD 250
void update_serial() {
  static unsigned long last_update_ms = millis();

  unsigned long now = millis();
  if (now - last_update_ms > SERIAL_REFRESH_PERIOD) {
    DEBUG3_VALUE("[", now);
    DEBUG3_VALUE("] Internal Temp:", thermocouple.readInternal());
    DEBUG3_VALUE(" C:", thermocoupleTemp);
    DEBUG3_VALUE(" F:", thermocouple.readFarenheit());
    DEBUG3_VALUELN(" pid:", pidOutput);
  }
}

#define LCD_REFRESH_PERIOD 100
void update_LCD() {
  static unsigned long last_update_ms = millis();

  unsigned long now = millis();
  if (now - last_update_ms > LCD_REFRESH_PERIOD) {
    last_update_ms = now;
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print((int) thermocoupleTemp);
    lcd.print("C Set:");
    lcd.print((int) targetTemp);
    lcd.print("C   ");

    lcd.setCursor(0, 1);

    lcd.print("PID output: ");
    lcd.print((int) pidOutput);
    lcd.print("    ");
  }
}

/*
 * Usage:
 *   c <temp>: Set the target temperature in Celsius
 */
void cliHandler(char **tokens, byte numtokens) {
  switch (tokens[0][0]) {
    case 'c': {
      if (numtokens < 2) return;
      int val = atoi(tokens[1]);
      Serial.print("Setting temp to C=");
      Serial.println(val);
      targetTemp = val;
      break;
    }
  }
}
