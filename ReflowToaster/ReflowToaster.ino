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
#include <PID_AutoTune_v0.h>

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

/*
 * PID autotuning
 */
byte ATuneModeRemember=2;

double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;

boolean tuning = false;

PID_ATune aTune(&thermocoupleTemp, &pidOutput);


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

  if (tuning) {
    checkTuning();
  } else {
    // Update pid outout
    myPID.Compute();
  }

  analogWrite(heatPin, pidOutput);

  update_LCD();
  update_serial();
}

#define SERIAL_REFRESH_PERIOD 500
void update_serial() {
  static unsigned long last_update_ms = millis();

  unsigned long now = millis();
  if (now - last_update_ms > SERIAL_REFRESH_PERIOD) {
    DEBUG3_VALUE("[", now);
    DEBUG3_VALUE("] Internal Temp:", thermocouple.readInternal());
    DEBUG3_VALUE(" C:", thermocoupleTemp);
    DEBUG3_VALUE(" F:", thermocouple.readFarenheit());
    DEBUG3_VALUE(" pid:", pidOutput);
    DEBUG3_VALUE(" Kp:", Kp);
    DEBUG3_VALUE(" Ki:", Ki);
    DEBUG3_VALUE(" Kd:", Kd);

    DEBUG3_VALUE(" tuning:", tuning);

    DEBUG_PRINT_END();
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
 *   a <temp>: Start autotune with target temperature
 *   s:        Disable autotune
 */
void cliHandler(char **tokens, byte numtokens) {
  switch (tokens[0][0]) {
    case 'c': {
      if (numtokens < 2) return;
      int val = atoi(tokens[1]);
      DEBUG1_VALUELN("Setting temp to C=", val);
      targetTemp = val;
      break;
    }

    case 'a': {
      if (numtokens < 2) return;

      if (tuning) {
        DEBUG1_PRINTLN("Autotune is already running");
      } else {
        int val = atoi(tokens[1]);
        DEBUG1_VALUELN("Starting autotune with temp to C=", val);
        targetTemp = val;

        changeAutoTune();
      }
      break;
    }

    case 's': {
      if (!tuning) {
        DEBUG1_PRINTLN("Autotune is not running");
      } else {
        DEBUG1_PRINTLN("Disabling autotune");
        changeAutoTune();
      }
    }

  }
}


/*
 * PID autotuning code:
 *   From https://github.com/br3ttb/Arduino-PID-AutoTune-Library/blob/master/PID_AutoTune_v0/Examples/AutoTune_Example/AutoTune_Example.pde
 */

void checkTuning() {
  int val = aTune.Runtime();
  if (val != 0) {
    tuning = false;
  }

  if (!tuning) {
    //we're done, set the tuning parameters
    Kp = aTune.GetKp();
    Ki = aTune.GetKi();
    Kd = aTune.GetKd();

    DEBUG1_VALUE("Tuning done.  Kp:", Kp);
    DEBUG1_VALUE("Ki:", Ki);
    DEBUG1_VALUE("Kd:", Kd);
    DEBUG_PRINT_END();

    myPID.SetTunings(Kp, Ki, Kd);
    AutoTuneHelper(false);
  }
}

void changeAutoTune()
{
  if (!tuning) {
    //Set the output to the desired starting frequency.
    pidOutput=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  } else { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start) {
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}