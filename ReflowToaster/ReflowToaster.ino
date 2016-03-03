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

int thermoDO = 12;
int thermoCS = 8;
int thermoCLK = 6 ;
Adafruit_MAX31855 thermocouple(thermoCLK, thermoCS, thermoDO);

int trigLED = 10;  // Must be PWM
int heatPin = 9;
   ; // Must be PWM

int debugLED = 13;


double targetTemp, thermocoupleTemp, pidOutput;

// TODO: Figure out proper tuning parameters
//double Kp=2, Ki=5, Kd=1;
//double Kp=1, Ki=0.05, Kd=0.25;
double Kp=1, Ki=0.5, Kd=0.1;
//double Kp=1, Ki=0.25, Kd=0.25;
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

/*
 * Reflow parameters
 */
boolean reflow = false;

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
  targetTemp = 0;
  myPID.SetMode(AUTOMATIC);

  // Configure autotune
  aTune.SetControlType(1); // Set to PID
  aTune.SetLookbackSec(60); // Needs to be high for toasters?
  //TODO: aTune.SetNoiseBand(???);

  DEBUG1_PRINTLN("AMP's Reflow Toaster initialized");
}

void loop() {
  // Check for serial commands
  serialcli.checkSerial();

  // Read the current temperature
  double newTemp = thermocouple.readCelsius();
  if (isnan(newTemp)) {
    DEBUG_ERR("Something wrong with thermocouple!");
  } else {
    thermocoupleTemp = newTemp;
  }

  if (tuning) {
    checkTuning();
  } else {
    if (reflow) {
      // If reflow set appropriate reflow
      adjustReflow();
    }

    // Update pid outout
    myPID.Compute();
  }

  analogWrite(heatPin, pidOutput);
  analogWrite(trigLED, pidOutput);

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
    DEBUG3_VALUE(" Set:", targetTemp);
    DEBUG3_VALUE(" pid:", pidOutput);
    DEBUG3_VALUE(" Kp:", Kp);
    DEBUG3_VALUE(" Ki:", Ki);
    DEBUG3_VALUE(" Kd:", Kd);

    DEBUG3_VALUE(" tuning:", tuning);

    if (reflow) {
      reflow_display();
    }

    DEBUG_PRINT_END();
  }
}



void print_usage() {
  Serial.print(F(
    "\n"
    "Usage:\n"
    "  h - print this help\n"
    "  c <temp>: Set the target temperature in Celsius\n"
    "  a <temp>: Start autotune with target temperature\n"
    "  R:        Start reflow\n"
    "  s:        Stop autotune or reflow\n"
    "  k <p> <i> <d> : Set PID values\n"
    "\n"
  ));
}

/*
 * Usage:
 *   c <temp>: Set the target temperature in Celsius
 *   a <temp>: Start autotune with target temperature
 *   R:        Start reflow
 *   s:        Disable autotune or reflow
 *   k <p> <i> <d> : Set PID values
 */
void cliHandler(char **tokens, byte numtokens) {
  switch (tokens[0][0]) {
    case 'h': {
      print_usage();
      break;
    }

    case 'k': {
      if (numtokens < 4) return;
      double val = atof(tokens[1]);
      Kp = val;
      val = atof(tokens[2]);
      Ki = val;
      val = atof(tokens[3]);
      Kd = val;
      myPID.SetTunings(Kp, Ki, Kd);
      DEBUG1_VALUE("New Kp:", Kp);
      DEBUG1_VALUE("Ki:", Ki);
      DEBUG1_VALUELN("Kd:", Kd);
      break;
    }

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
      if (tuning) {
        DEBUG1_PRINTLN("Disabling autotune");
        changeAutoTune();
      }
      if (reflow) {
        stopReflow();
      }
      break;
    }

    case 'R': {
      if (tuning) {
        DEBUG1_PRINT("Can't start reflow during autotune");
        break;
      }

      if (reflow) {
        DEBUG1_PRINTLN("Reflow is already running");
      } else {
        startReflow();
      }
      break;
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
    DEBUG3_PRINTLN("TUNING ON");
  } else { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
    DEBUG3_PRINTLN("TUNING OFF");
  }
}

void AutoTuneHelper(boolean start) {
  if (start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}

/*
 * Reflow!
 */
typedef struct {
  byte mode;
  unsigned long period_ms;
  int targetTemp;
} reflow_phase_t;


// Ramp up to target temp and then immediately move to next phase
#define MODE_REACH_TEMP 0

// Ramp up to target temp and then hold for the rest of the period
#define MODE_HOLD_TEMP  1

reflow_phase_t reflow_phases[] = {
        { MODE_REACH_TEMP, 90000, 150 }, // Ramp up
        { MODE_HOLD_TEMP,  90000, 200 }, // Soak
        { MODE_REACH_TEMP, 30000, 235 }, // Reflow
        { 3, 60000, 0   }  // Cooldown
};
#define NUM_PHASES (sizeof (reflow_phases) / sizeof (reflow_phase_t))


int current_phase = -1;
reflow_phase_t *phase = NULL;

boolean phase_increasing;
boolean phase_done;

unsigned long phase_start_ms = 0;
unsigned long phase_elapsed_ms = 0;


void startReflow() {
  reflow = true;
  setPhase(0);
  DEBUG1_PRINTLN("REFLOW STARTED");
}

void stopReflow() {
  reflow = false;
  phase = NULL;
  targetTemp = 0;
  DEBUG1_PRINTLN("REFLOW STOPPED");
}

// Set the current phase
void setPhase(int new_phase) {
  current_phase = new_phase;
  if ((current_phase >= NUM_PHASES) || (new_phase < 0)) {
    stopReflow();
    current_phase = -1;
    phase = NULL;
  } else {
    phase = &reflow_phases[current_phase];
    phase_start_ms = millis();
    phase_elapsed_ms = 0;

    phase_increasing = (phase->targetTemp > thermocoupleTemp);
    phase_done = false;

    DEBUG1_VALUE("PHASE SET: ", current_phase);
    DEBUG1_VALUELN(" INCR:", phase_increasing);
  }
}

boolean checkPhase() {
  if (phase_increasing) {
    // Temperature is being increased
    if (thermocoupleTemp >= phase->targetTemp) {
      return true;
    }
  } else {
    if (thermocoupleTemp <= phase->targetTemp) {
      return true;
    }
  }

  return false;
}

void adjustReflow() {
  unsigned long now = millis();
  phase_elapsed_ms = now - phase_start_ms;

  boolean advance_phase = false;

  // Check if the period for this phase has passed
  if (phase_elapsed_ms > phase->period_ms) {
    advance_phase = true;
    //DEBUG1_PRINTLN("PERIOD FINISHED");
  }

  if (checkPhase()) {
    /*
     * If the phase temperature has been reached, the set the phase as done.
     * Depending on the mode we may continue to hold the temperature.
     */
    phase_done = true;
  }

  // Set the temperature
  switch (phase->mode) {
    case MODE_REACH_TEMP: {
      /*
       * For this mode, increase the temperature until it reaches the target and
       * then proceed to the next phase.  The period is ignored in this mode.
       */
      advance_phase = phase_done;

      targetTemp = phase->targetTemp;
      break;
    }

    case MODE_HOLD_TEMP: {
      /*
       * For this mode, increase the temperature until it reaches the target and
       * then hold for any period of time that remains.
       */
      if (!phase_done) {
        advance_phase = false;
      }

      targetTemp = phase->targetTemp;
      break;
    }

    default: {
      targetTemp = phase->targetTemp;
      break;
    }
  }


  // Check if the phase should be advanced
  if (advance_phase) {
    setPhase(current_phase + 1);
  }
}

void reflow_display() {
  DEBUG3_VALUE(" phase:", current_phase);
  if (phase) {
    DEBUG3_VALUE(" mode:", phase->mode);
    DEBUG3_VALUE(" period:", phase->period_ms);
    DEBUG3_VALUE(" temp:", phase->targetTemp);
    DEBUG3_VALUE(" incr:", phase_increasing);
    DEBUG3_VALUE(" done:", phase_done);
  }
  DEBUG3_VALUE(" elapsed:", phase_elapsed_ms);
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
    lcd.print("C    ");

    lcd.setCursor(8, 0);
    lcd.print("Set:");
    lcd.print((int) targetTemp);
    lcd.print("C    ");

    lcd.setCursor(0, 1);

    lcd.print("PID:");
    lcd.print((int) pidOutput);
    lcd.print("    ");

    lcd.setCursor(8, 1);
    if (reflow) {
      lcd.print("Refl:");
      lcd.print(current_phase);
      lcd.print("   ");
    } else if (tuning) {
      lcd.print("Tuning");
      lcd.print("   ");
    } else {
      lcd.print("            ");
    }
  }
}
