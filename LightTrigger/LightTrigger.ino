/*
 * This program uses the value of a light sensor to enable and disable
 * an external device.
 *
 * On the arduino, connect a photo resister between the 5V output and 
 * an analog pin, and also connect that same analog pin to ground with a
 * 10,000 Ohm resistor.  And digital pin can then be used for the external 
 * trigger.
 *
 *                PhotoR     10K
 * +5V        o---/\/\/--+--/\/\/---o GND
 *                       |
 * Analog 6   o----------+
 * 
 * Digital 12 o-----------------> External trigger
 *
 */

#define DEBUG_LEVEL DEBUG_HIGH
#include "Debug.h"

#define LED_PIN 13

#define PHOTO_PIN A6

#define TRIGGER_PIN 12

/*
 * When the light sensor value falls under ON_THRESHOLD then hit the trigger,
 * release the trigger when it goes above OFF_THRESHOLD.  The separation
 * prevents flickering, and if the trigger is turning on something that
 * produces light this can be used to stop if from turning itself off.
 */
#define ON_THRESHOLD  500
#define OFF_THRESHOLD (ON_THRESHOLD + 50)

/* Period to wait between readings, in milliseconds */
#define PERIOD 250

void setup() {
  Serial.begin(9600);

  pinMode(LED_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(TRIGGER_PIN, LOW);
}

void loop() {
  int value = analogRead(PHOTO_PIN);

  Serial.print("Photo:");
  Serial.println(value);
  
  if (value < ON_THRESHOLD) {
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(TRIGGER_PIN, HIGH);
  } else if (value > OFF_THRESHOLD) {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(TRIGGER_PIN, LOW);
  }

  delay(PERIOD);
}
