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

#define PHOTO1_PIN 1
#define PHOTO2_PIN 4

#define USE_ADC 1

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
  digitalWrite(LED_PIN, LOW);

#if USE_ADC
  setupADC();
#endif

}


volatile uint16_t value1 = -1;
volatile uint16_t value2 = -1;

void loop() {

#ifdef USE_ADC
  while (ADCSRA & _BV(ADIE)) { // Check if audio sampling has finished
  }
  ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
#else
  value1 = analogRead(PHOTO1_PIN);
  value2 = analogRead(PHOTO2_PIN);
#endif

  Serial.print("Photo:");
  Serial.print(value1);
  Serial.print(",");
  Serial.print(value2);
  Serial.print(" - ADMUX:");
  Serial.println(ADMUX, HEX);
  
  if (value1 < ON_THRESHOLD) {
    digitalWrite(LED_PIN, HIGH);
  } else if (value1 > OFF_THRESHOLD) {
    digitalWrite(LED_PIN, LOW);
  }

  delay(PERIOD);
}

byte current_pin = PHOTO1_PIN;

void setupADC() {
  ADMUX  = bit (REFS0) | current_pin; // Channel sel, right-adj, use 5V

  ADCSRA = _BV(ADEN)  | // ADC enable
           _BV(ADSC)  | // ADC start
           _BV(ADATE) | // Auto trigger
           _BV(ADIE)  | // Interrupt enable
           _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128:1 / 13 = 9615 Hz

  ADCSRB = 0;                // Free run mode, no high MUX bit
  DIDR0  = 1 << PHOTO1_PIN | 1 << PHOTO2_PIN; // Turn off digital input for ADC pin
  TIMSK0 = 0;                // Timer0 off

  sei(); // Enable interrupts
}

ISR(ADC_vect) {
  int16_t              sample         = ADC; // 0-1023

  ADCSRA &= ~_BV(ADIE); // Buffer full, interrupt off

  if (current_pin == PHOTO1_PIN) {
    value1 = sample;
    current_pin = PHOTO2_PIN;
  } else {
    value2 = sample;
    current_pin = PHOTO1_PIN;
  }
  ADMUX  = bit (REFS0) | current_pin;
}

