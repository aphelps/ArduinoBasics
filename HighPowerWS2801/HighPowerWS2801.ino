#include "FastLED.h"

// How many leds in your strip?
#define NUM_LEDS 1

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
#define DATA_PIN 8
#define CLOCK_PIN 12

#define DEBUG_RED 11
#define DEBUG_GREEN 13
#define DEBUG_BLUE 10

#define DELAY 2000

// Define the array of leds
CRGB leds[NUM_LEDS];

void setup() { 
  Serial.begin(9600);

  //FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.addLeds<WS2801, DATA_PIN, CLOCK_PIN, BGR>(leds, NUM_LEDS);
  FastLED.setBrightness(64);

  pinMode(DEBUG_RED, OUTPUT);
}

void loop() { 
  // Turn the LED on, then pause
  Serial.println("RED");
  leds[0] = CRGB::Red;
  FastLED.show();
  digitalWrite(DEBUG_RED, HIGH);
  delay(DELAY);
  digitalWrite(DEBUG_RED, LOW);
  
  // Now turn the LED off, then pause
  Serial.println("GREEN");
  leds[0] = CRGB::Green;
  FastLED.show();
  digitalWrite(DEBUG_GREEN, HIGH);
  delay(DELAY);
  digitalWrite(DEBUG_GREEN, LOW);

  Serial.println("BLUE");
  leds[0] = CRGB::Blue;
  FastLED.show();
  digitalWrite(DEBUG_BLUE, HIGH);
  delay(DELAY);
  digitalWrite(DEBUG_BLUE, LOW);

#if 0
  Serial.println("WHITE");
  leds[0] = CRGB::White;
  FastLED.show();
  digitalWrite(DEBUG_RED, HIGH);
  digitalWrite(DEBUG_GREEN, HIGH);
  digitalWrite(DEBUG_BLUE, HIGH);
  delay(DELAY);
  digitalWrite(DEBUG_RED, LOW);
  digitalWrite(DEBUG_GREEN, LOW);
  digitalWrite(DEBUG_BLUE, LOW);
#endif
}
