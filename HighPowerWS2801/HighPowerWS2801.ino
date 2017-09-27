#include "FastLED.h"

#define NUM_LEDS 4

#define DATA_PIN 12
#define CLOCK_PIN 8

#define DEBUG_RED 11
#define DEBUG_GREEN 13
#define DEBUG_BLUE 10

#define DELAY 200

// Define the array of leds
CRGB leds[NUM_LEDS];

void setup() { 
  Serial.begin(9600);

  FastLED.addLeds<WS2801, DATA_PIN, CLOCK_PIN, BGR>(leds, NUM_LEDS);
  FastLED.setBrightness(128);

  pinMode(DEBUG_RED, OUTPUT);
}

#if 1
// Convert RGB leds to single value ones
void light_led(uint16_t led, uint8_t value) {
  uint16_t i = led / (uint16_t)3;

  switch (led % 3) {
    default: leds[i].r = value; break;
    case 1: leds[i].g = value; break;
    case 2: leds[i].b = value; break;
  }
}
#endif


void loop() {

  for (int led = 0; led < NUM_LEDS; led++) {
    // Turn the LED on, then pause
    Serial.println("RED");
    leds[led] = CRGB::Red;
    FastLED.show();
    digitalWrite(DEBUG_RED, HIGH);
    delay(DELAY);
    digitalWrite(DEBUG_RED, LOW);

    // Now turn the LED off, then pause
    Serial.println("GREEN");
    leds[led] = CRGB::Green;
    FastLED.show();
    digitalWrite(DEBUG_GREEN, HIGH);
    delay(DELAY);
    digitalWrite(DEBUG_GREEN, LOW);

    Serial.println("BLUE");
    leds[led] = CRGB::Blue;
    FastLED.show();
    digitalWrite(DEBUG_BLUE, HIGH);
    delay(DELAY);
    digitalWrite(DEBUG_BLUE, LOW);

    leds[led] = 0;
  }
}
