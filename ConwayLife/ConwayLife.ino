#include <FastLED.h>
#include <math.h>

#include "Wire.h"
#include "LiquidCrystal.h"

#define DEBUG_LEVEL DEBUG_HIGH
#include <Debug.h>

#define LED_PIN  6

#define COLOR_ORDER GRB
#define CHIPSET     WS2812B

#define BRIGHTNESS 128

const uint8_t width = 20;
const uint8_t height = 12;

#define NUM_LEDS (width * height)
CRGB leds_plus_safety_pixel[ NUM_LEDS + 1];
CRGB* leds( leds_plus_safety_pixel + 1);

#define CURRENT 0x1
#define NEXT    0x2
byte cells[width][height];


// Connect via i2c, default address #0 (A0-A2 not jumpered)
LiquidCrystal lcd(0);

void setup() {
  Serial.begin(9600);

  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalSMD5050);
  FastLED.setBrightness( BRIGHTNESS );

  randomSeed(analogRead(0));

  lcd.begin(16,2);
  lcd.setBacklight(HIGH);

  initialize(random(20,50));
}


uint16_t XY( uint8_t x, uint8_t y)
{
  return (y * width) + x;  
}

uint32_t last_reset = 0;
uint16_t age = 0;
uint16_t max_age = 0;
void initialize(uint32_t threshold) {
  DEBUG1_VALUELN("Initing with threshold:", threshold);

  if (age > max_age)
    max_age = age;

  lcd.setCursor(0, 0);
  lcd.print("Thre:");
  lcd.print(threshold);
  lcd.print(" Max:");
  lcd.print(max_age);

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      if (random(0,100) < threshold) {
        cells[x][y] = CURRENT;
        leds[XY(x, y)] = CRGB(255, 0, 0);
      } else {
        leds[XY(x, y)] = CRGB(0, 0, 0);
      }
    }
  }

  FastLED.show();

  last_reset = millis();
  age = 0;
}

#define CELL(x, y, ix, iy) \
  (cells[(x + width + ix) % width][(y + height + iy) % height])

byte neighbors(byte x, byte y) {
  byte count = 0;

  if (CELL(x, y, -1, -1) & CURRENT) count++;
  if (CELL(x, y, -1,  0) & CURRENT) count++;
  if (CELL(x, y, -1,  1) & CURRENT) count++;

  if (CELL(x, y, 0, -1) & CURRENT) count++;
  if (CELL(x, y, 0,  1) & CURRENT) count++;

  if (CELL(x, y, 1, -1) & CURRENT) count++;
  if (CELL(x, y, 1,  0) & CURRENT) count++;
  if (CELL(x, y, 1,  1) & CURRENT) count++;

  return count;
}

uint16_t prev_changes = 0;
void loop() {
  uint16_t changes = conways_rules();
  uint16_t count = 0;

  age++;

  DEBUG1_VALUE(" Changes:", changes);

  boolean done = (changes == 0);
  float cycles = 0;
  while (!done) {
    cycles += 1;
    for (int x = 0; x < width; x++) {
      for (int y = 0; y < height; y++) {
        if (cells[x][y] & NEXT) {
          leds[XY(x, y)] += CRGB(ceil(cycles/8), 0, 0);

          if (!(cells[x][y] & CURRENT) && (leds[XY(x, y)].red == 255)) {
            done = true;
          }
        } else if (cells[x][y] & CURRENT) {
          leds[XY(x, y)] -= CRGB(10, 0, 0);
          if (leds[XY(x, y)].red == 0) {
            done = true;
          }
        } else {
          leds[XY(x, y)] = CRGB(0, 0, 0);
        }
      }
    }

    FastLED.show();
    delay(8);
  }

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      if (cells[x][y] & NEXT) {
        leds[XY(x, y)] = CRGB(255, 0, 0);
        cells[x][y] = CURRENT;
        count++;
      } else {
        leds[XY(x, y)] = CRGB(0, 0, 0);
        cells[x][y] = 0;
      }
    }
  }
  FastLED.show();


  DEBUG1_VALUE(" Count:", count);
  DEBUG1_VALUELN(" Cycles:", cycles);

  lcd.setCursor(0, 1);
  lcd.print("");
  lcd.print(changes);
  lcd.print("-");
  lcd.print(count);
  lcd.print(" :");
  lcd.print((int)cycles);
  lcd.print("-");
  lcd.print(age);
  lcd.print("  ");

  static uint32_t repeating_start = 0;
  uint32_t elapsed = 0;
  if (prev_changes == changes) {
    if (repeating_start) {
      elapsed = millis() - repeating_start;
    } else {
      repeating_start = millis();
    }
  } else {
    repeating_start = 0;
  }

  if ((changes == 0) ||
      ((prev_changes == changes) && (elapsed > 15 * 1000))) {
    delay(5000);
    initialize(random(20,50));
    prev_changes = 0;
  } else {
    prev_changes = changes;
  }

}

uint16_t conways_rules() {
  uint16_t changes = 0;

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      byte count = neighbors(x, y);

      if (cells[x][y] & CURRENT) {
        // Only lives on a 2 or 3
        if (count == 2 || count == 3) {
          cells[x][y] |= NEXT;
        } else {
          changes++;
        }
      } else {
        // Reproduce on a 3
        if (count == 3) {
          cells[x][y] |= NEXT;
          changes++;
        }
      }
    }
  }

  return changes;
}
