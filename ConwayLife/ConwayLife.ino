#include <FastLED.h>

#define DEBUG_LEVEL DEBUG_HIGH
#include <Debug.h>

#define LED_PIN  5

#define COLOR_ORDER GRB
#define CHIPSET     WS2812B

#define BRIGHTNESS 255

const uint8_t width = 20;
const uint8_t height = 12;

#define NUM_LEDS (width * height)
CRGB leds_plus_safety_pixel[ NUM_LEDS + 1];
CRGB* leds( leds_plus_safety_pixel + 1);

#define CURRENT 0x1
#define NEXT    0x2
byte cells[width][height];

void setup() {
  Serial.begin(9600);

  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalSMD5050);
  FastLED.setBrightness( BRIGHTNESS );

  randomSeed(analogRead(0));

  initialize(random(0,10));
}


uint16_t XY( uint8_t x, uint8_t y)
{
  return (y * width) + x;  
}

uint32_t last_reset = 0;
void initialize(uint32_t threshold) {
  DEBUG1_VALUELN("Initing with threshold:", threshold);
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      if (random(0,10) < threshold) {
        cells[x][y] = NEXT;
      }
    }
  }

  last_reset = millis();
}

byte neighbors(byte x, byte y) {
  byte count = 0;

  if (x > 0) {
    if (cells[x-1][y] & CURRENT) count++;
    if ((y > 0) && (cells[x-1][y-1] & CURRENT)) count++;
    if ((y < height - 1) && (cells[x-1][y+1] & CURRENT)) count++;
  }

  if ((y > 0) && (cells[x][y-1] & CURRENT)) count++;
  if ((y < height - 1) && (cells[x][y+1] & CURRENT)) count++;

  if (x < width - 1) {
    if (cells[x+1][y] & CURRENT) count++;
    if ((y > 0) && (cells[x+1][y-1] & CURRENT)) count++;
    if ((y < height - 1) && (cells[x+1][y+1] & CURRENT)) count++;
  }

  return count;
}

uint16_t prev_changes = 0;
void loop() {
  uint32_t elapsed = millis() - last_reset;

  uint16_t changes = conways_rules();
  uint16_t count = 0;
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      if (cells[x][y] & NEXT) {
        count++;
        cells[x][y] = CURRENT;
        leds[XY(x, y)] = CRGB(255, 0, 0);
      } else {
        cells[x][y] = 0;
        leds[XY(x, y)] = CRGB(0, 0, 0);
      }
    }
  }

  DEBUG1_VALUE("Count:", count);
  DEBUG1_VALUELN(" Changes:", changes);
  if ((changes == 0) |
      ((prev_changes == changes) && (elapsed > 30 * 1000))) {
    delay(5000);
    initialize(random(0,10));
  }
  prev_changes = changes;

  FastLED.show();
  delay(500);
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
