#include <FastLED.h>

#define DATA_PIN    7
#define NUM_LEDS    36
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define TRIG_PIN    12
#define ECHO_PIN    13

CRGB leds[NUM_LEDS];

// Distance thresholds (in cm)
#define DIST_GREEN_START   100
#define DIST_YELLOW_START   50
#define DIST_RED_START      30
#define DIST_BLINK          15

#define NUM_SAMPLES         10
#define PARKED_TIMEOUT      30000UL
#define MAX_DISTANCE_JUMP   50   // cm

int readings[NUM_SAMPLES];
int readIndex = 0;
int total = 0;
int lastStableDistance = 0;
bool run = false;
unsigned long lastMoveTime = 0;

// Blink timer
unsigned long blinkTimer = 0;
bool blinkState = false;

// Current zone state
int currentZone = -1;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(128);
  FastLED.clear(true);
  for (int i = 0; i < NUM_SAMPLES; i++) readings[i] = 0;
  lastMoveTime = millis();
}

void loop() {
  int distance = getSmoothedDistance();

  if (abs(distance - lastStableDistance) > 2) {
    lastStableDistance = distance;
    lastMoveTime = millis();
    run = true;
  }

  if (millis() - lastMoveTime > PARKED_TIMEOUT) {
    run = false;
    FastLED.clear();
    idlePulse();
    FastLED.show();
    delay(20);
    return;
  }

  FastLED.clear();

  if (run) {
    int newZone = detectZone(distance);

    // Only update LEDs if zone has changed
    if (newZone != currentZone) {
      currentZone = newZone;
      switch (currentZone) {
        case 0:  // Too far
          FastLED.clear();
          break;
        case 1:  // Green
          fill_solid(leds, NUM_LEDS, CRGB::Green);
          break;
        case 2:  // Yellow
          fill_solid(leds, NUM_LEDS, CRGB::Yellow);
          break;
        case 3:  // Red
          fill_solid(leds, NUM_LEDS, CRGB::Red);
          break;
        case 4:  // Blink red - handled below
          break;
      }
      FastLED.show();
    }

    if (currentZone == 4) {
      if (millis() - blinkTimer >= 1000) {
        blinkTimer = millis();
        blinkState = !blinkState;
        fill_solid(leds, NUM_LEDS, blinkState ? CRGB::Red : CRGB::Black);
        FastLED.show();
      }
    }
  }

  delay(30);
}

int detectZone(int distance) {
  // Hysteresis buffers of +2 cm
  if (distance > DIST_GREEN_START) return 0;       // OFF
  else if (distance > DIST_YELLOW_START) return 1; // GREEN
  else if (distance > DIST_RED_START + 2) return 2; // YELLOW (hysteresis gap)
  else if (distance > DIST_BLINK + 2) return 3;    // RED
  else return 4;                                   // BLINK RED
}

int getSmoothedDistance() {
  int d = readDistance();

  // Limit wild swings
  if (abs(d - lastStableDistance) > MAX_DISTANCE_JUMP) {
    d = lastStableDistance;
  }

  total -= readings[readIndex];
  readings[readIndex] = d;
  total += readings[readIndex];
  readIndex = (readIndex + 1) % NUM_SAMPLES;
  return total / NUM_SAMPLES;
}

int readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return DIST_GREEN_START + 10;
  return (duration / 2) / 29.1; // in cm
}

void idlePulse() {
  static uint8_t pulse = 0;
  static bool up = true;
  if (up) pulse += 3; else pulse -= 3;
  if (pulse == 0 || pulse >= 100) up = !up;
  leds[0] = CRGB(0, 0, pulse); // soft blue idle indicator
}
