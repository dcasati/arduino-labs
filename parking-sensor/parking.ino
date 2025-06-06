#include <FastLED.h>
#include <avr/sleep.h>    // For sleep modes
#include <avr/power.h>    // For power management
#include <avr/wdt.h>      // Watchdog timer for wake-up

#define DATA_PIN    7
#define NUM_LEDS    36
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define TRIG_PIN    12
#define ECHO_PIN    13

CRGB leds[NUM_LEDS];

// Distance thresholds (in cm)
#define DIST_GREEN_START   500   // 5 meters
#define DIST_YELLOW_START   200  // 2 meters
#define DIST_RED_START      100  // 1 meter
#define DIST_BLINK          15   // 15 cm

#define NUM_SAMPLES         10        // Standard number of samples for smoothing
#define NUM_CRITICAL_SAMPLES 3        // Fewer samples for critical distance (more responsive)
#define PARKED_TIMEOUT      30000UL
#define MAX_DISTANCE_JUMP   100       // cm - increased for wider distance range
#define SLEEP_TIMEOUT       60000UL   // Time before deep sleep (1 minute)
#define WAKEUP_INTERVAL     5000UL    // Check every 5 seconds
#define CRITICAL_BLINK_INTERVAL 250UL // Faster blinking in critical zone (ms)
#define NORMAL_BLINK_INTERVAL 1000UL  // Normal blinking interval (ms)

// LED display smoothing parameters
#define LED_CHANGE_THRESHOLD 2        // Minimum distance change to update display
#define COLOR_SMOOTHING_FACTOR 0.7    // Color transition smoothing (0-1), higher = smoother
#define POSITION_HISTORY_SIZE 5       // Number of previous positions to average

// Forward declarations of all functions to prevent compiler errors
void startupAnimation();
void enterSleep();
int getSmoothedDistance();
int readDistance();
void idlePulse();
int detectZone(int distance);
void displayDistanceGradient(int distance);

int readings[NUM_SAMPLES];
int readIndex = 0;
int total = 0;
int lastStableDistance = 0;
bool run = false;
unsigned long lastMoveTime = 0;
unsigned long lastWakeTime = 0;
bool sleeping = false;

// Blink timer
unsigned long blinkTimer = 0;
bool blinkState = false;

// Current zone state
int currentZone = -1;

// Sleep mode variables
volatile bool wakeupFlag = false;
// Fast readings for critical distances
int criticalReadings[NUM_CRITICAL_SAMPLES];
int criticalIndex = 0;
int criticalTotal = 0;

// Flag to indicate we're in a critical distance zone (to use faster sampling)
bool inCriticalZone = false;

// Variables for LED flicker prevention
int lastPositionLed = -1;                      // Last LED position for hysteresis
CRGB lastLedColors[NUM_LEDS];                  // Previous LED colors for smooth transitions
int positionHistory[POSITION_HISTORY_SIZE];    // History of position values
int positionHistoryIndex = 0;                  // Current index in position history
bool positionHistoryFilled = false;            // Whether the history buffer is filled

void setup() {
  // Disable watchdog timer during setup
  wdt_disable();
  
  // Set pin modes
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Initialize LED strip
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(128);
  FastLED.clear(true);
  
  // Initialize standard smoothing array
  for (int i = 0; i < NUM_SAMPLES; i++) readings[i] = 0;
  
  // Initialize critical smoothing array
  for (int i = 0; i < NUM_CRITICAL_SAMPLES; i++) criticalReadings[i] = 0;
  
  // Initialize position history
  for (int i = 0; i < POSITION_HISTORY_SIZE; i++) positionHistory[i] = 0;
  
  // Initialize LED color arrays for flicker prevention
  for (int i = 0; i < NUM_LEDS; i++) {
    lastLedColors[i] = CRGB::Black;
  }
  
  // Run the startup animation to indicate system is ready and test LEDs
  startupAnimation();
  
  lastMoveTime = millis();
  lastWakeTime = millis();
}

void loop() {
  // Check if it's time to wake up and scan for cars
  unsigned long currentTime = millis();
  
  // If we're in sleep mode
  if (sleeping) {
    // We've just woken up from sleep via watchdog timer
    // The watchdog timer will automatically wake us every ~4 seconds
    
    // Check for a car - take multiple readings to be sure
    int sum = 0;
    const int numReadings = 3;
    
    for (int i = 0; i < numReadings; i++) {
      int d = readDistance(); // Use direct reading, not smoothed
      sum += d;
      delay(50);  // Brief pause between readings
    }
    
    int avgDistance = sum / numReadings;
    
    // Record when we last woke up
    lastWakeTime = currentTime;
    
    // If car detected, exit sleep mode
    if (avgDistance <= DIST_GREEN_START) {
      sleeping = false;
      run = true;
      lastMoveTime = currentTime;
      lastStableDistance = avgDistance;
      
      // Run wake-up animation
      // Quick robot eye wake-up animation will be shown as we exit enterSleep()
    } else {
      // No car detected, go back to sleep
      // Don't need any delay as we just did several readings
      enterSleep();  // Go back to sleep, will wake again in ~4 seconds
    }
    return;  // Process the rest of the loop next time
  }
  
  // Normal operation when not sleeping
  int distance = getSmoothedDistance();

  if (abs(distance - lastStableDistance) > 2) {
    lastStableDistance = distance;
    lastMoveTime = currentTime;
    run = true;
  }

  // Check if we haven't detected movement for a while
  if (currentTime - lastMoveTime > PARKED_TIMEOUT) {
    // Enter idle mode
    run = false;
    FastLED.clear();
    idlePulse();
    FastLED.show();
    
    // If we've been idle for a long time, enter sleep mode
    if (currentTime - lastMoveTime > SLEEP_TIMEOUT) {
      // Set sleeping flag first
      sleeping = true;
      
      // Take one more distance reading just to be sure there's no car
      int finalCheck = readDistance();
      if (finalCheck <= DIST_GREEN_START) {
        // Car detected at the last moment, abort sleep
        sleeping = false;
        lastMoveTime = currentTime;
        lastStableDistance = finalCheck;
      } else {
        // No car detected, proceed with sleep
        enterSleep();
      }
    }
    
    delay(20);
    return;
  }

  FastLED.clear();

  if (run) {
    int newZone = detectZone(distance);
    currentZone = newZone;
    
    // Update blinking state for critical zone
    if (currentZone == 4) {
      // In critical zone, make the blink rate depend on how close the car is to the wall
      // The closer it gets, the faster it blinks
      unsigned long blinkInterval;
      if (distance < DIST_BLINK / 2) {
        // Ultra critical - very fast blinking (100ms)
        blinkInterval = 100;
      } else if (inCriticalZone) {
        // Critical zone - fast blinking
        blinkInterval = CRITICAL_BLINK_INTERVAL;
      } else {
        // Regular blinking
        blinkInterval = NORMAL_BLINK_INTERVAL;
      }
      
      if (currentTime - blinkTimer >= blinkInterval) {
        blinkTimer = currentTime;
        blinkState = !blinkState;
      }
    } else {
      blinkState = true; // Always on when not in critical zone
    }
    
    // Display smooth gradient based on exact distance
    displayDistanceGradient(distance);
  }

  // Check if it's time to sleep
  if (!sleeping && millis() - lastMoveTime > SLEEP_TIMEOUT) {
    // Set sleeping flag first, before we enter sleep
    sleeping = true;
    enterSleep();
  }
  
  // Adjust delay based on zone - faster response in critical zones
  if (inCriticalZone) {
    delay(10); // Faster sampling in critical zone
  } else if (currentZone == 3) { // Red zone
    delay(20); // Faster sampling in red zone
  } else {
    delay(30); // Standard delay for other zones
  }
}

int detectZone(int distance) {
  static int lastZone = 0;
  int newZone;
  
  // Determine the base zone
  if (distance > DIST_GREEN_START) newZone = 0;       // OFF
  else if (distance > DIST_YELLOW_START) newZone = 1; // GREEN
  else if (distance > DIST_RED_START) newZone = 2;    // YELLOW
  else if (distance > DIST_BLINK) newZone = 3;        // RED
  else newZone = 4;                                   // BLINK RED
  
  // Apply hysteresis, but only when moving AWAY from the wall
  // When getting closer to the wall (decreasing distance), respond immediately
  // This makes the system more responsive to the car approaching the wall
  if (newZone < lastZone) {  // Moving away from wall
    // Apply hysteresis to prevent flickering
    switch (lastZone) {
      case 1:  // Was GREEN
        if (distance < DIST_GREEN_START + 3) newZone = 1;  // Stay GREEN
        break;
      case 2:  // Was YELLOW
        if (distance < DIST_YELLOW_START + 3) newZone = 2;  // Stay YELLOW
        break;
      case 3:  // Was RED
        if (distance < DIST_RED_START + 3) newZone = 3;  // Stay RED
        break;
      case 4:  // Was BLINKING RED
        if (distance < DIST_BLINK + 3) newZone = 4;  // Stay BLINKING
        break;
    }
  }
  
  lastZone = newZone;
  return newZone;
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
  int smoothed = total / NUM_SAMPLES;

  // If we're in a critical zone, use faster, less smooth readings
  if (inCriticalZone) {
    criticalTotal -= criticalReadings[criticalIndex];
    criticalReadings[criticalIndex] = d;
    criticalTotal += criticalReadings[criticalIndex];
    criticalIndex = (criticalIndex + 1) % NUM_CRITICAL_SAMPLES;
    int criticalSmoothed = criticalTotal / NUM_CRITICAL_SAMPLES;

    // If the critical smoothed value is significantly different, update the stable distance
    if (abs(criticalSmoothed - lastStableDistance) > 2) {
      lastStableDistance = criticalSmoothed;
      run = true;
    }

    // Also, update the inCriticalZone flag based on the current distance
    inCriticalZone = (d <= DIST_BLINK);
  } else {
    inCriticalZone = (d <= DIST_BLINK);
  }

  return smoothed;
}

int readDistance() {
  // Power management: only enable the sensor when we're actively measuring
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Increased timeout to 35000µs (~6m max) to ensure reliable readings up to 5m
  long duration = pulseIn(ECHO_PIN, HIGH, 35000);
  
  if (duration == 0) return DIST_GREEN_START + 10; // Return just beyond max range if no echo detected
  return (duration / 2) / 29.1; // in cm
}

void idlePulse() {
  static uint8_t pulse = 0;
  static bool up = true;
  if (up) pulse += 3; else pulse -= 3;
  if (pulse == 0 || pulse >= 100) up = !up;
  
  // In idle mode, only light up a single LED to save power
  FastLED.clear();
  leds[0] = CRGB(0, 0, pulse); // soft blue idle indicator
}

// Function to enter sleep mode with watchdog timer for wake-up
void enterSleep() {
  // Quick fade out before sleep
  for (int brightness = 128; brightness >= 0; brightness -= 10) {
    FastLED.setBrightness(brightness);
    FastLED.show();
    delay(10);
  }
  FastLED.clear();
  FastLED.show();
  
  // Turn off various peripherals to save power
  power_adc_disable();
  
  // Set up the watchdog timer to wake us up
  // Time constants: 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms, 6=1s, 7=2s, 8=4s, 9=8s
  // We want roughly 5 seconds of sleep, so we'll use 8=4s and wake up slightly more often
  setupWatchdog(8); // 4 seconds
  
  // Enter power-save mode (allows watchdog timer to work)
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);  // Changed from POWER_DOWN to POWER_SAVE
  sleep_enable();
  
  // Enter sleep mode
  sleep_mode();
  
  // Code continues here after watchdog timer wakes us
  sleep_disable();
  
  // Re-enable peripherals
  power_all_enable();
  
  // KITT-style wake-up animation (Knight Rider) with 5 LEDs
  // Reset brightness
  FastLED.setBrightness(128);
  
  // Define robot eye colors
  const CRGB brightRed = CRGB(255, 0, 0);  // Center of KITT scanner (brightest)
  const CRGB mediumRed = CRGB(150, 0, 0);  // Inner edge LEDs
  const CRGB dimRed = CRGB(70, 0, 0);      // Outer edge LEDs
  
  // Clear all LEDs first
  FastLED.clear();
  FastLED.show();
  delay(50);
  
  // Create the classic KITT scanner effect with 5 LEDs only
  for (int cycles = 0; cycles < 3; cycles++) {  // Do 3 cycles of scanning
    // Left to right scan
    for (int i = 0; i < NUM_LEDS - 4; i++) {
      // Clear all LEDs first
      FastLED.clear();
      
      // Create the 5-LED KITT scanner with brightness gradient
      leds[i] = dimRed;           // Outer edge (dimmest)
      leds[i+1] = mediumRed;      // Inner edge (medium)
      leds[i+2] = brightRed;      // Center (brightest)
      leds[i+3] = mediumRed;      // Inner edge (medium)
      leds[i+4] = dimRed;         // Outer edge (dimmest)
      
      FastLED.show();
      delay(15); // Fast scanning like in Knight Rider
    }
    
    // Right to left scan
    for (int i = NUM_LEDS - 5; i >= 0; i--) {
      // Clear all LEDs first
      FastLED.clear();
      
      // Create the 5-LED KITT scanner with brightness gradient
      leds[i] = dimRed;           // Outer edge (dimmest)
      leds[i+1] = mediumRed;      // Inner edge (medium)
      leds[i+2] = brightRed;      // Center (brightest)
      leds[i+3] = mediumRed;      // Inner edge (medium)
      leds[i+4] = dimRed;         // Outer edge (dimmest)
      
      FastLED.show();
      delay(15); // Fast scanning like in Knight Rider
    }
  }
  
  // Final "power up" pulse - just with the 5 center LEDs
  int middle = NUM_LEDS / 2;
  int start = middle - 2; // Start 2 LEDs to the left of center
  
  for (int pulse = 0; pulse < 2; pulse++) {
    // Pulse bright - just the center 5 LEDs
    FastLED.clear();
    for (int i = 0; i < 5; i++) {
      if (start + i >= 0 && start + i < NUM_LEDS) {
        leds[start + i] = brightRed;
      }
    }
    FastLED.show();
    delay(50);
    
    // Pulse off
    FastLED.clear();
    FastLED.show();
    delay(50);
  }
  
  // Clear all LEDs
  FastLED.clear();
  FastLED.show();
  
  // Indicate we're awake
  sleeping = false;
  lastWakeTime = millis();
}

// Function to display a robot eye startup animation
void startupAnimation() {
  FastLED.clear();
  FastLED.show();
  
  // Define KITT colors
  const CRGB brightRed = CRGB(255, 0, 0);   // Main KITT scanner color (center - brightest)
  const CRGB mediumRed = CRGB(150, 0, 0);   // Inner edges (medium brightness)
  const CRGB dimRed = CRGB(70, 0, 0);       // Outer edges (dimmest)
  const CRGB veryDimRed = CRGB(20, 0, 0);   // Very dim for initial pulse
  
  // 1. Initial power-up sequence - pulse just the center 5 LEDs
  int middle = NUM_LEDS / 2;
  int start = middle - 2; // 2 LEDs to left of center
  
  // Slowly brighten the 5 center LEDs
  for (int bright = 0; bright < 100; bright += 3) {
    FastLED.clear();
    float factor = bright / 100.0;
    
    for (int i = 0; i < 5; i++) {
      if (start + i >= 0 && start + i < NUM_LEDS) {
        // Gradient brightness from outer to inner LEDs
        if (i == 0 || i == 4) {
          leds[start + i] = CRGB(bright * 0.3, 0, 0); // 30% brightness for outer LEDs
        } else if (i == 1 || i == 3) {
          leds[start + i] = CRGB(bright * 0.6, 0, 0); // 60% brightness for inner LEDs
        } else {
          leds[start + i] = CRGB(bright, 0, 0); // Full brightness for center
        }
      }
    }
    FastLED.show();
    delay(15);
  }
  delay(200);
  
  // 2. Initial KITT-style sweep - longer and more dramatic for startup
  
  // First slow sweep (left to right)
  for (int i = 0; i < NUM_LEDS - 4; i++) {
    // Clear all LEDs first
    FastLED.clear();
    
    // Create the 5-LED KITT scanner pattern with gradient
    leds[i] = dimRed;           // Outer edge (dimmest)
    leds[i+1] = mediumRed;      // Inner edge (medium)
    leds[i+2] = brightRed;      // Center (brightest)
    leds[i+3] = mediumRed;      // Inner edge (medium)
    leds[i+4] = dimRed;         // Outer edge (dimmest)
    
    FastLED.show();
    delay(25);  // Slightly slower for dramatic effect
  }
  
  // First slow sweep (right to left)
  for (int i = NUM_LEDS - 5; i >= 0; i--) {
    // Clear all LEDs first
    FastLED.clear();
    
    // Create the 5-LED KITT scanner pattern with gradient
    leds[i] = dimRed;           // Outer edge (dimmest)
    leds[i+1] = mediumRed;      // Inner edge (medium)
    leds[i+2] = brightRed;      // Center (brightest)
    leds[i+3] = mediumRed;      // Inner edge (medium)
    leds[i+4] = dimRed;         // Outer edge (dimmest)
    
    FastLED.show();
    delay(25);
  }
  
  // 3. Classic KITT scanner effect, gradually speeding up
  for (int cycle = 0; cycle < 3; cycle++) {
    // Calculate delay based on cycle (gets faster)
    int cycleDelay = 20 - (cycle * 5);  // 20ms, 15ms, 10ms
    
    // Left to right scan
    for (int i = 0; i < NUM_LEDS - 4; i++) {
      // Clear all LEDs first
      FastLED.clear();
      
      // Create the 5-LED KITT scanner pattern with gradient
      leds[i] = dimRed;           // Outer edge (dimmest)
      leds[i+1] = mediumRed;      // Inner edge (medium)
      leds[i+2] = brightRed;      // Center (brightest)
      leds[i+3] = mediumRed;      // Inner edge (medium)
      leds[i+4] = dimRed;         // Outer edge (dimmest)
      
      FastLED.show();
      delay(cycleDelay);
    }
    
    // Right to left scan
    for (int i = NUM_LEDS - 5; i >= 0; i--) {
      // Clear all LEDs first
      FastLED.clear();
      
      // Create the 5-LED KITT scanner pattern with gradient
      leds[i] = dimRed;           // Outer edge (dimmest)
      leds[i+1] = mediumRed;      // Inner edge (medium)
      leds[i+2] = brightRed;      // Center (brightest)
      leds[i+3] = mediumRed;      // Inner edge (medium)
      leds[i+4] = dimRed;         // Outer edge (dimmest)
      
      FastLED.show();
      delay(cycleDelay);
    }
  }
  
  // 4. System activation pulse - represents KITT "coming online" - just 5 center LEDs
  start = middle - 2; // 2 LEDs to left of center
  
  for (int pulse = 0; pulse < 3; pulse++) {
    // Pulse bright - just the center 5 LEDs
    FastLED.clear();
    for (int i = 0; i < 5; i++) {
      if (start + i >= 0 && start + i < NUM_LEDS) {
        // Full brightness for all 5 LEDs
        leds[start + i] = brightRed;
      }
    }
    FastLED.show();
    delay(80);
    
    // Pulse off
    FastLED.clear();
    FastLED.show();
    delay(80);
  }
  
  // 5. Final "ready" state - one more quick sweep
  // Left to right (fast)
  for (int i = 0; i < NUM_LEDS - 4; i += 2) {
    FastLED.clear();
    leds[i] = dimRed;           // Outer edge
    leds[i+1] = mediumRed;      // Inner edge
    leds[i+2] = brightRed;      // Center
    leds[i+3] = mediumRed;      // Inner edge
    leds[i+4] = dimRed;         // Outer edge
    FastLED.show();
    delay(10);
  }
  
  // Right to left (fast)
  for (int i = NUM_LEDS - 5; i >= 0; i -= 2) {
    FastLED.clear();
    leds[i] = dimRed;           // Outer edge
    leds[i+1] = mediumRed;      // Inner edge
    leds[i+2] = brightRed;      // Center
    leds[i+3] = mediumRed;      // Inner edge
    leds[i+4] = dimRed;         // Outer edge
    FastLED.show();
    delay(10);
  }
  
  // 6. System ready - clean finish
  FastLED.clear();
  FastLED.show();
  delay(100);
}

// Watchdog Timer Interrupt Service Routine
ISR(WDT_vect) {
  // This is executed when watchdog times out - do nothing here
  wdt_disable();  // Disable watchdog timer
}

// Function to initialize the watchdog timer
void setupWatchdog(int timerPrescaler) {
  // Enable the watchdog timer with specified prescaler
  byte wdtcsr = timerPrescaler & 7;
  if (timerPrescaler > 7) wdtcsr |= (1<<5); // Set the WDP3 bit if needed
  
  wdtcsr |= (1<<WDCE);
  MCUSR &= ~(1<<WDRF); // Clear reset flag
  
  // Start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  
  // Set new watchdog timeout prescaler and enable interrupt
  WDTCSR = wdtcsr | (1<<WDIE);
}

// Function to display a smooth gradient on LEDs based on the exact distance
void displayDistanceGradient(int distance) {
  // 1. If too far, don't show anything
  if (distance > DIST_GREEN_START) {
    if (lastPositionLed != -1) {
      // If we just went out of range, fade out rather than abrupt off
      for (int i = 0; i <= lastPositionLed; i++) {
        // Fade each LED by reducing brightness to 75%, then 50%, then 25%, then off
        leds[i].fadeToBlackBy(64);
      }
      FastLED.show();
      lastPositionLed = -1; // Reset last position
    } else {
      // Already out of range, keep LEDs off
      FastLED.clear();
      FastLED.show();
    }
    return;
  }
  
  // 2. Calculate the relative position in the entire distance range (0-100%)
  // 0% = wall/zero distance, 100% = max distance (DIST_GREEN_START)
  float relativePosition = (float)distance / DIST_GREEN_START;
  
  // 3. Calculate the LED that represents the current position (raw)
  // This is the "position indicator" LED
  int rawPositionLed = (int)((1.0 - relativePosition) * NUM_LEDS);
  rawPositionLed = constrain(rawPositionLed, 0, NUM_LEDS - 1);
  
  // 4. Apply temporal smoothing to position using moving average
  // Add current position to history
  positionHistory[positionHistoryIndex] = rawPositionLed;
  positionHistoryIndex = (positionHistoryIndex + 1) % POSITION_HISTORY_SIZE;
  if (positionHistoryIndex == 0) positionHistoryFilled = true;
  
  // Calculate average position from history
  int sum = 0;
  int count = positionHistoryFilled ? POSITION_HISTORY_SIZE : positionHistoryIndex;
  if (count == 0) count = 1; // Avoid division by zero
  
  for (int i = 0; i < count; i++) {
    sum += positionHistory[i];
  }
  int avgPosition = sum / count;
  
  // 5. Apply hysteresis to prevent small oscillations
  int positionLed;
  if (lastPositionLed == -1) {
    // First time, use calculated position
    positionLed = avgPosition;
  } else {
    // Apply hysteresis - only change if the difference exceeds threshold
    if (abs(avgPosition - lastPositionLed) >= LED_CHANGE_THRESHOLD) {
      // Move part way toward the target position (damping)
      positionLed = lastPositionLed + ((avgPosition - lastPositionLed) / 2);
    } else {
      // Keep previous position
      positionLed = lastPositionLed;
    }
  }
  positionLed = constrain(positionLed, 0, NUM_LEDS - 1);
  
  // 6. Calculate the number of LEDs to light up
  // We'll use a fill approach from 0 to position LED
  int numLedsToLight = positionLed + 1;
  
  // 7. Calculate zone boundaries in LED positions rather than distances
  int greenYellowBoundary = (int)((1.0 - (float)DIST_YELLOW_START / DIST_GREEN_START) * NUM_LEDS);
  int yellowRedBoundary = (int)((1.0 - (float)DIST_RED_START / DIST_GREEN_START) * NUM_LEDS);
  int redBlinkBoundary = (int)((1.0 - (float)DIST_BLINK / DIST_GREEN_START) * NUM_LEDS);
  
  // 8. Temporary array for calculating new LED colors
  CRGB newColors[NUM_LEDS];
  
  // 9. Determine color transitions based on zones
  for (int i = 0; i < numLedsToLight; i++) {
    // Determine LED color based on its position
    if (i <= greenYellowBoundary) {
      // Green zone
      newColors[i] = CRGB::Green;
    } 
    else if (i <= yellowRedBoundary) {
      // Yellow-Green transition zone
      // Calculate how far into the transition we are (0-100%)
      float transitionProgress = (float)(i - greenYellowBoundary) / (yellowRedBoundary - greenYellowBoundary);
      
      // Create a blend from green to yellow
      // Yellow is RGB(255,255,0), Green is RGB(0,255,0)
      // We're transitioning the red channel from 0 to 255
      newColors[i] = CRGB(
        255 * transitionProgress,  // Red increases from 0 to 255
        255,                       // Green stays at 255
        0                          // Blue stays at 0
      );
    }
    else if (i <= redBlinkBoundary) {
      // Red-Yellow transition zone
      // Calculate how far into the transition we are (0-100%)
      float transitionProgress = (float)(i - yellowRedBoundary) / (redBlinkBoundary - yellowRedBoundary);
      
      // Create a blend from yellow to red
      // Yellow is RGB(255,255,0), Red is RGB(255,0,0)
      // We're transitioning the green channel from 255 to 0
      newColors[i] = CRGB(
        255,                       // Red stays at 255
        255 * (1 - transitionProgress), // Green decreases from 255 to 0
        0                          // Blue stays at 0
      );
    }
    else {
      // Critical red zone - make it more intense
      // Calculate intensity based on how close to the end we are
      float intensity = 1.0 + (float)(i - redBlinkBoundary) / (NUM_LEDS - redBlinkBoundary) * 0.5;
      intensity = constrain(intensity, 1.0, 1.5); // Limit to 150% brightness
      
      // Create an increasingly bright red
      newColors[i] = CRGB(
        constrain((int)(255 * intensity), 0, 255), // Red increases intensity
        0,                                         // No green
        0                                          // No blue
      );
    }
  }
  
  // Clear LEDs beyond the lit ones
  for (int i = numLedsToLight; i < NUM_LEDS; i++) {
    newColors[i] = CRGB::Black;
  }
  
  // 10. Apply color smoothing between frames to reduce flicker
  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < numLedsToLight) {
      // For LEDs that should be on, blend between old and new colors
      leds[i].r = lerp8by8(leds[i].r, newColors[i].r, 255 * (1 - COLOR_SMOOTHING_FACTOR));
      leds[i].g = lerp8by8(leds[i].g, newColors[i].g, 255 * (1 - COLOR_SMOOTHING_FACTOR));
      leds[i].b = lerp8by8(leds[i].b, newColors[i].b, 255 * (1 - COLOR_SMOOTHING_FACTOR));
    } else {
      // For LEDs that should be off, fade to black more quickly
      leds[i].fadeToBlackBy(64);
    }
  }
  
  // 11. Handle critical zone blinking
  if (distance <= DIST_BLINK && !blinkState) {
    // Instead of clearing, dim LEDs significantly
    for (int i = 0; i < numLedsToLight; i++) {
      leds[i].fadeToBlackBy(230); // Almost completely dark but not fully off
    }
  }
  
  // 12. Make the position LED brighter to indicate the exact position
  if (blinkState && positionLed < NUM_LEDS) {
    // Get the current color
    CRGB baseColor = leds[positionLed];
    
    // Brighten it
    leds[positionLed] = CRGB(
      constrain(baseColor.r + 50, 0, 255),
      constrain(baseColor.g + 50, 0, 255),
      constrain(baseColor.b + 50, 0, 255)
    );
  }
  
  // 13. Save current position for next iteration
  lastPositionLed = positionLed;
  
  // 14. Display the LEDs
  FastLED.show();
}
