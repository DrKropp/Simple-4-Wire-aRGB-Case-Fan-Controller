#include <Arduino.h>
#include <FastLED.h>

// Pin definitions
#define FAN_PWM_PIN 9
#define FAN_TACH_PIN 2
#define LED_DATA_PIN 3
#define POT_PIN A0
// #define FAN_POWER_PIN 10  // Uncomment and wire MOSFET gate to this pin if fan won't stop

// LED configuration
#define NUM_LEDS 8
#define LED_TYPE NEOPIXEL
#define COLOR_ORDER GRB

// Fan configuration
#define PWM_FREQ 25000  // 25kHz PWM frequency
#define MIN_RPM_THRESHOLD 100
#define POT_DEADZONE 10  // Bottom 10% is off

// Timing constants
#define TACH_UPDATE_INTERVAL 1000  // Update RPM every 1 second
#define LED_UPDATE_INTERVAL 20     // Update LEDs at ~50Hz
#define ALERT_BLINK_INTERVAL 500   // 1Hz blink (500ms on, 500ms off)

// Global variables
CRGB leds[NUM_LEDS];
volatile unsigned int tachPulseCount = 0;
unsigned long lastTachUpdate = 0;
unsigned long lastLEDUpdate = 0;
unsigned long lastAlertToggle = 0;
bool alertState = false;
float currentRPM = 0;
uint8_t fanSpeed = 0;  // 0-255
uint8_t hueOffset = 0;
float breathPhase = 0;

// Function declarations
void setupPWM25kHz();
void tachISR();
void updateRPM();
void updateFanSpeed();
void updateLEDs();
void showRainbowBreathing();
void showAlertFlashing();
uint8_t calculateBrightness();

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);

  // Setup PWM pin
  pinMode(FAN_PWM_PIN, OUTPUT);
  setupPWM25kHz();

  // Setup optional MOSFET power control pin
  #ifdef FAN_POWER_PIN
  pinMode(FAN_POWER_PIN, OUTPUT);
  digitalWrite(FAN_POWER_PIN, LOW);  // Start with fan off
  #endif

  // Setup tachometer pin with interrupt
  pinMode(FAN_TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), tachISR, FALLING);

  // Setup potentiometer pin
  pinMode(POT_PIN, INPUT);

  // Initialize FastLED
  FastLED.addLeds<LED_TYPE, LED_DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
  FastLED.clear();
  FastLED.show();

  Serial.println("Fan Controller Initialized");
}

void loop() {
  unsigned long currentTime = millis();

  // Update fan speed based on potentiometer
  updateFanSpeed();

  // Update RPM reading
  if (currentTime - lastTachUpdate >= TACH_UPDATE_INTERVAL) {
    updateRPM();
    lastTachUpdate = currentTime;
  }

  // Update LED display
  if (currentTime - lastLEDUpdate >= LED_UPDATE_INTERVAL) {
    updateLEDs();
    lastLEDUpdate = currentTime;
  }
}

// Setup Timer1 for 25kHz PWM on pin 9
void setupPWM25kHz() {
  // Clear Timer1 control registers
  TCCR1A = 0;
  TCCR1B = 0;

  // Set Fast PWM mode with ICR1 as TOP
  // WGM13:0 = 14 (Fast PWM, TOP=ICR1)
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  // Set non-inverting mode on OC1A (pin 9)
  TCCR1A |= (1 << COM1A1);

  // Set prescaler to 1 (no prescaling)
  TCCR1B |= (1 << CS10);

  // Calculate TOP value for 25kHz
  // F_PWM = F_CPU / (Prescaler * (1 + TOP))
  // 25000 = 16000000 / (1 * (1 + TOP))
  // TOP = (16000000 / 25000) - 1 = 639
  ICR1 = 639;

  // Start with fan off
  OCR1A = 0;
}

// Tachometer interrupt service routine
void tachISR() {
  tachPulseCount++;
}

// Calculate RPM from tachometer pulses
void updateRPM() {
  // Disable interrupts while reading pulse count
  noInterrupts();
  unsigned int pulses = tachPulseCount;
  tachPulseCount = 0;
  interrupts();

  // 4-wire fans typically output 2 pulses per revolution
  // RPM = (pulses / 2) * (60 seconds / measurement_interval)
  // For 1 second interval: RPM = (pulses / 2) * 60
  currentRPM = (pulses / 2.0) * 60.0;

  // Debug output
  Serial.print("Fan Speed: ");
  Serial.print(fanSpeed);
  Serial.print(" (");
  Serial.print(map(fanSpeed, 0, 255, 0, 100));
  Serial.print("%)  RPM: ");
  Serial.println(currentRPM);
}

// Read potentiometer and update fan speed
void updateFanSpeed() {
  int potValue = analogRead(POT_PIN);

  // Map 0-1023 to 0-100 percentage
  int percentage = map(potValue, 0, 1023, 0, 100);

  // Apply deadzone: bottom 10% turns fan off
  if (percentage < POT_DEADZONE) {
    fanSpeed = 0;
    // Disable PWM and pull pin LOW to ensure fan stops
    TCCR1A &= ~(1 << COM1A1);  // Disconnect OC1A
    pinMode(FAN_PWM_PIN, OUTPUT);  // Ensure OUTPUT mode
    digitalWrite(FAN_PWM_PIN, LOW);

    // If MOSFET power control is available, cut power completely
    #ifdef FAN_POWER_PIN
    digitalWrite(FAN_POWER_PIN, LOW);  // Turn off MOSFET
    #endif
  } else {
    // Map 10-100% to 0-255 PWM
    fanSpeed = map(percentage, POT_DEADZONE, 100, 0, 255);

    // If MOSFET power control is available, enable power
    #ifdef FAN_POWER_PIN
    digitalWrite(FAN_POWER_PIN, HIGH);  // Turn on MOSFET
    #endif

    // Re-enable PWM if it was disabled
    TCCR1A |= (1 << COM1A1);  // Connect OC1A
    // Update PWM duty cycle
    OCR1A = map(fanSpeed, 0, 255, 0, ICR1);
  }
}

// Calculate LED brightness based on fan speed (30% to 100%)
uint8_t calculateBrightness() {
  // Map fan speed (0-255) to brightness (30%-100% of 255)
  return map(fanSpeed, 0, 255, 77, 255);
}

// Update LED display based on fan state
void updateLEDs() {
  if (fanSpeed == 0) {
    // Fan is off - turn off all LEDs
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  } else if (currentRPM < MIN_RPM_THRESHOLD) {
    // Fan should be running but RPM is too low - show alert
    showAlertFlashing();
  } else {
    // Normal operation - show rainbow breathing
    showRainbowBreathing();
  }

  FastLED.show();
}

// Rainbow spiral breathing effect
void showRainbowBreathing() {
  // Increment hue for rainbow spiral effect (doubled speed)
  hueOffset += 2;

  // Breathing effect using sine wave
  breathPhase += 0.05;  // Adjust for breathing speed
  if (breathPhase > TWO_PI) breathPhase = 0;

  // Calculate breathing factor (0.0 to 1.0)
  float breathFactor = (sin(breathPhase) + 1.0) / 2.0;

  // Get max brightness from fan speed (potentiometer position)
  uint8_t maxBrightness = calculateBrightness();

  // Minimum brightness is always 30% (77 out of 255)
  uint8_t minBrightness = 77;

  // Breathing sweeps from 30% to the potentiometer-set brightness
  uint8_t finalBrightness = minBrightness + (maxBrightness - minBrightness) * breathFactor;

  // Update all LEDs with rainbow spiral colors
  for (int i = 0; i < NUM_LEDS; i++) {
    // Create spiral effect by offsetting hue for each LED
    uint8_t hue = hueOffset + (i * 256 / NUM_LEDS);
    leds[i] = CHSV(hue, 255, finalBrightness);
  }
}

// Red flashing alert for low RPM
void showAlertFlashing() {
  unsigned long currentTime = millis();

  // Toggle alert state at 1Hz
  if (currentTime - lastAlertToggle >= ALERT_BLINK_INTERVAL) {
    alertState = !alertState;
    lastAlertToggle = currentTime;
  }

  // Show full brightness red or off
  if (alertState) {
    fill_solid(leds, NUM_LEDS, CRGB::Red);
  } else {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  }
}
