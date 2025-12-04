// Forward declaration
struct SensorSnapshot;

#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <string.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>
#include <AccelStepper.h>

// ---------------------------------------------------------
// 1. GLOBAL SETTINGS & ENUMS (MUST BE AT THE TOP)
// ---------------------------------------------------------

// Flag to control whether the Uno R4 connection is expected/required
bool expectUnoR4 = false;

// New Enum for the Fast Mode Display Preference
enum FastDisplayMode {
  DISPLAY_MODE_BUTTONS, // 0: The dot matrix view
  DISPLAY_MODE_SENSORS, // 1: The text values view
  DISPLAY_MODE_ACCEL    // 2: Accelerometer Debug
};

// GLOBAL VARIABLE: Defines what we see on screen. 
// Defined HERE so the Storage functions below can see it.
FastDisplayMode currentDisplayMode = DISPLAY_MODE_BUTTONS; 

const unsigned long BUTTON_SCAN_INTERVAL_US = 1000; 

// Sampling cadence
const unsigned long SENSOR_INTERVAL_FAST_MS = 100;      
const unsigned long SENSOR_INTERVAL_SLEEP_MS = 1000;    
const unsigned long SENSOR_INTERVAL_BASELINE_MS = 100; 

volatile uint16_t sensorUpdateIntervalTicks = SENSOR_INTERVAL_FAST_MS;
volatile uint8_t buttonScanPending = 0;
volatile bool sensorUpdateFlag = false;
volatile uint16_t sensorUpdateTickCounter = 0;

// --- Power / control pins ---
#define POWER_HOLD_PIN     26   
#define CHARGER_DETECT_PIN 27   
#define SWITCH_DETECT_PIN  25   
#define GATE_5V_PIN        23   
#define BUZZER_PIN         22   
#define BATTERY_PIN        A0   

const float BATTERY_R1_OHMS   = 100000.0f;
const float BATTERY_R2_OHMS   =  47000.0f;
const float BATTERY_MIN_V     =      9.6f;
const float BATTERY_MAX_V     =     12.6f;
const float ADC_REF_V         =      4.028f;
const uint8_t ADC_RESOLUTION  =       10;   
const float ADC_MAX_READING   = (1 << ADC_RESOLUTION) - 1;

// Buzzer settings
const bool BUZZER_USE_TONE = true;
const uint16_t BUZZER_TEST_FREQ_HZ = 2000;
const uint16_t BUZZER_TONE_DURATION_MS = 150;
const uint16_t BUZZER_PULSE_DURATION_MS = 120;
const uint16_t BUZZER_REWARD_FREQ_HZ = 432;
const uint16_t BUZZER_REWARD_DURATION_MS = 500;
const uint16_t BUZZER_MELODY_FREQ_HIGH_HZ = 670;
const uint16_t BUZZER_MELODY_FREQ_LOW_HZ = 605;
const uint16_t BUZZER_MELODY_STROKE_MS = 300;
const uint16_t BUZZER_MELODY_PAUSE_MS = 60;
const uint16_t BATTERY_WARNING_BEEP_MS = 140;
const uint16_t BATTERY_WARNING_GAP_MS = 120;

static unsigned long lastOledFrame = 0;

// ---------------------------------------------------------
// 2. STORAGE STRUCTURES & FUNCTIONS
// ---------------------------------------------------------

// Forward declare helper function needed by saveSequences
void buzzTest(uint16_t freqHz = BUZZER_TEST_FREQ_HZ, uint16_t durationMs = BUZZER_TONE_DURATION_MS);

// Constants
const uint8_t SIDE_COUNT = 4;
const uint8_t MAX_SEQUENCE_LENGTH = 8;
const uint8_t DEFAULT_SEQUENCE_LENGTH = 4;
const uint8_t MIN_SEQUENCE_LENGTH = 1;
const unsigned long SEQUENCE_TIMEOUT_MS = 5000;

uint8_t storedSequences[SIDE_COUNT][MAX_SEQUENCE_LENGTH];
uint8_t storedSequenceLengths[SIDE_COUNT];
uint8_t sequenceProgress[SIDE_COUNT];
unsigned long sequenceLastInput[SIDE_COUNT];
uint8_t currentTunnelSide = 1;

const uint8_t DEFAULT_SEQUENCE_TEMPLATE[DEFAULT_SEQUENCE_LENGTH] = {1, 2, 3, 4};
const uint32_t SEQUENCE_STORAGE_MAGIC = 0xB105EED1;
const uint8_t SEQUENCE_STORAGE_VERSION = 4;
const int SEQUENCE_STORAGE_ADDR = 0;

// The Main Storage Struct
struct SequenceStorage {
  uint32_t magic;
  uint8_t version;
  uint8_t expectUnoR4Enabled;
  uint8_t fastDisplayMode; // Store as uint8_t
  uint8_t lengths[SIDE_COUNT];
  uint8_t sequences[SIDE_COUNT][MAX_SEQUENCE_LENGTH];
  uint8_t checksum;

  uint8_t computeChecksum() const {
    uint16_t sum = 0;
    sum += (uint8_t)(magic & 0xFF);
    sum += (uint8_t)((magic >> 8) & 0xFF);
    sum += (uint8_t)((magic >> 16) & 0xFF);
    sum += (uint8_t)((magic >> 24) & 0xFF);
    sum += version;
    sum += expectUnoR4Enabled;
    sum += fastDisplayMode; 
    for (uint8_t side = 0; side < SIDE_COUNT; side++) {
      sum += lengths[side];
      for (uint8_t i = 0; i < MAX_SEQUENCE_LENGTH; i++) {
        sum += sequences[side][i];
      }
    }
    return (uint8_t)(sum & 0xFF);
  }
};

void resetSequenceTracking() {
  for (uint8_t side = 0; side < SIDE_COUNT; side++) {
    sequenceProgress[side] = 0;
    sequenceLastInput[side] = 0;
  }
}

void applyDefaultSequences() {
  for (uint8_t side = 0; side < SIDE_COUNT; side++) {
    storedSequenceLengths[side] = DEFAULT_SEQUENCE_LENGTH;
    for (uint8_t i = 0; i < MAX_SEQUENCE_LENGTH; i++) {
      if (i < DEFAULT_SEQUENCE_LENGTH) {
        storedSequences[side][i] = DEFAULT_SEQUENCE_TEMPLATE[i];
      } else {
        storedSequences[side][i] = 0;
      }
    }
  }
  resetSequenceTracking();
  expectUnoR4 = false; // Default off
  currentDisplayMode = DISPLAY_MODE_BUTTONS; // Default mode
}

void saveSequencesToEEPROM() {
  SequenceStorage data;
  data.magic = SEQUENCE_STORAGE_MAGIC;
  data.version = SEQUENCE_STORAGE_VERSION;
  data.expectUnoR4Enabled = expectUnoR4 ? 1 : 0;
  
  // NOW VALID: currentDisplayMode is defined above
  data.fastDisplayMode = (uint8_t)currentDisplayMode; 

  for (uint8_t side = 0; side < SIDE_COUNT; side++) {
    data.lengths[side] = storedSequenceLengths[side];
    for (uint8_t i = 0; i < MAX_SEQUENCE_LENGTH; i++) {
      data.sequences[side][i] = storedSequences[side][i];
    }
  }
  data.checksum = data.computeChecksum();

  EEPROM.put(SEQUENCE_STORAGE_ADDR, data);
  // Note: buzzTest implementation is further down, but linker handles it.
  // Ideally, move buzzTest implementation up or forward declare it.
  // I added a forward declaration above for you.
  buzzTest(BUZZER_REWARD_FREQ_HZ, BUZZER_REWARD_DURATION_MS);
}

void loadSequencesFromEEPROM() {
  uint32_t magic = 0;
  EEPROM.get(SEQUENCE_STORAGE_ADDR, magic);

  // If magic is wrong, reset everything
  if (magic != SEQUENCE_STORAGE_MAGIC) {
    applyDefaultSequences();
    currentDisplayMode = DISPLAY_MODE_BUTTONS;
    saveSequencesToEEPROM();
    return;
  }

  uint8_t version = 0;
  EEPROM.get(SEQUENCE_STORAGE_ADDR + sizeof(uint32_t), version);

  // If version matches, load the new struct
  if (version == SEQUENCE_STORAGE_VERSION) {
    SequenceStorage data;
    EEPROM.get(SEQUENCE_STORAGE_ADDR, data);
    
    if (data.computeChecksum() == data.checksum) {
      expectUnoR4 = (data.expectUnoR4Enabled != 0);
      
      // Load Display Mode
      uint8_t mode = data.fastDisplayMode;
      if (mode > 2) mode = 0; 
      currentDisplayMode = (FastDisplayMode)mode;

      for (uint8_t side = 0; side < SIDE_COUNT; side++) {
        storedSequenceLengths[side] = data.lengths[side];
        for (uint8_t i = 0; i < MAX_SEQUENCE_LENGTH; i++) {
          storedSequences[side][i] = data.sequences[side][i];
        }
      }
      resetSequenceTracking();
      return;
    }
  }

  // Version mismatch or checksum fail -> Reset defaults
  applyDefaultSequences();
  saveSequencesToEEPROM();
}

// -------------------- OLED --------------------

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3D
#define SCREEN_CHANNEL  7       // TCA9548A channel for OLED and RTC
// -------------------- TCA9548A --------------------
#define TCA_ADDRESS     0x70
static inline void tcaSelect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << ch);
  Wire.endTransmission();
}
#define OLED_SELECT() tcaSelect(SCREEN_CHANNEL)
#define RTC_SELECT()  tcaSelect(SCREEN_CHANNEL)
#define UNO_R4_CHANNEL 4       // TCA9548A channel for external Uno R4
#define UNO_R4_ADDR    0x08    // I2C address of the Uno R4
#define UNO_R4_SELECT() tcaSelect(UNO_R4_CHANNEL)
#define HANDSHAKE_SIZE 1


// -------------------- OLED instance --------------------
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
RTC_DS3231 rtc;
Adafruit_AHTX0 aht20;
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

bool ahtReady = false;
bool accelReady = false;
float lastTempC = NAN;
float lastHumidity = NAN;
unsigned long lastAhtReadMs = 0;
const unsigned long AHT_READ_INTERVAL_MS = 2000;

float accelBaselineX = 0.0f;
float accelBaselineY = 9.81f;
float accelBaselineZ = 0.0f;
unsigned long lastPeckDetectedMs = 0;
// High-sensitivity, fast peck detection tuned for light taps
const float ACCEL_BASELINE_ALPHA = 0.15f;       // slow baseline keeps small shocks visible
const unsigned long PECK_HOLD_MS = 700;         // quick visual reset for rapid tuning
const bool SERIAL_ACCEL_DEBUG = true;           // emit raw + filtered accel readings over Serial
const unsigned long ACCEL_DEBUG_INTERVAL_MS = 10;
const unsigned long SERIAL_BAUD = 115200;

// -------- Low-power & sampling states --------
const unsigned long BASELINE_DURATION_MS = 3000;
const unsigned long FAST_STABLE_HOLD_MS = 5000;        // stay stable for 5 s before napping
const unsigned long BASELINE_REFRESH_MS = 180000;      // 3 minutes of steady readings => new baseline
const uint16_t DISTANCE_BASELINE_TOLERANCE_MM = 25;    // allowable drift before waking
const uint16_t DISTANCE_MAX_BEHAVIOR_MM = 1000;        // ignore readings beyond 1m for baseline/awake logic
const float ACCEL_BASELINE_TOLERANCE = 0.08f;          // m/s^2 wiggle room over baseline noise
const float ENV_TEMP_TOLERANCE_C = 0.6f;
const float ENV_HUMIDITY_TOLERANCE = 3.0f;
const float PECK_THRESHOLD_MARGIN_MS2 = 0.60f;         // extra kick above baseline vibration for a peck
// We store these to display them on the new screen
float vizDx = 0.0f, vizDy = 0.0f, vizDz = 0.0f;
float vizPeakX = 0.0f, vizPeakY = 0.0f, vizPeakZ = 0.0f;
// --- VISUALIZATION GLOBALS ---
// Live Raw Values
float vizRawX = 0.0f, vizRawY = 0.0f, vizRawZ = 0.0f;

// Timer for resetting peaks
unsigned long lastPeakResetMs = 0;

// -------------------- VL53L1X --------------------
#define SENSOR_CHANNELS     4            // channels 0..3 used for sensors
#define SENSORS_PER_CHANNEL 3
#define SENSOR_COUNT        (SENSOR_CHANNELS * SENSORS_PER_CHANNEL) // 12
// uint16_t distances[SENSOR_COUNT]; // SENSOR_COUNT == 12
// --- GLOBAL STATE VARIABLES ---
uint16_t latestDistances[SENSOR_COUNT] = {0}; // Holds the last valid reading
uint8_t  latestStatuses[SENSOR_COUNT]  = {0}; // Holds the last valid status

// XSHUT pins for sensors 1..12 (in order). NOTE: pin 1 is TX0 on Mega; avoid Serial while using it.
const uint8_t xshutPins[SENSOR_COUNT] = {
  13,12,11, 10,9,8,   // sensors 1..6 on channels 0 & 1
   6, 5, 4,  3,2,1    // sensors 7..12 on channels 2 & 3
};

// We reuse the same 3 I2C addresses per channel; the multiplexer isolates them
const uint8_t perChannelAddr[SENSORS_PER_CHANNEL] = { 0x30, 0x31, 0x32 };

VL53L1X sensors[SENSOR_COUNT];

struct BaselineValues {
  uint16_t distances[SENSOR_COUNT];
  bool distanceValid[SENSOR_COUNT];
  float tempC;
  float humidity;
  float accelNoise;
  bool valid;
};

struct BaselineAccumulator {
  uint32_t distanceSum[SENSOR_COUNT];
  uint16_t distanceSamples[SENSOR_COUNT];
  float tempSum;
  float humiditySum;
  float accelNoiseSum;
  uint16_t samples;
  uint16_t tempSamples;
  uint16_t humiditySamples;
  uint16_t accelSamples;
};

struct SensorSnapshot {
  DateTime timestamp;
  uint16_t distances[SENSOR_COUNT];
  uint8_t statuses[SENSOR_COUNT];
  float tempC;
  float humidity;
  float accelMag;
  bool peckActive;
};

enum SamplingState {
  STATE_BASELINING,
  STATE_SLEEPING,
  STATE_FAST_SAMPLING
};

SamplingState samplingState = STATE_BASELINING;
BaselineValues sessionBaseline = {};
BaselineAccumulator baselineAcc = {};
unsigned long baselineStartMs = 0;
unsigned long fastModeStartMs = 0;
unsigned long stableSinceMs = 0;
unsigned long lastChangeDuringFastMs = 0;
SensorSnapshot lastSnapshot;
float peckNoiseBaseline = 0.0f;
char lastButtonEvent[16] = "";
bool buttonEventPending = false;
bool foodDeliveredSinceLastLog = false;

// Simple buzzer helpers so we can try tone() first and fall back to a direct drive if needed.
// Set BUZZER_USE_TONE to false near the pin definitions to switch to the direct pulse.
void buzzTest(uint16_t freqHz = BUZZER_TEST_FREQ_HZ, uint16_t durationMs = BUZZER_TONE_DURATION_MS) {
  if (BUZZER_USE_TONE) {
    tone(BUZZER_PIN, freqHz, durationMs);
    delay(durationMs);
    noTone(BUZZER_PIN);
  } else {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(BUZZER_PULSE_DURATION_MS);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void playStartupMelody() {
  if (BUZZER_USE_TONE) {
    tone(BUZZER_PIN, BUZZER_MELODY_FREQ_LOW_HZ, BUZZER_MELODY_STROKE_MS);
    delay(BUZZER_MELODY_STROKE_MS);
    noTone(BUZZER_PIN);

    delay(BUZZER_MELODY_PAUSE_MS);

    tone(BUZZER_PIN, BUZZER_MELODY_FREQ_LOW_HZ, BUZZER_MELODY_STROKE_MS / 2);
    delay(BUZZER_MELODY_STROKE_MS / 2);

    tone(BUZZER_PIN, BUZZER_MELODY_FREQ_HIGH_HZ, BUZZER_MELODY_STROKE_MS / 2);
    delay(BUZZER_MELODY_STROKE_MS / 2);
    noTone(BUZZER_PIN);
  } else {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(BUZZER_MELODY_STROKE_MS);
    digitalWrite(BUZZER_PIN, LOW);

    delay(BUZZER_MELODY_PAUSE_MS);

    digitalWrite(BUZZER_PIN, HIGH);
    delay(BUZZER_MELODY_STROKE_MS / 2);
    digitalWrite(BUZZER_PIN, LOW);

    digitalWrite(BUZZER_PIN, HIGH);
    delay(BUZZER_MELODY_STROKE_MS / 2);
    digitalWrite(BUZZER_PIN, LOW);
  }
}
void playShutdownMelody() {
  if (BUZZER_USE_TONE) {
    tone(BUZZER_PIN, BUZZER_MELODY_FREQ_HIGH_HZ, BUZZER_MELODY_STROKE_MS);
    delay(BUZZER_MELODY_STROKE_MS);
    noTone(BUZZER_PIN);

    delay(BUZZER_MELODY_PAUSE_MS);

    tone(BUZZER_PIN, BUZZER_MELODY_FREQ_HIGH_HZ, BUZZER_MELODY_STROKE_MS / 2);
    delay(BUZZER_MELODY_STROKE_MS / 2);

    tone(BUZZER_PIN, BUZZER_MELODY_FREQ_LOW_HZ, BUZZER_MELODY_STROKE_MS / 2);
    delay(BUZZER_MELODY_STROKE_MS / 2);
    noTone(BUZZER_PIN);
  } else {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(BUZZER_MELODY_STROKE_MS);
    digitalWrite(BUZZER_PIN, LOW);

    delay(BUZZER_MELODY_PAUSE_MS);

    digitalWrite(BUZZER_PIN, HIGH);
    delay(BUZZER_MELODY_STROKE_MS / 2);
    digitalWrite(BUZZER_PIN, LOW);

    digitalWrite(BUZZER_PIN, HIGH);
    delay(BUZZER_MELODY_STROKE_MS / 2);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void playBatteryTriplet(uint16_t freqHz) {
  for (uint8_t i = 0; i < 3; i++) {
    if (BUZZER_USE_TONE) {
      tone(BUZZER_PIN, freqHz, BATTERY_WARNING_BEEP_MS);
      delay(BATTERY_WARNING_BEEP_MS);
      noTone(BUZZER_PIN);
    } else {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(BATTERY_WARNING_BEEP_MS);
      digitalWrite(BUZZER_PIN, LOW);
    }
    delay(BATTERY_WARNING_GAP_MS);
  }
}

void drawFlashingBatteryIcon(int percent, bool filled) {
  const uint8_t iconW = 24;
  const uint8_t iconH = 12;
  const uint8_t iconX = 0;
  const uint8_t iconY = 0;

  display.drawRect(iconX, iconY, iconW, iconH, SSD1306_WHITE);
  display.fillRect(iconX + iconW, iconY + 3, 2, iconH - 6, SSD1306_WHITE);

  if (filled) {
    int cappedPercent = constrain(percent, 0, 100);
    const uint8_t fillMaxW = iconW - 4;
    uint8_t fillW = (uint8_t)(fillMaxW * cappedPercent / 100.0f);
    if (fillW > 0) {
      display.fillRect(iconX + 2, iconY + 2, fillW, iconH - 4, SSD1306_WHITE);
    }
  }
}

void showBatteryWarningScreen(int percent, const __FlashStringHelper* line1, const __FlashStringHelper* line2, uint16_t beepFreqHz) {
  for (uint8_t cycle = 0; cycle < 2; cycle++) {
    OLED_SELECT();
    display.clearDisplay();
    bool fillIcon = (cycle % 2 == 0);
    drawFlashingBatteryIcon(percent, fillIcon);

    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(6, 16);
    display.println(line1);
    display.setCursor(6, 40);
    display.println(line2);
    display.display();

    playBatteryTriplet(beepFreqHz);
    delay(240);
  }
}

// --- power switch debounce state ---
const unsigned long SWITCH_DEBOUNCE_DELAY_MS = 200;
const unsigned long SWITCH_STARTUP_GRACE_MS = 1000;
unsigned long switchHighStart = 0;
unsigned long startupMillis = 0;
bool shutdownInitiated = false;

// void timerISR() {
//   if (buttonScanPending < 255) {
//     buttonScanPending++;
//   }

//   sensorUpdateTickCounter++;
//   if (sensorUpdateTickCounter >= sensorUpdateIntervalTicks) {
//     sensorUpdateTickCounter = 0;
//     sensorUpdateFlag = true;
//   }
// }
void timerISR() {
  // 1. Actually SCAN the buttons right now!
  // This ensures we catch the press even if the main loop is busy.
  scanButtons(millis());

  // 2. Handle Sensor Timing
  sensorUpdateTickCounter++;
  if (sensorUpdateTickCounter >= sensorUpdateIntervalTicks) {
    sensorUpdateTickCounter = 0;
    sensorUpdateFlag = true;
  }
}

void setSensorIntervalMs(unsigned long intervalMs) {
  noInterrupts();
  sensorUpdateIntervalTicks = (uint16_t)intervalMs;
  sensorUpdateTickCounter = 0;
  interrupts();
}

// -------------------- STEP/DIR DRIVER --------------------
// Existing driver: STEP on pin 33, DIR on pin 31
#define STEP_PIN       33
#define DIR_PIN        31
// Second driver: STEP on pin 34, DIR on pin 36
#define STEP_PIN2      34
#define DIR_PIN2       36
#define EN_PIN         37
#define EN_PIN2        30
#define MICROSTEPS      8     // set to the hardware microstep setting (1,2,4,8,16, ...)
#define STEPS_PER_REV      200     // 1.8° motor
#define STEPS_PER_REV_FOOD  200     // gear motor for food dispenser
const int STEPS_90      = (STEPS_PER_REV * MICROSTEPS) / 4;        // quarter turn main motor
const int STEPS_45_FOOD = (STEPS_PER_REV_FOOD * MICROSTEPS) / 8;  // 360 degrees turn food motor
const int STEPS_DELOCK_FOOD = (int)((STEPS_PER_REV_FOOD * (long)MICROSTEPS * 1L) / 360L); // ~20° back-off

// pulse timing (adjust for your driver/motor)
const unsigned int STEP_PULSE_US = 1500;   // high/low pulse width (slower for precise tunnel alignment)
const unsigned int STEP_PULSE_FOOD_US = 1000; // high/low pulse width for food motor
const float TUNNEL_MAX_SPEED_STEPS_S = 220.0f;  // slower for precise hall alignment
const float TUNNEL_ACCEL_STEPS_S2    = 400.0f;  // gentler ramp to reduce overshoot
const float FOOD_MAX_SPEED_STEPS_S = 800.0f;   // top speed with torque-preserving ramping
const float FOOD_ACCEL_STEPS_S2    = 1200.0f;  // acceleration profile to prevent stalls
const unsigned long TUNNEL_VIBRATION_DURATION_MS = 1200; // brief shake to clear stuck pellets

// Fine-tune how far past the hall sensor the tunnel should travel to align with the opening
// Extra microstepped nudge after the hall sensor triggers to finish alignment.
const uint8_t ALIGNMENT_OVERSHOOT_STEPS = 10;
// Additional steps to park the tunnel halfway between two sides (~23 degrees)
const uint16_t TUNNEL_PARK_STEPS = (uint16_t)((STEPS_PER_REV * (long)MICROSTEPS * 45L) / 360L);

// --- state for motor trigger & switch edge ---
unsigned long lastMoveMs = 0;
const unsigned long moveCooldownMs = 1500;

bool sdAvailable = false;
int currentLogYear = -1;
int currentLogMonth = -1;
int currentLogDay = -1;
char currentLogFilename[16] = "";

// Hall effect alignment sensors by tunnel side (1..4)
const uint8_t sideHallPins[4] = {
  14, // Side 1 (formerly hall 4)
  17, // Side 2 (formerly hall 1)
  16, // Side 3 (formerly hall 2)
  15  // Side 4 (formerly hall 3)
};

// Button panels (active LOW). Panels are ordered 4,3,2,1 to preserve existing behavior that
// watches the first entry for panel 4 button 1. Panels 4 & 3 use analog pins A4..A11, panels 2 & 1
// use digital pins 49..42. Panel 1's wiring is reversed: button 4 is on pin 48 down to button 1 on pin 42.
const uint8_t BUTTON_COUNT = 16;
const uint8_t PANEL_UNKNOWN = 0xFF;
const uint8_t buttonPins[BUTTON_COUNT] = {A4, A5, A6, A7, A8, A9, A10, A11, 49, 47, 45, 43, 48, 46, 44, 42};
// PANEL_UNKNOWN marks legacy buttons where the panel number is not yet defined
const uint8_t buttonPanels[BUTTON_COUNT] = {4, 4, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1};
const uint8_t buttonNumbers[BUTTON_COUNT] = {1, 2, 4, 3, 1, 2, 4, 3, 1, 2, 4, 3, 3, 4, 2, 1};
const unsigned long BUTTON_DEBOUNCE_MS = 5;
volatile uint8_t buttonRawState[BUTTON_COUNT]  = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
volatile uint8_t buttonState[BUTTON_COUNT]     = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
uint8_t prevButtonState[BUTTON_COUNT]          = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
volatile unsigned long buttonLastChange[BUTTON_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile uint16_t buttonPendingMask = 0;
volatile uint16_t buttonEventMask = 0;

AccelStepper foodStepper(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);
AccelStepper tunnelStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

enum ButtonPortGroup : uint8_t {
  BUTTON_PORT_F = 0,
  BUTTON_PORT_K,
  BUTTON_PORT_L,
  BUTTON_PORT_COUNT
};

volatile uint8_t * buttonPortInputRegs[BUTTON_PORT_COUNT] = {nullptr, nullptr, nullptr};


const uint8_t buttonPortGroup[BUTTON_COUNT] = {
  BUTTON_PORT_F, BUTTON_PORT_F, BUTTON_PORT_F, BUTTON_PORT_F,
  BUTTON_PORT_K, BUTTON_PORT_K, BUTTON_PORT_K, BUTTON_PORT_K,
  BUTTON_PORT_L, BUTTON_PORT_L, BUTTON_PORT_L, BUTTON_PORT_L,
  BUTTON_PORT_L, BUTTON_PORT_L, BUTTON_PORT_L, BUTTON_PORT_L
};

uint8_t buttonBitMask[BUTTON_COUNT] = {0};

void initButtonPortMetadata() {
  buttonPortInputRegs[BUTTON_PORT_F] = portInputRegister(digitalPinToPort(A4));
  buttonPortInputRegs[BUTTON_PORT_K] = portInputRegister(digitalPinToPort(A8));
  buttonPortInputRegs[BUTTON_PORT_L] = portInputRegister(digitalPinToPort(49));

  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    buttonBitMask[i] = digitalPinToBitMask(buttonPins[i]);
  }

  unsigned long now = millis();

  for (uint8_t group = 0; group < BUTTON_PORT_COUNT; group++) {
    volatile uint8_t *portReg = buttonPortInputRegs[group];
    uint8_t portValue = portReg ? *portReg : 0xFF;
    for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
      if (buttonPortGroup[i] != group) continue;
      uint8_t state = (portValue & buttonBitMask[i]) ? HIGH : LOW;
      buttonRawState[i] = state;
      buttonState[i] = state;
      prevButtonState[i] = state;
      buttonLastChange[i] = now;
    }
  }

  buttonPendingMask = 0;
  buttonEventMask = 0;
}


// Forward declare the storage struct so Arduino's autogenerated prototypes know about it.
struct SequenceStorage;

// --- button sequence & menu state ---
enum SystemMode {
  MODE_IDLE,
  MODE_MENU_MAIN,           // <--- NEW: Top Level
  MODE_MENU_SELECT_SIDE,    // (Submenu 1)
  MODE_MENU_SELECT_LENGTH,
  MODE_MENU_DISPLAY_SELECT, // (Submenu 2)
  MODE_MENU_OPTIONS,        // (Submenu 3)
  MODE_MENU_ENTER_SEQUENCE,
  MODE_MENU_CONFIRM,
  MODE_MENU_RESET_CONFIRM
};

SystemMode currentMode = MODE_IDLE;

const unsigned long MENU_HOLD_MS = 3000;
bool panel1MenuHoldActive = false;
unsigned long menuHoldStart = 0;

bool menuAwaitingRelease = false;
uint8_t menuSelectedSide = 0;
uint8_t menuSequenceBuffer[MAX_SEQUENCE_LENGTH];
uint8_t menuSequenceLength = 0;
uint8_t menuSequenceTargetLength = DEFAULT_SEQUENCE_LENGTH;

const uint8_t panel1Indices[4] = {12, 13, 14, 15};

// --- tiny sleeping bird bitmap (32x16) ---
const uint8_t BIRD_W = 64, BIRD_H = 64;
const uint8_t PROGMEM birdBitmap[4096] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xfe, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x01, 0xf6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xfb, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x03, 0xfb, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe6, 0xe0, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x70, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x01, 0xe0, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xe0, 0x0c, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x01, 0xe0, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xe0, 0x03, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x03, 0x00, 0x00, 0x00, 0x00, 0xff, 0x03, 0xe0, 0x01, 0x80, 
	0x00, 0x00, 0x07, 0xff, 0xff, 0xe0, 0x00, 0x80, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xe0, 0x00, 0xc0, 
	0x00, 0x00, 0x7f, 0xff, 0xff, 0xc0, 0x00, 0x40, 0x00, 0x00, 0xff, 0xff, 0xff, 0x80, 0x00, 0x40, 
	0x00, 0x01, 0xff, 0xff, 0xff, 0x80, 0x00, 0x40, 0x00, 0x03, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 
	0x00, 0x07, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 
	0x00, 0x1f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 
	0x00, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 
	0x01, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x07, 0xef, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 
	0x07, 0xbd, 0xbe, 0xfa, 0x00, 0x00, 0x00, 0x00, 0x06, 0xd2, 0x7f, 0x90, 0x00, 0x00, 0x00, 0x00, 
	0x0c, 0x03, 0xdc, 0x78, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x38, 0x38, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x30, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x40, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x20, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0xc0, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x60, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x01, 0x80, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x60, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x80, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x01, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x02, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x02, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x06, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x0e, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0xe0, 0x4f, 0xf0, 0x00, 0x00, 0x00, 
	0x00, 0x11, 0xa0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// -------------------- Helpers --------------------
void oledPrintf(uint8_t x, uint8_t y, const __FlashStringHelper* label, int value, const __FlashStringHelper* unit = nullptr) {
  tcaSelect(SCREEN_CHANNEL);
  display.setCursor(x, y);
  display.print(label);
  display.print(value);
  if (unit) display.print(unit);
}

void motorStep(int steps, bool dirCW) {
  tunnelStepper.enableOutputs();
  tunnelStepper.setCurrentPosition(0);
  tunnelStepper.moveTo(dirCW ? steps : -steps);
  tunnelStepper.runToPosition();
}

void motorStepFood(int steps, bool dirCW, bool disableAfter = true) {
  foodStepper.enableOutputs();
  foodStepper.setMaxSpeed(FOOD_MAX_SPEED_STEPS_S);
  foodStepper.setAcceleration(FOOD_ACCEL_STEPS_S2);
  foodStepper.setCurrentPosition(0);
  foodStepper.moveTo(dirCW ? steps : -steps);
  while (foodStepper.distanceToGo() != 0) {
    foodStepper.run();
  }

  if (disableAfter) {
    foodStepper.disableOutputs();
  }
}

void vibrateTunnel(unsigned long durationMs) {
  const int amplitudeSteps = ALIGNMENT_OVERSHOOT_STEPS;
  unsigned long start = millis();
  bool dirCW = true;

  tunnelStepper.enableOutputs();
  tunnelStepper.setCurrentPosition(0);

  while (millis() - start < durationMs) {
    long target = dirCW ? amplitudeSteps : -amplitudeSteps;
    tunnelStepper.moveTo(target);
    while (tunnelStepper.distanceToGo() != 0 && millis() - start < durationMs) {
      tunnelStepper.run();
    }
    dirCW = !dirCW;
  }

  tunnelStepper.stop();
  tunnelStepper.setCurrentPosition(0);
}

bool isSideAligned(uint8_t side) {
  if (side < 1 || side > SIDE_COUNT) return false;
  uint8_t hallPin = sideHallPins[side - 1];
  return digitalRead(hallPin) == LOW;
}

bool stepMotorUntilAligned(uint8_t hallPin, bool dirCW, unsigned long maxSteps) {
  if (digitalRead(hallPin) == LOW) {
    return true;
  }

  bool aligned = false;
  tunnelStepper.enableOutputs();
  tunnelStepper.setCurrentPosition(0);
  tunnelStepper.moveTo(dirCW ? (long)maxSteps : -(long)maxSteps);

  long lastPosition = tunnelStepper.currentPosition();
  while (tunnelStepper.distanceToGo() != 0) {
    tunnelStepper.run();
    long pos = tunnelStepper.currentPosition();
    if (pos != lastPosition) {
      lastPosition = pos;
      if (digitalRead(hallPin) == LOW) {
        tunnelStepper.stop();
        long alignTarget = tunnelStepper.currentPosition() + (dirCW ? ALIGNMENT_OVERSHOOT_STEPS : -ALIGNMENT_OVERSHOOT_STEPS);
        tunnelStepper.moveTo(alignTarget); // small nudge past the hall sensor for final alignment
        while (tunnelStepper.distanceToGo() != 0) {
          tunnelStepper.run();
        }
        tunnelStepper.disableOutputs();
        aligned = true;
        break;
      }
    }
  }

  return aligned || digitalRead(hallPin) == LOW;
}

bool rotateTunnelToSide(uint8_t targetSide) {
  if (targetSide < 1 || targetSide > SIDE_COUNT) return false;

  uint8_t hallPin = sideHallPins[targetSide - 1];

  if (isSideAligned(targetSide)) {
    currentTunnelSide = targetSide;
    return true;
  }

  uint8_t cwQuarterTurns = (targetSide + SIDE_COUNT - currentTunnelSide) % SIDE_COUNT;
  uint8_t ccwQuarterTurns = (currentTunnelSide + SIDE_COUNT - targetSide) % SIDE_COUNT;

  bool dirCW = true;
  uint8_t quarterTurns = cwQuarterTurns;
  if (ccwQuarterTurns < cwQuarterTurns) {
    dirCW = false;
    quarterTurns = ccwQuarterTurns;
  }

  if (quarterTurns == 0) {
    quarterTurns = SIDE_COUNT;
  }

  unsigned long maxSteps = (unsigned long)quarterTurns * STEPS_90 + (STEPS_90 / 2) + ALIGNMENT_OVERSHOOT_STEPS;
  bool aligned = stepMotorUntilAligned(hallPin, dirCW, maxSteps);

  if (!aligned && quarterTurns != SIDE_COUNT) {
    unsigned long fullRotationSteps = (unsigned long)SIDE_COUNT * STEPS_90 + (STEPS_90 / 2) + ALIGNMENT_OVERSHOOT_STEPS;
    aligned = stepMotorUntilAligned(hallPin, dirCW, fullRotationSteps);
  }

  if (aligned) {
    currentTunnelSide = targetSide;
  }

  return aligned;
}

void parkTunnelBetweenSides() {
  if (!isSideAligned(currentTunnelSide)) {
    rotateTunnelToSide(currentTunnelSide);
  }

  motorStep(TUNNEL_PARK_STEPS, true);
}

bool initSensorOn(uint8_t ch, uint8_t idxInCh) {
  // idxInCh: 0..2 within the channel
  const uint8_t globalIndex = ch * SENSORS_PER_CHANNEL + idxInCh;
  const uint8_t pin = xshutPins[globalIndex];

  // bring this sensor out of reset
  tcaSelect(ch);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(10);

  sensors[globalIndex].setTimeout(1000);
  if (!sensors[globalIndex].init()) return false;

  sensors[globalIndex].setAddress(perChannelAddr[idxInCh]);
  sensors[globalIndex].setDistanceMode(VL53L1X::Short);         // Short/Medium/Long
  sensors[globalIndex].setMeasurementTimingBudget(20000);        // µs
  sensors[globalIndex].startContinuous(20);                      // ms between readings
  return true;
}

void resetChannelGroup(uint8_t ch) {
  // Hold all sensors on this channel in reset (XSHUT LOW)
  for (uint8_t i = 0; i < SENSORS_PER_CHANNEL; i++) {
    uint8_t gi = ch * SENSORS_PER_CHANNEL + i;
    pinMode(xshutPins[gi], OUTPUT);
    digitalWrite(xshutPins[gi], LOW);
  }
}

bool connectUnoR4() {
  if (!expectUnoR4) {
    return true;
  }
  UNO_R4_SELECT();
  delay(3);

  // Pure read triggers onRequest() on the Uno
  if (Wire.requestFrom(UNO_R4_ADDR, (uint8_t)1) != 1) return false;
  char k = Wire.read();

  // Optional: OLED print
  tcaSelect(SCREEN_CHANNEL);
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(F("UNO R4: "));
  display.println(k);
  display.display();

  return (k == 'K');
}



// Sends header + 12 uint16_t as little-endian (lo,hi)
bool sendDistancesToR4(const uint16_t *vals, uint8_t count) {
  if (!expectUnoR4) {
    return true;
  }
  UNO_R4_SELECT();
  delay(3); // TCA settle

  Wire.beginTransmission(UNO_R4_ADDR);
  for (uint8_t i = 0; i < count; i++) {
    uint16_t v = vals[i];
    Wire.write((uint8_t)(v & 0xFF));        // low byte
    Wire.write((uint8_t)((v >> 8) & 0xFF)); // high byte
  }
  uint8_t err = Wire.endTransmission();
  if (err != 0 && expectUnoR4) {
    OLED_SELECT();
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(12, 10);
    display.println(F("No"));
    display.setCursor(12, 32);
    display.println(F("send:("));
    display.display();
    delay(1000); // show it briefly
  }
  return (err == 0);
}
bool sendDistancesToR4_chunked(const uint16_t* vals) {
  if (!expectUnoR4) {
    return true;
  }
  for (uint8_t off = 0; off < SENSOR_COUNT; off += 6) { // 6 values = 12 bytes
    UNO_R4_SELECT();
    delay(3);

    Wire.beginTransmission(UNO_R4_ADDR);
    for (uint8_t i = 0; i < 6; i++) {
      uint16_t v = vals[off + i];
      Wire.write((uint8_t)(v & 0xFF));
      Wire.write((uint8_t)(v >> 8));
    }
    uint8_t err = Wire.endTransmission();
    if (err && expectUnoR4) {
      // show err on OLED if you want
      return false;
    }
    delayMicroseconds(200); // tiny gap
  }
  return true;
}

// Send distances in framed chunks (4 values per frame)
// Send distances in framed chunks (4 values per frame)
bool sendDistancesFramed(const uint16_t *vals) {
  if (!expectUnoR4) {
    return true;
  }
  
  const uint8_t CHUNK = 4;  // 4 values = 8 data bytes + 3 header = 11 total
  
  for (uint8_t off = 0; off < SENSOR_COUNT; off += CHUNK) {
    uint8_t cnt = (off + CHUNK <= SENSOR_COUNT) ? CHUNK : (SENSOR_COUNT - off);

    UNO_R4_SELECT();
    // No delay needed here if we aren't switching constantly, 
    // but a tiny one helps the multiplexer settle.
    delayMicroseconds(10); 

    Wire.beginTransmission(UNO_R4_ADDR);
    Wire.write('D');        // header
    Wire.write(off);        // start index
    Wire.write(cnt);        // count

    for (uint8_t i = 0; i < cnt; i++) {
      uint16_t v = vals[off + i];
      Wire.write((uint8_t)(v & 0xFF));       // low byte
      Wire.write((uint8_t)((v >> 8) & 0xFF));// high byte
    }

    uint8_t err = Wire.endTransmission();    // 0 = OK
    
    if (err != 0) {
      // FIX: Do NOT pause or draw to OLED here. 
      // If the R4 is missing, just return false and keep running.
      return false; 
    }

    delayMicroseconds(200); // small gap between frames
  }
  return true;
}

bool ensureLogFile(DateTime now) {
  if (!sdAvailable) {
    return false;
  }

  if (currentLogYear == now.year() &&
      currentLogMonth == now.month() &&
      currentLogDay == now.day() &&
      currentLogFilename[0] != '\0') {
    return true;
  }

  currentLogYear = now.year();
  currentLogMonth = now.month();
  currentLogDay = now.day();
  snprintf(currentLogFilename, sizeof(currentLogFilename), "%04d%02d%02d.txt", currentLogYear, currentLogMonth, currentLogDay);

  bool fileExists = SD.exists(currentLogFilename);
  File file = SD.open(currentLogFilename, FILE_WRITE);
  if (!file) {
    return false;
  }

  if (!fileExists) {
    file.print(F("Date, Time"));
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      file.print(F(", Sensor"));
      file.print(i + 1);
      file.print(F(" (mm)"));
    }
    file.print(F(", Temp (C), Humidity (%), Peck, Food, State, Button"));
    file.println();
  }

  file.flush();
  file.close();
  return true;
}

void appendLogEntry(DateTime timestamp, const uint16_t *distances, const uint8_t *statuses, bool peckActive, bool foodDelivered = false, const char* stateLabel = "", const char* buttonLabel = "") {
  if (!sdAvailable) {
    return;
  }

  if (!ensureLogFile(timestamp)) {
    return;
  }

  File file = SD.open(currentLogFilename, FILE_WRITE);
  if (!file) {
    return;
  }

  file.print(timestamp.year());
  file.print('-');
  if (timestamp.month() < 10) file.print('0');
  file.print(timestamp.month());
  file.print('-');
  if (timestamp.day() < 10) file.print('0');
  file.print(timestamp.day());
  file.print(F(", "));
  if (timestamp.hour() < 10) file.print('0');
  file.print(timestamp.hour());
  file.print(':');
  if (timestamp.minute() < 10) file.print('0');
  file.print(timestamp.minute());
  file.print(':');
  if (timestamp.second() < 10) file.print('0');
  file.print(timestamp.second());

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    file.print(F(", "));
    if (statuses[i] == 0) {
      file.print(distances[i]);
    } else {
      file.print(F("timeout"));
    }
  }

  file.print(F(", "));
  if (isnan(lastTempC)) {
    file.print(F("n/a"));
  } else {
    file.print(lastTempC, 2);
  }

  file.print(F(", "));
  if (isnan(lastHumidity)) {
    file.print(F("n/a"));
  } else {
    file.print(lastHumidity, 2);
  }

  file.print(F(", "));
  file.print(peckActive ? F("yes") : F("no"));
  file.print(F(", "));
  file.print(foodDelivered ? F("yes") : F("no"));
  file.print(F(", "));
  file.print(stateLabel);
  file.print(F(", "));
  file.println(buttonLabel);
  file.flush();
  file.close();
}

void appendSessionHeader(DateTime now) {
  if (!sdAvailable) {
    return;
  }

  if (!ensureLogFile(now)) {
    return;
  }

  File file = SD.open(currentLogFilename, FILE_WRITE);
  if (!file) {
    return;
  }

  file.println();
  file.print(F("-------------- "));
  file.print(now.year());
  file.print('-');
  if (now.month() < 10) file.print('0');
  file.print(now.month());
  file.print('-');
  if (now.day() < 10) file.print('0');
  file.print(now.day());
  file.println(F(" --------"));

  float voltage = readBatteryVoltage();
  int percent = batteryPercentFromVoltage(voltage);
  file.print(F("Battery: "));
  file.print(voltage, 2);
  file.print(F("V ("));
  file.print(percent);
  file.println(F("%)"));
  file.flush();
  file.close();
}

void appendShutdownNotice() {
  if (!sdAvailable) {
    return;
  }

  RTC_SELECT();
  DateTime now = rtc.now();
  if (!ensureLogFile(now)) {
    return;
  }

  File file = SD.open(currentLogFilename, FILE_WRITE);
  if (!file) {
    return;
  }

  file.println();
  file.print(F("Shutdown starting at "));
  file.print(now.year());
  file.print('-');
  if (now.month() < 10) file.print('0');
  file.print(now.month());
  file.print('-');
  if (now.day() < 10) file.print('0');
  file.print(now.day());
  file.print(' ');
  if (now.hour() < 10) file.print('0');
  file.print(now.hour());
  file.print(':');
  if (now.minute() < 10) file.print('0');
  file.print(now.minute());
  file.print(':');
  if (now.second() < 10) file.print('0');
  file.print(now.second());
  float voltage = readBatteryVoltage();
  int percent = batteryPercentFromVoltage(voltage);
  file.print(F(", Battery: "));
  file.print(voltage, 2);
  file.print(F("V ("));
  file.print(percent);
  file.println(F("%)"));
  file.flush();
  file.close();
}

void flushSdCard() {
  if (!sdAvailable || currentLogFilename[0] == '\0') {
    return;
  }

  File file = SD.open(currentLogFilename, FILE_WRITE);
  if (file) {
    file.flush();
    file.close();
  }
}

// void scanButtons() {
//   unsigned long now = millis();
//   uint8_t portValues[BUTTON_PORT_COUNT];

//   for (uint8_t group = 0; group < BUTTON_PORT_COUNT; group++) {
//     volatile uint8_t *portReg = buttonPortInputRegs[group];
//     portValues[group] = portReg ? *portReg : 0xFF;
//   }

//   for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
//     uint8_t group = buttonPortGroup[i];
//     uint8_t newState = (portValues[group] & buttonBitMask[i]) ? HIGH : LOW;
//     if (newState != buttonRawState[i]) {
//       buttonRawState[i] = newState;
//       buttonLastChange[i] = now;
//       buttonPendingMask |= (uint16_t)1 << i;
//     }
//   }
// }

// Update this function to accept 'now' as an argument
void scanButtons(unsigned long now) {
  // Read all ports immediately (Direct Port Manipulation is fast enough for ISR)
  uint8_t portValues[BUTTON_PORT_COUNT];
  for (uint8_t group = 0; group < BUTTON_PORT_COUNT; group++) {
    volatile uint8_t *portReg = buttonPortInputRegs[group];
    portValues[group] = portReg ? *portReg : 0xFF;
  }

  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    uint8_t group = buttonPortGroup[i];
    uint8_t newState = (portValues[group] & buttonBitMask[i]) ? HIGH : LOW;
    
    // Detect Change
    if (newState != buttonRawState[i]) {
      buttonRawState[i] = newState;
      buttonLastChange[i] = now;
      // We found a change, mark it pending so the main loop can handle the Logic later
      buttonPendingMask |= (uint16_t)1 << i; 
    }
  }
}


bool allPanel1Pressed() {
  for (uint8_t i = 0; i < 4; i++) {
    if (buttonState[panel1Indices[i]] != LOW) return false;
  }
  return true;
}

bool allPanel1Released() {
  for (uint8_t i = 0; i < 4; i++) {
    if (buttonState[panel1Indices[i]] == LOW) return false;
  }
  return true;
}

void resetMenuSequenceBuffer() {
  menuSequenceLength = 0;
  for (uint8_t i = 0; i < MAX_SEQUENCE_LENGTH; i++) {
    menuSequenceBuffer[i] = 0;
  }
}

float readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  float sensedVoltage = (raw / ADC_MAX_READING) * ADC_REF_V;
  float dividerScale = (BATTERY_R1_OHMS + BATTERY_R2_OHMS) / BATTERY_R2_OHMS;
  return sensedVoltage * dividerScale;
}

int readBatteryPercent() {
  float voltage = readBatteryVoltage();
  return batteryPercentFromVoltage(voltage);
}

int batteryPercentFromVoltage(float voltage) {
  float pct = ((voltage - BATTERY_MIN_V) * 100.0f) / (BATTERY_MAX_V - BATTERY_MIN_V);
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return (int)(pct + 0.5f);
}

bool isChargerConnected() {
  return digitalRead(CHARGER_DETECT_PIN) == HIGH;
}

bool shouldShowBatteryIndicator(int percent, unsigned long nowMs) {
  if (percent > 20) return true;
  return ((nowMs / 1000) % 2) == 0;
}

void showStartupBatteryWarnings() {
  int percent = readBatteryPercent();

  if (percent <= 5) {
    showBatteryWarningScreen(percent, F("I'm too tired!"), F("Charge me"), 2000);
    shutdownInitiated = true;
    shutdownSequence();
    return;
  }

  if (percent <= 10) {
    showBatteryWarningScreen(percent, F("Please"), F("charge me"), 3000);
    return;
  }

  if (percent <= 20) {
    showBatteryWarningScreen(percent, F("Low batt"), F("Charge me"), 2000);
  }
}

void showShutdownBatteryWarnings() {
  int percent = readBatteryPercent();

  if (percent <= 5) {
    showBatteryWarningScreen(percent, F("I'm too tired!"), F("Charge me"), 2000);
    return;
  }

  if (percent <= 10) {
    showBatteryWarningScreen(percent, F("Please"), F("charge me"), 3000);
    return;
  }

  if (percent <= 20) {
    showBatteryWarningScreen(percent, F("Low batt"), F("Charge soon"), 2000);
  }
}

void drawBatteryStatus(unsigned long nowMs) {
  float voltage = readBatteryVoltage();
  int percent = batteryPercentFromVoltage(voltage);
  bool chargerConnected = isChargerConnected();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.fillRect(0, 0, SCREEN_WIDTH, 10, SSD1306_BLACK);

  if (shouldShowBatteryIndicator(percent, nowMs)) {
    const uint8_t iconW = 16;
    const uint8_t iconH = 8;
    const uint8_t iconX = 0;  // anchor at the left edge
    const uint8_t iconY = 0;

    display.drawRect(iconX, iconY, iconW, iconH, SSD1306_WHITE);
    display.fillRect(iconX + iconW, iconY + 2, 2, iconH - 4, SSD1306_WHITE);

    int cappedPercent = percent;
    if (cappedPercent < 0) cappedPercent = 0;
    if (cappedPercent > 100) cappedPercent = 100;

    const uint8_t fillMaxW = iconW - 2;
    uint8_t fillW = (uint8_t)(fillMaxW * cappedPercent / 100.0f);
    bool showFill = !chargerConnected || ((nowMs / 500) % 2 == 0);
    if (showFill && fillW > 0) {
      display.fillRect(iconX + 1, iconY + 1, fillW, iconH - 2, SSD1306_WHITE);
    }

    display.setCursor(20, 0);
    display.print(percent);
    display.print(F("% "));
    display.print(voltage, 1);
    display.print(F("V"));

  }
}

void updateEnvironmentReadings(unsigned long nowMs) {
  if (!ahtReady) return;
  if (nowMs - lastAhtReadMs < AHT_READ_INTERVAL_MS) return;
  lastAhtReadMs = nowMs;

  RTC_SELECT();
  sensors_event_t humidityEvent, tempEvent;
  aht20.getEvent(&humidityEvent, &tempEvent);

  if (!isnan(tempEvent.temperature)) {
    lastTempC = tempEvent.temperature;
  }
  if (!isnan(humidityEvent.relative_humidity)) {
    lastHumidity = humidityEvent.relative_humidity;
  }
}

float updatePeckDetection(unsigned long nowMs, bool allowDetect) {
  if (!accelReady) return 0.0f;

  // 1. MOTOR BLANKING
  if (nowMs - lastMoveMs < 500) {
    return 0.0f;
  }

  RTC_SELECT();
  sensors_event_t event;
  if (!accel.getEvent(&event)) return 0.0f;

  // 1. Capture Raw "Live" Values for display
  vizRawX = event.acceleration.x;
  vizRawY = event.acceleration.y;
  vizRawZ = event.acceleration.z;

  // 2. High-pass filter calculations
  float dx = event.acceleration.x - accelBaselineX;
  float dy = event.acceleration.y - accelBaselineY;
  float dz = event.acceleration.z - accelBaselineZ;

  // 3. Update Baselines (Slowly adapt to gravity)
  accelBaselineX += ACCEL_BASELINE_ALPHA * dx;
  accelBaselineY += ACCEL_BASELINE_ALPHA * dy;
  accelBaselineZ += ACCEL_BASELINE_ALPHA * dz;

  // 4. Track 5-Second Peaks (Absolute Deltas)
  // This shows the intensity of the "Shock" relative to baseline
  if (nowMs - lastPeakResetMs > 5000) {
    vizPeakX = 0;
    vizPeakY = 0;
    vizPeakZ = 0;
    lastPeakResetMs = nowMs;
  }
  
  if (abs(dx) > vizPeakX) vizPeakX = abs(dx);
  if (abs(dy) > vizPeakY) vizPeakY = abs(dy);
  if (abs(dz) > vizPeakZ) vizPeakZ = abs(dz);

  // 5. Calculate Magnitude for Trigger
  float highpassMag = sqrtf(dx * dx + dy * dy + dz * dz);

  if (allowDetect && highpassMag > PECK_THRESHOLD_MARGIN_MS2) {
    lastPeckDetectedMs = nowMs;
  }

  return highpassMag;
}

bool peckDetectedRecently(unsigned long nowMs) {
  return accelReady && (nowMs - lastPeckDetectedMs) < PECK_HOLD_MS;
}

void resetBaselineAccumulator() {
  memset(&baselineAcc, 0, sizeof(baselineAcc));
}

void recordBaselineSample(const SensorSnapshot& snapshot) {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (snapshot.statuses[i] == 0 && snapshot.distances[i] <= DISTANCE_MAX_BEHAVIOR_MM) {
      baselineAcc.distanceSum[i] += snapshot.distances[i];
      baselineAcc.distanceSamples[i]++;
    }
  }
  if (!isnan(snapshot.tempC)) {
    baselineAcc.tempSum += snapshot.tempC;
    baselineAcc.tempSamples++;
  }
  if (!isnan(snapshot.humidity)) {
    baselineAcc.humiditySum += snapshot.humidity;
    baselineAcc.humiditySamples++;
  }
  if (snapshot.accelMag < 1.0f) {
    baselineAcc.accelNoiseSum += snapshot.accelMag;
    baselineAcc.accelSamples++;
  }
  baselineAcc.samples++;
}

void applySnapshotAsBaseline(const SensorSnapshot& snapshot) {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    bool valid = (snapshot.statuses[i] == 0 && snapshot.distances[i] <= DISTANCE_MAX_BEHAVIOR_MM);
    sessionBaseline.distanceValid[i] = valid;
    sessionBaseline.distances[i] = valid ? snapshot.distances[i] : 0;
  }
  sessionBaseline.tempC = snapshot.tempC;
  sessionBaseline.humidity = snapshot.humidity;
  sessionBaseline.accelNoise = snapshot.accelMag;
  peckNoiseBaseline = snapshot.accelMag;
  sessionBaseline.valid = true;
}

void finalizeBaselineFromAccumulator() {
  if (baselineAcc.samples == 0) return;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (baselineAcc.distanceSamples[i] > 0) {
      sessionBaseline.distances[i] = baselineAcc.distanceSum[i] / baselineAcc.distanceSamples[i];
      sessionBaseline.distanceValid[i] = true;
    } else {
      sessionBaseline.distances[i] = 0;
      sessionBaseline.distanceValid[i] = false;
    }
  }
  sessionBaseline.tempC = (baselineAcc.tempSamples > 0) ? (baselineAcc.tempSum / baselineAcc.tempSamples) : NAN;
  sessionBaseline.humidity = (baselineAcc.humiditySamples > 0) ? (baselineAcc.humiditySum / baselineAcc.humiditySamples) : NAN;
  sessionBaseline.accelNoise = (baselineAcc.accelSamples > 0) ? (baselineAcc.accelNoiseSum / baselineAcc.accelSamples) : 0.0f;
  peckNoiseBaseline = sessionBaseline.accelNoise;
  sessionBaseline.valid = true;
}

bool snapshotWithinBaseline(const SensorSnapshot& snapshot) {
  if (!sessionBaseline.valid) return false;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    bool currentValid = (snapshot.statuses[i] == 0 && snapshot.distances[i] <= DISTANCE_MAX_BEHAVIOR_MM);
    bool baselineValid = sessionBaseline.distanceValid[i];

    if (!baselineValid && currentValid) return false; // cannot compare without a baseline
    if (!currentValid) continue; // ignore far/invalid readings entirely

    uint16_t baseVal = sessionBaseline.distances[i];
    uint16_t currentVal = snapshot.distances[i];
    if (abs((int)currentVal - (int)baseVal) > DISTANCE_BASELINE_TOLERANCE_MM) {
      return false;
    }
  }

  if (!isnan(sessionBaseline.tempC) && !isnan(snapshot.tempC)) {
    if (fabs(sessionBaseline.tempC - snapshot.tempC) > ENV_TEMP_TOLERANCE_C) return false;
  }
  if (!isnan(sessionBaseline.humidity) && !isnan(snapshot.humidity)) {
    if (fabs(sessionBaseline.humidity - snapshot.humidity) > ENV_HUMIDITY_TOLERANCE) return false;
  }

  if (snapshot.accelMag > sessionBaseline.accelNoise + ACCEL_BASELINE_TOLERANCE) return false;

  return true;
}

bool snapshotChangedSince(const SensorSnapshot& a, const SensorSnapshot& b) {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    bool aValid = (a.statuses[i] == 0 && a.distances[i] <= DISTANCE_MAX_BEHAVIOR_MM);
    bool bValid = (b.statuses[i] == 0 && b.distances[i] <= DISTANCE_MAX_BEHAVIOR_MM);
    if (!aValid && !bValid) continue;
    if (aValid != bValid) return true;
    if (abs((int)a.distances[i] - (int)b.distances[i]) > DISTANCE_BASELINE_TOLERANCE_MM) {
      return true;
    }
  }

  if (!isnan(a.tempC) && !isnan(b.tempC) && fabs(a.tempC - b.tempC) > ENV_TEMP_TOLERANCE_C) return true;
  if (!isnan(a.humidity) && !isnan(b.humidity) && fabs(a.humidity - b.humidity) > ENV_HUMIDITY_TOLERANCE) return true;
  if (fabs(a.accelMag - b.accelMag) > ACCEL_BASELINE_TOLERANCE) return true;

  return false;
}

bool captureSensorSnapshot(SensorSnapshot& snapshot) {
  unsigned long nowMs = millis();
  
  // -------------------------------------------------
  // 1. ALWAYS UPDATE TIME & ACCELEROMETER (High Speed)
  // -------------------------------------------------
  RTC_SELECT();
  snapshot.timestamp = rtc.now();

  bool allowPeckDetect = samplingState != STATE_BASELINING;
  // Check peck ONCE per loop, not 12 times inside the sensor loop
  float currentAccelMag = updatePeckDetection(nowMs, allowPeckDetect);
  
  // Update environment (Temperature/Humidity)
  updateEnvironmentReadings(nowMs);
  snapshot.tempC = lastTempC;
  snapshot.humidity = lastHumidity;

  // -------------------------------------------------
  // 2. CONDITIONALLY UPDATE DISTANCE SENSORS (Throttled)
  // -------------------------------------------------
  // Only poll sensors every 25ms. 
  // Sensors run at 20ms, so 25ms ensures data IS ready when we ask.
  static unsigned long lastSensorPollMs = 0;
  bool timeToReadSensors = (nowMs - lastSensorPollMs >= 25);

  if (timeToReadSensors) {
    lastSensorPollMs = nowMs;

    for (uint8_t ch = 0; ch < SENSOR_CHANNELS; ch++) {
      tcaSelect(ch);
      delayMicroseconds(5); // Tiny settling time for Mux

      for (uint8_t i = 0; i < SENSORS_PER_CHANNEL; i++) {
        uint8_t idx = ch * SENSORS_PER_CHANNEL + i;

        // Try to read. If 'E4' is happening, this checks safety.
        // dataReady() is a quick I2C register check.
        if (sensors[idx].dataReady()) { 
            // Valid data waiting? Read it.
            sensors[idx].read();
            latestDistances[idx] = sensors[idx].ranging_data.range_mm;
            latestStatuses[idx]  = sensors[idx].ranging_data.range_status;
        }
        // If data wasn't ready, we simply keep the value from the last pass.
      }
    }
  }

  // -------------------------------------------------
  // 3. FILL SNAPSHOT FROM GLOBAL MEMORY
  // -------------------------------------------------
  // We always fill the snapshot with the latest KNOWN values.
  // This prevents "0" flickering between reads.
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    snapshot.distances[i] = latestDistances[i];
    snapshot.statuses[i]  = latestStatuses[i];
  }

  snapshot.accelMag = currentAccelMag;
  snapshot.peckActive = peckDetectedRecently(nowMs);
  
  return true;
}

void showStateBanner(const __FlashStringHelper* stateText) {
  if (currentMode != MODE_IDLE) return;

  OLED_SELECT();
  display.clearDisplay();
  unsigned long now = millis();
  drawStatusHeader(now);
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.println(stateText);
  display.setTextSize(1);
  display.setCursor(0, 48);
  display.print(F("Hall:"));
  bool anyHall = false;
  for (uint8_t i = 0; i < SIDE_COUNT; i++) {
    if (digitalRead(sideHallPins[i]) == LOW) {
      display.print(' ');
      display.print(i + 1);
      anyHall = true;
    }
  }
  if (!anyHall) {
    display.print(F(" none"));
  }
  display.display();
}

void displayFastSnapshot(const SensorSnapshot& snapshot) {
  if (currentMode != MODE_IDLE) return;

  OLED_SELECT();
  display.clearDisplay();
  
  // 1. Draw Header (Batt, Temp, Hum, Peck)
  drawStatusHeader(millis());

  // 2. Draw "FAST" label (Optional, maybe small)
  display.setTextSize(1);
  display.setCursor(0, 12);
  display.print(F("ACTIVE MODE"));

  // 3. Draw Button Matrix
  // Layout: 4 Columns for Panels 1, 2, 3, 4
  // Y-Start: 25
  const uint8_t yBase = 25;
  const uint8_t panelSpacing = 32; // 128px / 4 panels = 32px per panel
  
  // Draw Panel Labels (1, 2, 3, 4)
  for(uint8_t p=1; p<=4; p++) {
    uint8_t x = (p-1) * panelSpacing;
    display.setCursor(x + 10, yBase);
    display.print(F("P")); display.print(p);
  }

  // Iterate over all 16 buttons to draw their state
  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    uint8_t panel = buttonPanels[i];
    uint8_t btnNum = buttonNumbers[i]; // 1, 2, 3, 4

    // Safety check for unknown panels
    if (panel == PANEL_UNKNOWN || panel < 1 || panel > 4) continue;

    // Calculate Position
    // X: Based on Panel column + offset
    // Y: Based on Button Number (stacking them vertically)
    //    Button 1 & 2 on top row, 3 & 4 on bottom row? 
    //    Or just a vertical stack: 1, 2, 3, 4. Let's do vertical stack.
    
    uint8_t colX = (panel - 1) * panelSpacing + 12; // Center in the column
    uint8_t rowY = yBase + 10 + ((btnNum - 1) * 8); // 8px spacing per button

    // Draw the indicator
    // Radius 3 circle
    if (buttonState[i] == LOW) {
      // PRESSED: Filled Circle
      display.fillCircle(colX, rowY, 3, SSD1306_WHITE);
    } else {
      // RELEASED: Outline Circle
      display.drawCircle(colX, rowY, 3, SSD1306_WHITE);
    }
  }

  display.display();
}

void displayAccelDebug() {
  // BUG FIX: Do not draw if we are inside the menu!
  if (currentMode != MODE_IDLE) return;

  OLED_SELECT();
  display.clearDisplay();
  drawStatusHeader(millis()); // Top bar

  display.setTextSize(1);
  display.setCursor(0, 12);
  // Show Threshold vs Alpha
  display.print(F("THR:")); display.print(PECK_THRESHOLD_MARGIN_MS2, 2);
  display.print(F(" A:")); display.print(ACCEL_BASELINE_ALPHA, 2);

  // Column Headers
  display.setCursor(0, 23);  display.print(F("Ax"));
  display.setCursor(15, 23); display.print(F("Base"));
  display.setCursor(55, 23); display.print(F("Live"));
  display.setCursor(95, 23); display.print(F("Peak"));

  // Row X
  uint8_t y = 33;
  display.setCursor(0, y); display.print(F("X"));
  display.setCursor(15, y); display.print(accelBaselineX, 1);
  display.setCursor(55, y); display.print(vizRawX, 1);
  display.setCursor(95, y); display.print(vizPeakX, 2);

  // Row Y
  y += 10;
  display.setCursor(0, y); display.print(F("Y"));
  display.setCursor(15, y); display.print(accelBaselineY, 1);
  display.setCursor(55, y); display.print(vizRawY, 1);
  display.setCursor(95, y); display.print(vizPeakY, 2);

  // Row Z
  y += 10;
  display.setCursor(0, y); display.print(F("Z"));
  display.setCursor(15, y); display.print(accelBaselineZ, 1);
  display.setCursor(55, y); display.print(vizRawZ, 1);
  display.setCursor(95, y); display.print(vizPeakZ, 2);

  display.display();
}

void displaySensorSnapshot(const SensorSnapshot& snapshot) {
  if (currentMode != MODE_IDLE) return;

  OLED_SELECT();
  display.clearDisplay();
  drawStatusHeader(millis()); 

  display.setTextSize(1);
  for (uint8_t i = 0; i < SENSOR_COUNT; i += 2) {
    uint8_t y = 12 + (i / 2) * 9; 
    
    // Left Column
    display.setCursor(0, y);
    display.print(F("S")); display.print(i + 1); display.print(F(":"));
    if (snapshot.statuses[i] == 0) {
       display.print(snapshot.distances[i]);
    } else {
       display.print(F("E")); display.print(snapshot.statuses[i]); // Show Error Code
    }
    
    // Right Column
    display.setCursor(64, y);
    display.print(F("S")); display.print(i + 2); display.print(F(":"));
    if (snapshot.statuses[i + 1] == 0) {
       display.print(snapshot.distances[i + 1]);
    } else {
       display.print(F("E")); display.print(snapshot.statuses[i + 1]); // Show Error Code
    }
  }
  display.display();
}


void drawPeckIndicator(unsigned long nowMs) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.fillRect(118, 0, 10, 10, SSD1306_BLACK);

  if (accelReady && peckDetectedRecently(nowMs)) {
    const uint8_t x = 120;
    const uint8_t y = 1;
    display.fillTriangle(x, y + 6, x + 6, y + 6, x + 3, y, SSD1306_WHITE);
  }
}

// --- VISUALIZATION HELPERS ---

void drawStatusHeader(unsigned long nowMs) {
  // 1. Clear the top bar
  display.fillRect(0, 0, SCREEN_WIDTH, 10, SSD1306_BLACK);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // --- BATTERY (Left Side) ---
  float voltage = readBatteryVoltage();
  int percent = batteryPercentFromVoltage(voltage);
  bool chargerConnected = isChargerConnected();

  // Draw Battery Icon
  display.drawRect(0, 0, 14, 8, SSD1306_WHITE);
  display.fillRect(14, 2, 2, 4, SSD1306_WHITE); // Positive nub

  // Fill Battery Icon
  int cappedPercent = constrain(percent, 0, 100);
  uint8_t fillW = (uint8_t)(10.0f * cappedPercent / 100.0f); // 12px internal width
  if (fillW > 0 && (!chargerConnected || ((nowMs / 500) % 2 == 0))) {
    display.fillRect(2, 2, fillW, 4, SSD1306_WHITE);
  }

  // Draw Battery Text (Tight layout)
  display.setCursor(19, 0);
  display.print(percent);
  display.print(F("%"));
  // Only show voltage if we have room (optional, helps fit temp/hum)
  // display.print(voltage, 1); 

  // --- TEMP & HUMIDITY (Right of Battery) ---
  // Positioned at x=55 to center-right
  display.setCursor(55, 0);
  if (!isnan(lastTempC)) {
    display.print((int)lastTempC);
    display.print(F("C "));
  }
  if (!isnan(lastHumidity)) {
    display.print((int)lastHumidity);
    display.print(F("%"));
  }

  // --- PECK INDICATOR (Far Right) ---
  // Drawn at x=118
  if (accelReady && peckDetectedRecently(nowMs)) {
    // Draw a "Play" triangle icon to indicate activity
    display.fillTriangle(120, 1, 120, 7, 126, 4, SSD1306_WHITE);
  }
  
  // Draw a separator line
  display.drawLine(0, 9, SCREEN_WIDTH, 9, SSD1306_WHITE);
}

void displayMenuMessage(const __FlashStringHelper* line1,
                        const __FlashStringHelper* line2 = nullptr,
                        const __FlashStringHelper* line3 = nullptr) {
  OLED_SELECT();
  display.clearDisplay();
  unsigned long now = millis();
  drawStatusHeader(now);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  if (line1) display.println(line1);
  if (line2) display.println(line2);
  if (line3) display.println(line3);
  display.display();
}

void showMenuSelectSide() {
  OLED_SELECT();
  display.clearDisplay();
  unsigned long now = millis();
  drawStatusHeader(now);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println(F("Menu: select side"));
  display.println(F("1=Side1 2=Side2"));
  display.println(F("3=Side3"));
  display.println(F("4=Side4"));

  display.display();
}

void showMenuSelectLength() {
  OLED_SELECT();
  display.clearDisplay();
  unsigned long now = millis();
  drawStatusHeader(now);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.print(F("Side "));
  display.print(menuSelectedSide);
  display.println(F(" length"));
  display.print(F("Len: "));
  display.println(menuSequenceTargetLength);
  display.println(F("1=- 2=+"));
  display.println(F("3=Select"));
  display.println(F("4=Back"));
  display.display();
}

void showMenuEnterSequence() {
  OLED_SELECT();
  display.clearDisplay();
  unsigned long now = millis();
  drawStatusHeader(now);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.print(F("Side "));
  display.print(menuSelectedSide);
  display.println(F(" sequence"));
  display.print(F("Enter "));
  display.print(menuSequenceTargetLength);
  display.println(F(" buttons"));

  display.setTextSize(2);
  display.setCursor(0, 34);
  for (uint8_t i = 0; i < menuSequenceTargetLength; i++) {
    if (i < menuSequenceLength) {
      display.print(menuSequenceBuffer[i]);
    } else {
      display.print('_');
    }
    if (i < menuSequenceTargetLength - 1) display.print(' ');
  }

  display.setTextSize(1);
  display.setCursor(0, 54);
  display.print(F("Step "));
  display.print(menuSequenceLength);
  display.print(F("/"));
  display.print(menuSequenceTargetLength);
  display.display();
}

void showMenuConfirm() {
  OLED_SELECT();
  display.clearDisplay();
  unsigned long now = millis();
  drawStatusHeader(now);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.print(F("Side "));
  display.print(menuSelectedSide);
  display.println(F(" sequence"));

  display.setTextSize(2);
  display.setCursor(0, 34);
  for (uint8_t i = 0; i < menuSequenceTargetLength; i++) {
    display.print(menuSequenceBuffer[i]);
    if (i < menuSequenceTargetLength - 1) display.print(' ');
  }

  display.setTextSize(1);
  display.setCursor(0, 50);
  display.println(F("1=Save 2=Again"));
  display.setCursor(0, 58);
  display.println(F("3=Menu 4=Back"));
  display.display();
}

void showMenuMoreOptions() {
  OLED_SELECT();
  display.clearDisplay();
  unsigned long now = millis();
  drawStatusHeader(now);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println(F("Options"));
  display.println(F("1=Factory reset"));
  display.println(F("2=Exit menu"));
  display.print(F("3=Uno R4: "));
  display.println(expectUnoR4 ? F("On") : F("Off"));
  display.println(F("4=Back"));
  display.display();
}


void showMenuResetConfirm() {
  displayMenuMessage(F("Reset sequences?"),
                     F("1=Yes 2=No"),
                     F("Restore defaults"));
}

void enterMenu() {
  currentMode = MODE_MENU_MAIN; // Changed from MODE_MENU_SELECT_SIDE
  menuAwaitingRelease = true;
  menuSelectedSide = 0;
  menuSequenceTargetLength = DEFAULT_SEQUENCE_LENGTH;
  resetMenuSequenceBuffer();
  panel1MenuHoldActive = false;
  menuHoldStart = 0;
  displayMenuMessage(F("Menu mode"),
                     F("Release panel 1"),
                     F("buttons to begin"));
}

void exitMenu() {
  currentMode = MODE_IDLE;
  menuAwaitingRelease = false;
  menuSelectedSide = 0;
  panel1MenuHoldActive = false;
  menuHoldStart = 0;
  menuSequenceTargetLength = DEFAULT_SEQUENCE_LENGTH;
  resetMenuSequenceBuffer();
}

void resetToFactoryDefaults() {
  applyDefaultSequences();
  saveSequencesToEEPROM();
  displayMenuMessage(F("Factory reset"),
                     F("Defaults restored"),
                     F("Returning..."));
  delay(1000);
  exitMenu();
}

void saveMenuSequence() {
  if (menuSelectedSide < 1 || menuSelectedSide > SIDE_COUNT) return;
  uint8_t idx = menuSelectedSide - 1;
  uint8_t lengthToSave = menuSequenceTargetLength;
  if (lengthToSave < MIN_SEQUENCE_LENGTH || lengthToSave > MAX_SEQUENCE_LENGTH) {
    lengthToSave = DEFAULT_SEQUENCE_LENGTH;
  }
  storedSequenceLengths[idx] = lengthToSave;
  for (uint8_t i = 0; i < lengthToSave; i++) {
    storedSequences[idx][i] = menuSequenceBuffer[i];
  }
  for (uint8_t i = lengthToSave; i < MAX_SEQUENCE_LENGTH; i++) {
    storedSequences[idx][i] = 0;
  }
  saveSequencesToEEPROM();
  sequenceProgress[idx] = 0;
  sequenceLastInput[idx] = 0;
  displayMenuMessage(F("Sequence saved"),
                     F("Returning to"),
                     F("sensor view"));
  delay(800);
  exitMenu();
}

void deliverRewardForSide(uint8_t side) {
  if (side < 1 || side > SIDE_COUNT) return;
  if (!rotateTunnelToSide(side)) {
    return;
  }

  // 1. Energize Tunnel Motor (Hold position)
  digitalWrite(EN_PIN, LOW); 

  foodStepper.enableOutputs();

  // --- PHASE 1: THE JACKHAMMER (Vibration) ---
  // We use extreme acceleration to create mechanical shock.
  // This shakes the pellets to break "bridges" (air gaps) in the hopper.
  foodStepper.setMaxSpeed(2000);      
  foodStepper.setAcceleration(50000); // Extreme acceleration = Impact force

  // Rapidly oscillate 10 times
  for(int i=0; i<10; i++) {
    // Jolt Forward
    foodStepper.move(90); 
    while (foodStepper.distanceToGo() != 0) foodStepper.run();
    
    // Jolt Backward
    foodStepper.move(-90);
    while (foodStepper.distanceToGo() != 0) foodStepper.run();
  }

  // --- PHASE 2: THE UN-WEDGE (Relief) ---
  // Back up significantly to pull the jammed pellet away from the wall.
  foodStepper.setMaxSpeed(800); 
  foodStepper.setAcceleration(1000);
  foodStepper.move(-STEPS_45_FOOD*2); // Back up ~45 degrees (adjust if needed)
  while (foodStepper.distanceToGo() != 0) foodStepper.run();
  
  delay(100); // Brief pause to let gravity drop the pellet

  // --- PHASE 3: THE BULLDOZER (Extrusion) ---
   foodStepper.setMaxSpeed(150);      // Very slow
  foodStepper.setAcceleration(500);  // Smooth ramp up
 
  int dispenseAmount = STEPS_45_FOOD * 4; 
  foodStepper.move(STEPS_45_FOOD + dispenseAmount); 
  while (foodStepper.distanceToGo() != 0) {
    foodStepper.run();
    // Safety: If you want to keep buttons alive during this slow phase, uncomment below:
    // if (buttonScanPending) { scanButtons(millis()); buttonScanPending = 0; }
  }
  // Back up significantly to pull the jammed pellet away from the wall.
  foodStepper.setMaxSpeed(800); 
  foodStepper.setAcceleration(1000);
  foodStepper.move(-STEPS_45_FOOD*4); // Back up ~45 degrees (adjust if needed)
  while (foodStepper.distanceToGo() != 0) foodStepper.run();

  // Slow and steady. Low speed = Maximum Torque.
  foodStepper.setMaxSpeed(350);      // Very slow
  foodStepper.setAcceleration(500);  // Smooth ramp up
  
  // Move forward enough to dispense (Back + Dispense Amount)
  // We backed up 1x STEPS_45, so we need to go forward 1x just to return to start,
  // plus whatever amount you actually want to dispense.
  dispenseAmount = STEPS_45_FOOD * 4; 
  foodStepper.move(STEPS_45_FOOD + dispenseAmount); 
  
  while (foodStepper.distanceToGo() != 0) {
    foodStepper.run();
    // Safety: If you want to keep buttons alive during this slow phase, uncomment below:
    // if (buttonScanPending) { scanButtons(millis()); buttonScanPending = 0; }
  }

  // --- PHASE 4: ANTI-DRIP ---
  // Small retract to stop food from falling out later
  foodStepper.setSpeed(400);
  foodStepper.move(-STEPS_DELOCK_FOOD);
  while (foodStepper.distanceToGo() != 0) foodStepper.run();

  // Cleanup
  foodStepper.disableOutputs(); 
  
  // Restore Tunnel
  delay(1000); 
  digitalWrite(EN_PIN, HIGH); // Release tunnel motor

  foodDeliveredSinceLastLog = true;
  lastMoveMs = millis();
}

void processSequenceInput(uint8_t panel, uint8_t buttonNumber, unsigned long now) {
  if (panel == 0 || panel > SIDE_COUNT) return;
  uint8_t idx = panel - 1;
  uint8_t expectedLength = storedSequenceLengths[idx];
  if (expectedLength == 0) return;

  if (sequenceProgress[idx] > 0 && sequenceLastInput[idx] != 0 &&
      now - sequenceLastInput[idx] > SEQUENCE_TIMEOUT_MS) {
    sequenceProgress[idx] = 0;
  }

  if (sequenceProgress[idx] < expectedLength &&
      buttonNumber == storedSequences[idx][sequenceProgress[idx]]) {
    sequenceProgress[idx]++;
  } else if (buttonNumber == storedSequences[idx][0]) {
    sequenceProgress[idx] = 1;
  } else {
    sequenceProgress[idx] = 0;
  }

  sequenceLastInput[idx] = now;

  if (sequenceProgress[idx] >= expectedLength) {
    if (now - lastMoveMs > moveCooldownMs) {
      buzzTest(BUZZER_REWARD_FREQ_HZ, BUZZER_REWARD_DURATION_MS);
      deliverRewardForSide(panel);
    }
    sequenceProgress[idx] = 0;
  }
}

void updateSequenceTimeouts(unsigned long now) {
  for (uint8_t i = 0; i < SIDE_COUNT; i++) {
    if (sequenceProgress[i] > 0 && sequenceLastInput[i] != 0 &&
        now - sequenceLastInput[i] > SEQUENCE_TIMEOUT_MS) {
      sequenceProgress[i] = 0;
    }
  }
}

void handleMenuActivationHold(unsigned long now) {
  if (currentMode != MODE_IDLE) {
    panel1MenuHoldActive = false;
    menuHoldStart = 0;
    return;
  }

  if (allPanel1Pressed()) {
    if (!panel1MenuHoldActive) {
      panel1MenuHoldActive = true;
      menuHoldStart = now;
      sequenceProgress[0] = 0;
      sequenceLastInput[0] = 0;
    } else if (now - menuHoldStart >= MENU_HOLD_MS) {
      enterMenu();
    }
  } else {
    panel1MenuHoldActive = false;
    menuHoldStart = 0;
  }
}
void showMenuMain() {
  buzzTest(BUZZER_REWARD_FREQ_HZ, BUZZER_REWARD_DURATION_MS);
  OLED_SELECT();
  display.clearDisplay();
  drawStatusHeader(millis());
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.println(F("MAIN MENU"));
  display.println(F("1. Change Sequences"));
  display.println(F("2. Change Display"));
  display.println(F("3. Options"));
  display.println(F("4. Exit Menu"));
  display.display();
}
void showMenuDisplaySelect() {
  OLED_SELECT();
  display.clearDisplay();
  drawStatusHeader(millis());
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.println(F("DISPLAY MODE"));
  
  display.setCursor(0, 22);
  if(currentDisplayMode == DISPLAY_MODE_BUTTONS) display.print(F(">"));
  display.println(F("1. Buttons"));
  
  display.setCursor(0, 32);
  if(currentDisplayMode == DISPLAY_MODE_SENSORS) display.print(F(">"));
  display.println(F("2. Sensors"));

  // NEW OPTION
  display.setCursor(0, 42);
  if(currentDisplayMode == DISPLAY_MODE_ACCEL) display.print(F(">"));
  display.println(F("3. Accel Tune"));
  
  display.setCursor(0, 56);
  display.println(F("4. Back"));
  display.display();
}
void showMenuOptions() {
  OLED_SELECT();
  display.clearDisplay();
  drawStatusHeader(millis());
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.println(F("OPTIONS"));
  
  display.println(F("1. Factory Reset"));
  // Spacer
  display.print(F("2. Uno R4: "));
  display.println(expectUnoR4 ? F("ON") : F("OFF"));
  
  display.setCursor(0, 55);
  display.println(F("4. Back"));
  display.display();
}

void handleButtonPress(uint8_t index, unsigned long now) {
  uint8_t panel = buttonPanels[index];
  if (panel == PANEL_UNKNOWN) return;
  uint8_t number = buttonNumbers[index];

  // --- FAST MODE TRIGGER ---
  // Any button press wakes the system into fast mode
  snprintf(lastButtonEvent, sizeof(lastButtonEvent), "P%uB%u", panel, number);
  buttonEventPending = true;
  if (samplingState != STATE_FAST_SAMPLING && samplingState != STATE_BASELINING) {
    samplingState = STATE_FAST_SAMPLING;
    setSensorIntervalMs(SENSOR_INTERVAL_FAST_MS);
    stableSinceMs = 0;
    lastChangeDuringFastMs = now;
    showStateBanner(F("WAKE UP")); // Brief banner
  }

  // --- IDLE / SEQUENCE ENTRY ---
  if (currentMode == MODE_IDLE) {
    if (panel == 1 && panel1MenuHoldActive) return; // Ignore if holding for menu
    processSequenceInput(panel, number, now);
    return;
  }

  // --- MENU NAVIGATION ---
  // Only Panel 1 controls the menu
  if (panel != 1) return;
  if (menuAwaitingRelease) return;

  switch (currentMode) {
    
    // 1. MAIN MENU
    case MODE_MENU_MAIN:
      if (number == 1) {
        currentMode = MODE_MENU_SELECT_SIDE;
        showMenuSelectSide();
      } else if (number == 2) {
        currentMode = MODE_MENU_DISPLAY_SELECT;
        showMenuDisplaySelect();
      } else if (number == 3) {
        currentMode = MODE_MENU_OPTIONS;
        showMenuOptions();
      } else if (number == 4) {
        exitMenu();
      }
      break;

    // 1.1 SELECT SIDE (For Sequences)
    case MODE_MENU_SELECT_SIDE:
      if (number >= 1 && number <= SIDE_COUNT) {
        menuSelectedSide = number;
        resetMenuSequenceBuffer();
        uint8_t idx = menuSelectedSide - 1;
        menuSequenceTargetLength = storedSequenceLengths[idx];
        if (menuSequenceTargetLength < MIN_SEQUENCE_LENGTH ||
            menuSequenceTargetLength > MAX_SEQUENCE_LENGTH) {
          menuSequenceTargetLength = DEFAULT_SEQUENCE_LENGTH;
        }
        currentMode = MODE_MENU_SELECT_LENGTH;
        showMenuSelectLength();
      }
      // Note: No "Back" button defined here because buttons 1-4 are taken by sides.
      // You could check for a long hold to go back, but for now we follow the simple logic.
      break;

    case MODE_MENU_SELECT_LENGTH:
      if (number == 1) {
        if (menuSequenceTargetLength > MIN_SEQUENCE_LENGTH) {
          menuSequenceTargetLength--;
          showMenuSelectLength();
        }
      } else if (number == 2) {
        if (menuSequenceTargetLength < MAX_SEQUENCE_LENGTH) {
          menuSequenceTargetLength++;
          showMenuSelectLength();
        }
      } else if (number == 3) {
        resetMenuSequenceBuffer();
        currentMode = MODE_MENU_ENTER_SEQUENCE;
        showMenuEnterSequence();
      } else if (number == 4) {
        currentMode = MODE_MENU_SELECT_SIDE;
        menuSelectedSide = 0;
        menuSequenceTargetLength = DEFAULT_SEQUENCE_LENGTH;
        resetMenuSequenceBuffer();
        showMenuSelectSide();
      }
      break;

    // 1.1.1 ENTER SEQUENCE
    case MODE_MENU_ENTER_SEQUENCE:
      if (menuSequenceLength < menuSequenceTargetLength) {
        menuSequenceBuffer[menuSequenceLength++] = number;
        showMenuEnterSequence();
        if (menuSequenceLength >= menuSequenceTargetLength) {
          currentMode = MODE_MENU_CONFIRM;
          showMenuConfirm();
        }
      }
      break;

    // 1.1.2 CONFIRM SEQUENCE
    case MODE_MENU_CONFIRM:
      if (number == 1) {
        saveMenuSequence(); // Saves and exits
      } else if (number == 2) {
        resetMenuSequenceBuffer();
        currentMode = MODE_MENU_ENTER_SEQUENCE;
        showMenuEnterSequence(); // Retry
      } else if (number == 3) {
        // Return to Main Menu
        currentMode = MODE_MENU_MAIN;
        showMenuMain();
      } else if (number == 4) {
        // Return to Side Selection
        currentMode = MODE_MENU_SELECT_SIDE;
        showMenuSelectSide();
      }
      break;

    // 2. DISPLAY SELECT
    case MODE_MENU_DISPLAY_SELECT:
      if (number == 1) {
        currentDisplayMode = DISPLAY_MODE_BUTTONS;
        saveSequencesToEEPROM();
        showMenuDisplaySelect();
      } else if (number == 2) {
        currentDisplayMode = DISPLAY_MODE_SENSORS;
        saveSequencesToEEPROM();
        showMenuDisplaySelect();
      } else if (number == 3) {
        // NEW OPTION
        currentDisplayMode = DISPLAY_MODE_ACCEL;
        saveSequencesToEEPROM();
        showMenuDisplaySelect();
      } else if (number == 4) {
        currentMode = MODE_MENU_MAIN;
        showMenuMain();
      }
      break;

    // 3. OPTIONS
    case MODE_MENU_OPTIONS:
      if (number == 1) {
        currentMode = MODE_MENU_RESET_CONFIRM;
        showMenuResetConfirm();
      } else if (number == 2) {
        expectUnoR4 = !expectUnoR4;
        saveSequencesToEEPROM(); // Save preference
        showMenuOptions();       // Update text
      } else if (number == 4) {
        currentMode = MODE_MENU_MAIN;
        showMenuMain();
      }
      break;

    // 3.1 RESET CONFIRM
    case MODE_MENU_RESET_CONFIRM:
      if (number == 1) {
        resetToFactoryDefaults(); // Resets and exits
      } else {
        // Any other button cancels back to Options
        currentMode = MODE_MENU_OPTIONS;
        showMenuOptions();
      }
      break;

    default:
      break;
  }
}


void readAndSend() {
  SensorSnapshot snapshot;
  if (!captureSensorSnapshot(snapshot)) {
    return;
  }

  unsigned long nowMs = millis();
  const char* buttonLabel = buttonEventPending ? lastButtonEvent : "";

  switch (samplingState) {
    case STATE_BASELINING:
      recordBaselineSample(snapshot);
      if (baselineStartMs == 0) baselineStartMs = nowMs;
      if (nowMs - baselineStartMs >= BASELINE_DURATION_MS) {
        finalizeBaselineFromAccumulator();
        samplingState = STATE_SLEEPING;
        setSensorIntervalMs(SENSOR_INTERVAL_SLEEP_MS);
        stableSinceMs = 0;
        showStateBanner(F("Sleep"));
      }
      break;

    case STATE_SLEEPING: {
      if (!sessionBaseline.valid) {
        samplingState = STATE_BASELINING;
        resetBaselineAccumulator();
        baselineStartMs = nowMs;
        setSensorIntervalMs(SENSOR_INTERVAL_BASELINE_MS);
        showStateBanner(F("Baseline"));
        break;
      }

      bool outsideBaseline = !snapshotWithinBaseline(snapshot);
      if (outsideBaseline || snapshot.peckActive || buttonEventPending) {
        samplingState = STATE_FAST_SAMPLING;
        setSensorIntervalMs(SENSOR_INTERVAL_FAST_MS);
        fastModeStartMs = nowMs;
        stableSinceMs = 0;
        lastChangeDuringFastMs = nowMs;
        
        displayFastSnapshot(snapshot);
        
        // FIX: Only send to R4 if we are NOT in the menu
        if (currentMode == MODE_IDLE) {
          sendDistancesFramed(snapshot.distances);
        }
        
        appendLogEntry(snapshot.timestamp, snapshot.distances, snapshot.statuses, snapshot.peckActive, foodDeliveredSinceLastLog, "FAST", buttonLabel);
        foodDeliveredSinceLastLog = false;
        buttonEventPending = false;
        lastSnapshot = snapshot;
      } else {
        showStateBanner(F("Sleep"));
        lastSnapshot = snapshot;
      }
      break;
    }

    case STATE_FAST_SAMPLING: {
      if (currentDisplayMode == DISPLAY_MODE_SENSORS) {
          displaySensorSnapshot(snapshot);
      } else if (currentDisplayMode == DISPLAY_MODE_ACCEL) {
          displayAccelDebug(); // <--- NEW CALL
      } else {
          displayFastSnapshot(snapshot); // Default Buttons
      }

      // FIX: Only send to R4 if we are NOT in the menu
      if (currentMode == MODE_IDLE) {
        sendDistancesFramed(snapshot.distances);
      }

      appendLogEntry(snapshot.timestamp, snapshot.distances, snapshot.statuses, snapshot.peckActive, foodDeliveredSinceLastLog, "FAST", buttonLabel);
      foodDeliveredSinceLastLog = false;
      buttonEventPending = false;

      bool withinBaseline = snapshotWithinBaseline(snapshot);
      if (withinBaseline) {
        if (stableSinceMs == 0) stableSinceMs = nowMs;
        if (nowMs - stableSinceMs >= FAST_STABLE_HOLD_MS) {
          samplingState = STATE_SLEEPING;
          setSensorIntervalMs(SENSOR_INTERVAL_SLEEP_MS);
          stableSinceMs = 0;
          showStateBanner(F("Sleep"));
        }
      } else {
        stableSinceMs = 0;
      }

      if (snapshotChangedSince(snapshot, lastSnapshot)) {
        lastChangeDuringFastMs = nowMs;
      }

      if (nowMs - lastChangeDuringFastMs >= BASELINE_REFRESH_MS) {
        applySnapshotAsBaseline(snapshot);
        samplingState = STATE_SLEEPING;
        setSensorIntervalMs(SENSOR_INTERVAL_SLEEP_MS);
        stableSinceMs = 0;
        showStateBanner(F("New base"));
      }

      lastSnapshot = snapshot;
      break;
    }
  }
}

// --- SHUTDOWN SEQUENCE ---
void drawSleepingBird(int x,int y){
  OLED_SELECT();
  display.drawBitmap(x, y, birdBitmap, BIRD_W, BIRD_H, SSD1306_WHITE);
}

void shutdownSequence(){
  appendShutdownNotice();
  showShutdownBatteryWarnings();

  // 1) Say good night
  OLED_SELECT();
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(12, 10);
  display.println(F("Good"));
  display.setCursor(12, 32);
  display.println(F("night"));
  display.display();
  delay(1200); // show it briefly
  display.clearDisplay();
  drawSleepingBird(32, 0);
  display.display();
  delay(1200); // show it briefly
  playShutdownMelody();

  // 2) Quiet sensors (optional)
  for (uint8_t ch=0; ch<SENSOR_CHANNELS; ch++) resetChannelGroup(ch);

  // 3) Park tunnel between sides to sit midway on power-down
  parkTunnelBetweenSides();

  // 4) Flush any SD writes to avoid corruption
  flushSdCard();

  // 5) Cut 5V rail to peripherals
  digitalWrite(GATE_5V_PIN, LOW);
  delay(100);

  // 6) Release power hold – system powers off
  digitalWrite(POWER_HOLD_PIN, LOW);

  // If hardware doesn't cut immediately, wait here
  while(1){}
}

void forcePrimingRead() {
  tcaSelect(SCREEN_CHANNEL);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Priming Sensors..."));
  display.display();

  for (uint8_t ch = 0; ch < SENSOR_CHANNELS; ch++) {
    tcaSelect(ch);
    delay(5);
    for (uint8_t i = 0; i < SENSORS_PER_CHANNEL; i++) {
      uint8_t idx = ch * SENSORS_PER_CHANNEL + i;
      
      // Force a blocking read to ensure we have data
      // We accept that this halts the system for ~100ms per sensor
      // but it only happens once at boot.
      sensors[idx].read(); 
      latestDistances[idx] = sensors[idx].ranging_data.range_mm;
      latestStatuses[idx]  = sensors[idx].ranging_data.range_status;
    }
  }
}



// -------------------- SETUP/LOOP --------------------
void setup() {
  // Serial.begin(SERIAL_BAUD);

  Wire.begin();
  Wire.setClock(100000); // keep it conservative and rock solid

  // Power/Control pins
  pinMode(POWER_HOLD_PIN, OUTPUT);
  pinMode(GATE_5V_PIN,    OUTPUT);
  pinMode(BUZZER_PIN,     OUTPUT);
  digitalWrite(POWER_HOLD_PIN, HIGH);   // hold power after boot
  digitalWrite(GATE_5V_PIN,    HIGH);   // enable 5V rail for peripherals
  digitalWrite(BUZZER_PIN,     LOW);    // idle buzzer

  pinMode(BATTERY_PIN, INPUT);
#if defined(analogReadResolution)
  analogReadResolution(ADC_RESOLUTION);
#endif
  analogRead(BATTERY_PIN); // prime ADC for battery readings

  // Bring up rails BEFORE touching OLED
  pinMode(CHARGER_DETECT_PIN, INPUT_PULLUP); // pulled LOW when charger jack is removed
  pinMode(SWITCH_DETECT_PIN,  INPUT_PULLUP);// LOW when switch ON (change to INPUT if externally driven)

  Timer1.initialize(BUTTON_SCAN_INTERVAL_US); // 1ms button scan intervals
  Timer1.attachInterrupt(timerISR);

  // STEP/DIR outputs
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  digitalWrite(STEP_PIN2, LOW);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); // disable driver at start
  pinMode(EN_PIN2, OUTPUT);
  digitalWrite(EN_PIN2, HIGH); // disable food driver at start

  tunnelStepper.setEnablePin(EN_PIN);
  tunnelStepper.setPinsInverted(false, false, true); // enable is active LOW
  tunnelStepper.setMaxSpeed(TUNNEL_MAX_SPEED_STEPS_S);
  tunnelStepper.setAcceleration(TUNNEL_ACCEL_STEPS_S2);
  tunnelStepper.disableOutputs();

  foodStepper.setEnablePin(EN_PIN2);
  foodStepper.setPinsInverted(false, false, true); // enable is active LOW
  foodStepper.setMaxSpeed(FOOD_MAX_SPEED_STEPS_S);
  foodStepper.setAcceleration(FOOD_ACCEL_STEPS_S2);
  foodStepper.disableOutputs();

  for (uint8_t i = 0; i < SIDE_COUNT; i++) {
    pinMode(sideHallPins[i], INPUT_PULLUP);
  }

  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  initButtonPortMetadata();

  loadSequencesFromEEPROM();
  currentTunnelSide = 1;
  resetMenuSequenceBuffer();

  // Now init OLED on its TCA channel

  tcaSelect(SCREEN_CHANNEL);
  delay(50);  // let the mux settle
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    // Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  delay(200);


  // OLED
  tcaSelect(SCREEN_CHANNEL);
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("Hello bird..."));
  display.display();
  playStartupMelody();
  delay(3000);

  showStartupBatteryWarnings();

  // RTC
  RTC_SELECT();
  rtc.begin(); // assume time is already set

  // Temperature/humidity and peck detection sensors share the RTC/OLED channel
  RTC_SELECT();
  ahtReady = aht20.begin();
  tcaSelect(SCREEN_CHANNEL);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print(F("AHT20: "));
  display.println(ahtReady ? F("OK") : F("FAIL"));
  display.display();
  delay(500);

  RTC_SELECT();
  accelReady = accel.begin();
  if (accelReady) {
    // Y axis up, X axis sideways; use most sensitive ±2G range and high-res mode for light pecks
    accel.setRange(LSM303_RANGE_2G);
    accel.setMode(LSM303_MODE_HIGH_RESOLUTION);
    sensors_event_t event;
    if (accel.getEvent(&event)) {
      accelBaselineX = event.acceleration.x;
      accelBaselineY = event.acceleration.y;
      accelBaselineZ = event.acceleration.z;
    }
  }
  tcaSelect(SCREEN_CHANNEL);
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(F("LSM303: "));
  display.println(accelReady ? F("OK") : F("FAIL"));
  display.display();
  delay(500);

  // SD card (SPI)
  pinMode(SD_CHIP_SELECT_PIN, OUTPUT);
  if (SD.begin(SD_CHIP_SELECT_PIN)) {
    sdAvailable = true;
    tcaSelect(SCREEN_CHANNEL);
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("SD card OK"));
    display.display();
    delay(500);
  } else {
    sdAvailable = false;
    tcaSelect(SCREEN_CHANNEL);
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("SD init FAIL"));
    display.display();
    delay(1000);
  }

  RTC_SELECT();
  appendSessionHeader(rtc.now());

  // Put all sensors on all channels into reset first
  for (uint8_t ch = 0; ch < SENSOR_CHANNELS; ch++) {
    resetChannelGroup(ch);
  }
  delay(100);

  // Bring-up per channel, assigning 0x30/0x31/0x32 on each
  for (uint8_t ch = 0; ch < SENSOR_CHANNELS; ch++) {
    bool ok = true;
    for (uint8_t i = 0; i < SENSORS_PER_CHANNEL; i++) {
      if (!initSensorOn(ch, i)) { ok = false; }
    }

    // brief status on OLED
    tcaSelect(SCREEN_CHANNEL);
    display.clearDisplay();
    display.setCursor(0,0);
    display.print(F("CH ")); display.print(ch+1); display.println(ok ? F(": OK") : F(": FAIL"));
    display.display();
    delay(500);
  }
   

  // Uno R4 on channel 4 with 10s timeout
  bool unoR4Ok = false;
  if (expectUnoR4) {
    unsigned long startAttempt = millis();
    while (millis() - startAttempt < 10000 && !unoR4Ok) {
      unoR4Ok = connectUnoR4();
      if (!unoR4Ok) delay(500);
    }
  }
  tcaSelect(SCREEN_CHANNEL);
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(F("UNO R4: "));
  if (expectUnoR4) {
    display.println(unoR4Ok ? F("OK") : F("FAIL"));
  } else {
    display.println(F("SKIP"));
  }
  display.display();
  delay(500);
   
  // Ready banner
  tcaSelect(SCREEN_CHANNEL);
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("birdBox GO"));
  display.display();
  delay(1000);

  forcePrimingRead(); 

  samplingState = STATE_BASELINING;
  resetBaselineAccumulator();
  baselineStartMs = millis();
  setSensorIntervalMs(SENSOR_INTERVAL_BASELINE_MS);
  showStateBanner(F("Baseline"));

  // Record the completion of the boot sequence so the shutdown debounce
  // ignores any start-up noise on the switch sense line.
  startupMillis = millis();
}



void loop() {
  unsigned long now = millis();

  // ---------------------------------------------------------
  // 1. CHECK SENSOR TIMER (Atomic)
  // ---------------------------------------------------------
  bool performSensorUpdate = false;

  noInterrupts();
  if (sensorUpdateFlag) {
    sensorUpdateFlag = false;
    performSensorUpdate = true;
  }
  interrupts();

  // ---------------------------------------------------------
  // 2. PROCESS PENDING BUTTON SCANS (The Optimized Block)
  // ---------------------------------------------------------
  // Take a snapshot of pending buttons so we don't toggle interrupts constantly
  uint16_t snapshotPending = 0;
  
  noInterrupts();
  snapshotPending = buttonPendingMask;
  interrupts();

  if (snapshotPending) {
    for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
      uint16_t bit = (uint16_t)1 << i;

      // Only waste CPU cycles checking buttons that actually changed
      if (snapshotPending & bit) {
        
        unsigned long lastChange;
        uint8_t rawState;

        // Retrieve the ISR data atomically
        noInterrupts();
        lastChange = buttonLastChange[i];
        rawState = buttonRawState[i];
        interrupts();

        // Debounce Check
        if ((now - lastChange) >= BUTTON_DEBOUNCE_MS) {
          
          // Clear the pending bit safely
          noInterrupts();
          buttonPendingMask &= ~bit;
          interrupts();

          // Update the stable state
          // We don't need noInterrupts for buttonState access here 
          // because the ISR only touches buttonRawState, not buttonState.
          if (buttonState[i] != rawState) {
            buttonState[i] = rawState;
            
            // Mark that a verified event occurred
            noInterrupts();
            buttonEventMask |= bit;
            interrupts();
          }
        }
      }
    }
  }

  // ---------------------------------------------------------
  // 3. HANDLE VERIFIED BUTTON EVENTS
  // ---------------------------------------------------------
  uint16_t pendingEvents = 0;

  noInterrupts();
  pendingEvents = buttonEventMask;
  buttonEventMask = 0;
  interrupts();

  // Handle "Hold Panel 1" logic (Menu entry)
  handleMenuActivationHold(now);

  // If in menu and waiting for release
  if (currentMode != MODE_IDLE && menuAwaitingRelease && allPanel1Released()) {
    menuAwaitingRelease = false;
    showMenuMain();
  }

  updateSequenceTimeouts(now);

  // Process the actual clicks
  if (pendingEvents) {
    for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
      if (pendingEvents & ((uint16_t)1 << i)) {
        
        // We already updated buttonState[i] in step 2, so just read it
        uint8_t state = buttonState[i];

        // Edge Detection: Input Pullup means LOW is pressed
        if (prevButtonState[i] == HIGH && state == LOW) {
          handleButtonPress(i, now);
        }
        prevButtonState[i] = state;
      }
    }
  }

  // ---------------------------------------------------------
  // 4. POWER SWITCH LOGIC (Shutdown)
  // ---------------------------------------------------------
  if (!shutdownInitiated && (now - startupMillis) > SWITCH_STARTUP_GRACE_MS) {
    // If switch is HIGH, it means it is OPEN (OFF position for active-low switch)
    if (digitalRead(SWITCH_DETECT_PIN) == HIGH) {
      if (switchHighStart == 0) {
        switchHighStart = now;
      } else if (now - switchHighStart >= SWITCH_DEBOUNCE_DELAY_MS) {
        shutdownInitiated = true;
        shutdownSequence();
      }
    } else {
      switchHighStart = 0;
    }
  }

  // ---------------------------------------------------------
  // 5. SENSOR READS (Low Priority)
  // ---------------------------------------------------------
  // Only read sensors if the flag was set AND we aren't mid-shutdown
  if (performSensorUpdate && !shutdownInitiated) {
    readAndSend();
  }
}