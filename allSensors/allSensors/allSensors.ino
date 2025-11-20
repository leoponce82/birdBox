#include <Wire.h>
#include <VL53L1X.h>          // Pololu VL53L1X library
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>

// Flag to control whether the Uno R4 connection is expected/required
const bool UNO_R4_EXPECTED = true;

const unsigned long BUTTON_SCAN_INTERVAL_US = 50; // 10 ms between button scans
const uint8_t SENSOR_UPDATE_TICKS = 20;              // 20 * 10 ms = 200 ms sensor updates = 5Hz

volatile uint8_t buttonScanPending = 0;
volatile bool sensorUpdateFlag = false;
volatile uint8_t sensorUpdateTickCounter = 0;

// --- Power / control pins ---
#define POWER_HOLD_PIN     26   // output, goes HIGH after boot
#define CHARGER_DETECT_PIN 27   // input, 5V via divider when charger is present
#define SWITCH_DETECT_PIN  25   // input, reads LOW when main switch is ON
#define GATE_5V_PIN        23   // output, goes HIGH after boot
#define BATTERY_PIN        A0   // input, battery voltage via divider (R1=100k, R2=47k)

const float BATTERY_R1_OHMS   = 100000.0f;
const float BATTERY_R2_OHMS   =  47000.0f;
const float BATTERY_MIN_V     =      9.6f;
const float BATTERY_MAX_V     =     12.6f;
// ADC reference calibrated to the measured 4.028V seen at 12.6V battery
const float ADC_REF_V         =      4.028f;
const uint8_t ADC_RESOLUTION  =       10;   // enforce 10-bit reads for predictable scaling
const float ADC_MAX_READING   = (1 << ADC_RESOLUTION) - 1;


// -------------------- OLED --------------------

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3D
#define SCREEN_CHANNEL  7      // TCA9548A channel for OLED and RTC
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
#define UNO_R4_CHANNEL 4      // TCA9548A channel for external Uno R4
#define UNO_R4_ADDR    0x08   // I2C address of the Uno R4
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
const float ACCEL_BASELINE_ALPHA = 0.02f;      // slow baseline keeps small shocks visible
const float PECK_THRESHOLD_MS2 = 0.18f;        // ~0.02g delta catches very light taps
const unsigned long PECK_HOLD_MS = 700;        // quick visual reset for rapid tuning
const bool SERIAL_ACCEL_DEBUG = true;          // emit raw + filtered accel readings over Serial
const unsigned long ACCEL_DEBUG_INTERVAL_MS = 10;
const unsigned long SERIAL_BAUD = 115200;

// -------------------- VL53L1X --------------------
#define SENSOR_CHANNELS     4            // channels 0..3 used for sensors
#define SENSORS_PER_CHANNEL 3
#define SENSOR_COUNT        (SENSOR_CHANNELS * SENSORS_PER_CHANNEL) // 12
uint16_t distances[SENSOR_COUNT]; // SENSOR_COUNT == 12

// XSHUT pins for sensors 1..12 (in order). NOTE: pin 1 is TX0 on Mega; avoid Serial while using it.
const uint8_t xshutPins[SENSOR_COUNT] = {
  13,12,11, 10,9,8,   // sensors 1..6 on channels 0 & 1
   6, 5, 4,  3,2,1    // sensors 7..12 on channels 2 & 3
};

// We reuse the same 3 I2C addresses per channel; the multiplexer isolates them
const uint8_t perChannelAddr[SENSORS_PER_CHANNEL] = { 0x30, 0x31, 0x32 };

VL53L1X sensors[SENSOR_COUNT];

// --- power switch debounce state ---
const unsigned long SWITCH_DEBOUNCE_DELAY_MS = 200;
const unsigned long SWITCH_STARTUP_GRACE_MS = 1000;
unsigned long switchHighStart = 0;
unsigned long startupMillis = 0;
bool shutdownInitiated = false;

void timerISR() {
  if (buttonScanPending < 255) {
    buttonScanPending++;
  }

  sensorUpdateTickCounter++;
  if (sensorUpdateTickCounter >= SENSOR_UPDATE_TICKS) {
    sensorUpdateTickCounter = 0;
    sensorUpdateFlag = true;
  }
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
const int STEPS_90      = (STEPS_PER_REV * MICROSTEPS) / 4;       // quarter turn main motor
const int STEPS_90_FOOD = (STEPS_PER_REV_FOOD * MICROSTEPS) / 4;  // quarter turn food motor

// pulse timing (adjust for your driver/motor)
const unsigned int STEP_PULSE_US = 1000;   // high/low pulse width
const unsigned int STEP_PULSE_FOOD_US = 1000; // high/low pulse width for food motor

// --- state for motor trigger & switch edge ---
unsigned long lastMoveMs = 0;
const unsigned long moveCooldownMs = 1500;

// -------------------- SD logging --------------------
const uint16_t LOG_TRIGGER_DISTANCE_MM = 200;
const unsigned long LOG_INTERVAL_MS = 1000;    // log every second
const unsigned long LOG_DURATION_MS = 60000;   // log for 1 minute
// const uint8_t SD_CHIP_SELECT_PIN = 53;

bool sdAvailable = false;
bool loggingActive = false;
unsigned long loggingStartMillis = 0;
unsigned long lastLogWriteMillis = 0;
bool requireClearBeforeNextLog = false;
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
const uint8_t SIDE_COUNT = 4;
const uint8_t MAX_SEQUENCE_LENGTH = 8;
const uint8_t DEFAULT_SEQUENCE_LENGTH = 4;
const unsigned long SEQUENCE_TIMEOUT_MS = 5000;

uint8_t storedSequences[SIDE_COUNT][MAX_SEQUENCE_LENGTH];
uint8_t storedSequenceLengths[SIDE_COUNT];
uint8_t sequenceProgress[SIDE_COUNT];
unsigned long sequenceLastInput[SIDE_COUNT];

uint8_t currentTunnelSide = 1;


const uint8_t DEFAULT_SEQUENCE_TEMPLATE[DEFAULT_SEQUENCE_LENGTH] = {1, 2, 3, 4};

const uint32_t SEQUENCE_STORAGE_MAGIC = 0xB105EED1;
const uint8_t SEQUENCE_STORAGE_VERSION = 2;

const int SEQUENCE_STORAGE_ADDR = 0;

struct SequenceStorage {
  uint32_t magic;
  uint8_t version;
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
}

void saveSequencesToEEPROM() {
  SequenceStorage data;
  data.magic = SEQUENCE_STORAGE_MAGIC;
  data.version = SEQUENCE_STORAGE_VERSION;
  for (uint8_t side = 0; side < SIDE_COUNT; side++) {
    data.lengths[side] = storedSequenceLengths[side];
    for (uint8_t i = 0; i < MAX_SEQUENCE_LENGTH; i++) {
      data.sequences[side][i] = storedSequences[side][i];
    }
  }
  data.checksum = data.computeChecksum();

  EEPROM.put(SEQUENCE_STORAGE_ADDR, data);
}

void loadSequencesFromEEPROM() {
  SequenceStorage data;
  EEPROM.get(SEQUENCE_STORAGE_ADDR, data);

  bool valid = (data.magic == SEQUENCE_STORAGE_MAGIC);
  bool needsMigration = false;
  if (valid) {
    if (data.version == SEQUENCE_STORAGE_VERSION) {
      // up-to-date
    } else if (data.version == 1) {
      needsMigration = true;
    } else {
      valid = false;
    }
  }

  if (valid) {
    uint8_t expectedChecksum = data.computeChecksum();

    if (expectedChecksum != data.checksum) {
      valid = false;
    }
  }

  if (valid) {
    for (uint8_t side = 0; side < SIDE_COUNT && valid; side++) {
      uint8_t length = data.lengths[side];
      if (length == 0 || length > MAX_SEQUENCE_LENGTH) {
        valid = false;
        break;
      }
      for (uint8_t i = 0; i < length; i++) {
        uint8_t value = data.sequences[side][i];
        if (value < 1 || value > 4) {
          valid = false;
          break;
        }
      }
    }
  }

  if (!valid) {
    applyDefaultSequences();
    saveSequencesToEEPROM();
    return;
  }

  for (uint8_t side = 0; side < SIDE_COUNT; side++) {
    storedSequenceLengths[side] = data.lengths[side];
    for (uint8_t i = 0; i < MAX_SEQUENCE_LENGTH; i++) {
      uint8_t value = data.sequences[side][i];
      if (needsMigration) {
        if (value == 3) value = 4;
        else if (value == 4) value = 3;
      }
      storedSequences[side][i] = value;
    }
  }
  resetSequenceTracking();
  if (needsMigration) {
    saveSequencesToEEPROM();
  }

}

enum SystemMode {
  MODE_IDLE,
  MODE_MENU_SELECT_SIDE,
  MODE_MENU_ENTER_SEQUENCE,
  MODE_MENU_CONFIRM,
  MODE_MENU_MORE_OPTIONS,
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
  digitalWrite(EN_PIN, LOW); // enable driver
  digitalWrite(DIR_PIN, dirCW ? HIGH : LOW);
  delayMicroseconds(2); // DIR setup time
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_PULSE_US);
  }
  digitalWrite(EN_PIN, HIGH); // disable driver to remove holding torque
}

void motorStepFood(int steps, bool dirCW) {
  digitalWrite(EN_PIN2, LOW); // enable driver
  digitalWrite(DIR_PIN2, dirCW ? HIGH : LOW);
  delayMicroseconds(2); // DIR setup time
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN2, HIGH);
    delayMicroseconds(STEP_PULSE_FOOD_US);
    digitalWrite(STEP_PIN2, LOW);
    delayMicroseconds(STEP_PULSE_FOOD_US);
  }
  digitalWrite(EN_PIN2, HIGH); // disable driver to remove holding torque
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
  digitalWrite(EN_PIN, LOW);
  digitalWrite(DIR_PIN, dirCW ? HIGH : LOW);
  delayMicroseconds(2);
  for (unsigned long step = 0; step < maxSteps; step++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_PULSE_US);
    if (digitalRead(hallPin) == LOW) {
      aligned = true;
      break;
    }
  }
  digitalWrite(EN_PIN, HIGH);
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

  unsigned long maxSteps = (unsigned long)quarterTurns * STEPS_90 + (STEPS_90 / 2);
  bool aligned = stepMotorUntilAligned(hallPin, dirCW, maxSteps);

  if (!aligned && quarterTurns != SIDE_COUNT) {
    unsigned long fullRotationSteps = (unsigned long)SIDE_COUNT * STEPS_90 + (STEPS_90 / 2);
    aligned = stepMotorUntilAligned(hallPin, dirCW, fullRotationSteps);
  }

  if (aligned) {
    currentTunnelSide = targetSide;
  }

  return aligned;
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
  if (!UNO_R4_EXPECTED) {
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
  if (!UNO_R4_EXPECTED) {
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
  if (err != 0 && UNO_R4_EXPECTED) {
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
  if (!UNO_R4_EXPECTED) {
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
    if (err && UNO_R4_EXPECTED) {
      // show err on OLED if you want
      return false;
    }
    delayMicroseconds(200); // tiny gap
  }
  return true;
}

// Send distances in framed chunks (4 values per frame)
bool sendDistancesFramed(const uint16_t *vals) {
  if (!UNO_R4_EXPECTED) {
    return true;
  }
  const uint8_t CHUNK = 4;  // 4 values = 8 data bytes + 3 header = 11 total
  for (uint8_t off = 0; off < SENSOR_COUNT; off += CHUNK) {
    uint8_t cnt = (off + CHUNK <= SENSOR_COUNT) ? CHUNK : (SENSOR_COUNT - off);

    UNO_R4_SELECT();
    delay(3); // TCA settle

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
    if (err != 0 && UNO_R4_EXPECTED) {
      // Show exact error code to help if this ever trips again
      tcaSelect(SCREEN_CHANNEL);
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0); display.print(F("I2C send ERR=")); display.println(err); // 2=NACK addr, 3=NACK data
      display.display();
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
    file.println();
  }

  file.close();
  return true;
}

void appendLogEntry(DateTime timestamp, const uint16_t *distances, const uint8_t *statuses) {
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

  file.println();
  file.close();
}

void scanButtons() {
  unsigned long now = millis();
  uint8_t portValues[BUTTON_PORT_COUNT];

  for (uint8_t group = 0; group < BUTTON_PORT_COUNT; group++) {
    volatile uint8_t *portReg = buttonPortInputRegs[group];
    portValues[group] = portReg ? *portReg : 0xFF;
  }

  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    uint8_t group = buttonPortGroup[i];
    uint8_t newState = (portValues[group] & buttonBitMask[i]) ? HIGH : LOW;
    if (newState != buttonRawState[i]) {
      buttonRawState[i] = newState;
      buttonLastChange[i] = now;
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

int batteryPercentFromVoltage(float voltage) {
  float pct = ((voltage - BATTERY_MIN_V) * 100.0f) / (BATTERY_MAX_V - BATTERY_MIN_V);
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return (int)(pct + 0.5f);
}

bool shouldShowBatteryIndicator(int percent, unsigned long nowMs) {
  if (percent > 20) return true;
  return ((nowMs / 1000) % 2) == 0;
}

void drawBatteryStatus(unsigned long nowMs) {
  float voltage = readBatteryVoltage();
  int percent = batteryPercentFromVoltage(voltage);

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.fillRect(0, 0, 64, 10, SSD1306_BLACK);

  if (shouldShowBatteryIndicator(percent, nowMs)) {
    display.setCursor(0, 0);
    display.print(F("Bat "));
    display.print(percent);
    display.print(F("%"));
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

void updatePeckDetection(unsigned long nowMs) {
  if (!accelReady) return;

  RTC_SELECT();
  sensors_event_t event;
  if (!accel.getEvent(&event)) return;

  // High-pass filter: keep a slow baseline, detect quick deltas on any axis
  float dx = event.acceleration.x - accelBaselineX;
  float dy = event.acceleration.y - accelBaselineY;
  float dz = event.acceleration.z - accelBaselineZ;

  accelBaselineX += ACCEL_BASELINE_ALPHA * dx;
  accelBaselineY += ACCEL_BASELINE_ALPHA * dy;
  accelBaselineZ += ACCEL_BASELINE_ALPHA * dz;

  float highpassMag = sqrtf(dx * dx + dy * dy + dz * dz);

  static unsigned long lastAccelDebugMs = 0;
  if (SERIAL_ACCEL_DEBUG && (nowMs - lastAccelDebugMs) >= ACCEL_DEBUG_INTERVAL_MS) {
    lastAccelDebugMs = nowMs;
    Serial.print(F("raw_x:")); Serial.print(event.acceleration.x, 3);
    Serial.print(F("\traw_y:")); Serial.print(event.acceleration.y, 3);
    Serial.print(F("\traw_z:")); Serial.print(event.acceleration.z, 3);
    Serial.print(F("\thp_x:")); Serial.print(dx, 4);
    Serial.print(F("\thp_y:")); Serial.print(dy, 4);
    Serial.print(F("\thp_z:")); Serial.print(dz, 4);
    Serial.print(F("\thp_mag:")); Serial.println(highpassMag, 4);
  }

  if (highpassMag > PECK_THRESHOLD_MS2) {
    lastPeckDetectedMs = nowMs;
  }
}

bool peckDetectedRecently(unsigned long nowMs) {
  return accelReady && (nowMs - lastPeckDetectedMs) < PECK_HOLD_MS;
}

void drawPeckIndicator(unsigned long nowMs) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.fillRect(64, 0, 64, 10, SSD1306_BLACK);
  display.setCursor(70, 0);

  if (!accelReady) {
    display.print(F("PECK --"));
  } else if (peckDetectedRecently(nowMs)) {
    display.print(F("PECK!"));
  } else {
    display.print(F("Calm"));
  }
}

void drawStatusHeader(unsigned long nowMs) {
  drawBatteryStatus(nowMs);
  drawPeckIndicator(nowMs);
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
  display.println(F("Enter 4 buttons"));

  display.setTextSize(2);
  display.setCursor(0, 34);
  for (uint8_t i = 0; i < DEFAULT_SEQUENCE_LENGTH; i++) {
    if (i < menuSequenceLength) {
      display.print(menuSequenceBuffer[i]);
    } else {
      display.print('_');
    }
    if (i < DEFAULT_SEQUENCE_LENGTH - 1) display.print(' ');
  }

  display.setTextSize(1);
  display.setCursor(0, 54);
  display.print(F("Step "));
  display.print(menuSequenceLength);
  display.print(F("/"));
  display.print(DEFAULT_SEQUENCE_LENGTH);
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
  for (uint8_t i = 0; i < menuSequenceLength; i++) {
    display.print(menuSequenceBuffer[i]);
    if (i < menuSequenceLength - 1) display.print(' ');
  }

  display.setTextSize(1);
  display.setCursor(0, 50);
  display.println(F("1=Save 2=Again"));
  display.setCursor(0, 58);
  display.println(F("3=Menu 4=Opt"));
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
  display.println(F("3/4=Back"));
  display.display();
}


void showMenuResetConfirm() {
  displayMenuMessage(F("Reset sequences?"),
                     F("1=Yes 2=No"),
                     F("Restore defaults"));
}

void enterMenu() {
  currentMode = MODE_MENU_SELECT_SIDE;
  menuAwaitingRelease = true;
  menuSelectedSide = 0;
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
  storedSequenceLengths[idx] = menuSequenceLength;
  for (uint8_t i = 0; i < menuSequenceLength; i++) {
    storedSequences[idx][i] = menuSequenceBuffer[i];
  }
  for (uint8_t i = menuSequenceLength; i < MAX_SEQUENCE_LENGTH; i++) {
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
  motorStepFood(STEPS_90_FOOD, true);
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

void handleButtonPress(uint8_t index, unsigned long now) {
  uint8_t panel = buttonPanels[index];
  if (panel == PANEL_UNKNOWN) return;
  uint8_t number = buttonNumbers[index];

  if (currentMode == MODE_IDLE) {
    if (panel == 1 && panel1MenuHoldActive) return;
    processSequenceInput(panel, number, now);
    return;
  }

  if (panel != 1) return;
  if (menuAwaitingRelease) return;

  switch (currentMode) {
    case MODE_MENU_SELECT_SIDE:
      if (number >= 1 && number <= SIDE_COUNT) {
        menuSelectedSide = number;
        resetMenuSequenceBuffer();
        currentMode = MODE_MENU_ENTER_SEQUENCE;
        showMenuEnterSequence();
      }
      break;
    case MODE_MENU_ENTER_SEQUENCE:
      if (menuSequenceLength < DEFAULT_SEQUENCE_LENGTH) {
        menuSequenceBuffer[menuSequenceLength++] = number;
        showMenuEnterSequence();
        if (menuSequenceLength >= DEFAULT_SEQUENCE_LENGTH) {
          currentMode = MODE_MENU_CONFIRM;
          showMenuConfirm();
        }
      }
      break;
    case MODE_MENU_CONFIRM:
      if (number == 1) {
        saveMenuSequence();
      } else if (number == 2) {
        resetMenuSequenceBuffer();
        currentMode = MODE_MENU_ENTER_SEQUENCE;
        showMenuEnterSequence();
      } else if (number == 3) {
        menuSelectedSide = 0;
        resetMenuSequenceBuffer();
        currentMode = MODE_MENU_SELECT_SIDE;
        showMenuSelectSide();
      } else if (number == 4) {

        currentMode = MODE_MENU_MORE_OPTIONS;
        showMenuMoreOptions();
      }
      break;
    case MODE_MENU_MORE_OPTIONS:
      if (number == 1) {
        currentMode = MODE_MENU_RESET_CONFIRM;
        showMenuResetConfirm();
      } else if (number == 2) {
        exitMenu();
      } else if (number == 3 || number == 4) {
        currentMode = MODE_MENU_CONFIRM;
        showMenuConfirm();

      }
      break;
    case MODE_MENU_RESET_CONFIRM:
      if (number == 1) {
        resetToFactoryDefaults();
      } else if (number == 2) {
        currentMode = MODE_MENU_MORE_OPTIONS;
        showMenuMoreOptions();
      } else if (number == 3) {
        exitMenu();
      } else if (number == 4) {
        currentMode = MODE_MENU_MORE_OPTIONS;
        showMenuMoreOptions();

      }
      break;
    default:
      break;
  }
}


void readAndSend() {
  // Keep RTC time updated even if not displayed
  RTC_SELECT();
  DateTime nowTs = rtc.now();

  uint16_t distances[SENSOR_COUNT];
  uint8_t statuses[SENSOR_COUNT];
  bool objectDetected = false;
  for (uint8_t ch = 0; ch < SENSOR_CHANNELS; ch++) {
    tcaSelect(ch);
    for (uint8_t i = 0; i < SENSORS_PER_CHANNEL; i++) {
      uint8_t idx = ch * SENSORS_PER_CHANNEL + i;
      sensors[idx].read();
      distances[idx] = sensors[idx].ranging_data.range_mm;
      statuses[idx]  = sensors[idx].ranging_data.range_status;
      if (!objectDetected && statuses[idx] == 0 && distances[idx] < LOG_TRIGGER_DISTANCE_MM) {
        objectDetected = true;
      }
    }
  }

  sendDistancesFramed(distances);

  unsigned long nowMs = millis();
  if (loggingActive) {
    if (nowMs - loggingStartMillis >= LOG_DURATION_MS) {
      loggingActive = false;
      requireClearBeforeNextLog = true;
    } else if (nowMs - lastLogWriteMillis >= LOG_INTERVAL_MS) {
      appendLogEntry(nowTs, distances, statuses);
      lastLogWriteMillis = nowMs;
    }
  } else if (objectDetected && sdAvailable && !requireClearBeforeNextLog) {
    loggingActive = true;
    loggingStartMillis = nowMs;
    appendLogEntry(nowTs, distances, statuses);
    lastLogWriteMillis = nowMs;
    requireClearBeforeNextLog = true;
  }

  if (!objectDetected && !loggingActive) {
    requireClearBeforeNextLog = false;
  }

  if (currentMode != MODE_IDLE) {
    return;
  }

  OLED_SELECT();
  display.clearDisplay();
  unsigned long nowDisplay = millis();
  drawStatusHeader(nowDisplay);
  display.setTextSize(1);
  for (uint8_t i = 0; i < SENSOR_COUNT; i += 2) {
    uint8_t y = 8 + (i / 2) * 8;
    display.setCursor(0, y);
    display.print(F("S")); display.print(i + 1); display.print(F(":"));
    if (statuses[i] == 0) display.print(distances[i]);
    else display.print(F("--"));
    display.setCursor(64, y);
    display.print(F("S")); display.print(i + 2); display.print(F(":"));
    if (statuses[i + 1] == 0) display.print(distances[i + 1]);
    else display.print(F("--"));
  }
  display.setCursor(0,56);
  int8_t pressedIndex = -1;
  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    if (buttonState[i] == LOW) {
      pressedIndex = i;
      break;
    }
  }
  if (pressedIndex >= 0) {
    display.print(F("Button "));
    display.print(buttonNumbers[pressedIndex]);
    if (buttonPanels[pressedIndex] != PANEL_UNKNOWN) {
      display.print(F(", panel "));
      display.print(buttonPanels[pressedIndex]);
    }
  } else {
    if (ahtReady && !isnan(lastTempC) && !isnan(lastHumidity)) {
      display.print(F("Temp "));
      display.print(lastTempC, 1);
      display.print(F("C  "));
      display.print(lastHumidity, 0);
      display.print(F("% RH"));
    } else {
      display.print(F("Env sensor N/A"));
    }
  }
  display.display();
}

// --- SHUTDOWN SEQUENCE ---
void drawSleepingBird(int x,int y){
  OLED_SELECT();
  display.drawBitmap(x, y, birdBitmap, BIRD_W, BIRD_H, SSD1306_WHITE);
}

void shutdownSequence(){
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

  // 2) Quiet sensors (optional)
  for (uint8_t ch=0; ch<SENSOR_CHANNELS; ch++) resetChannelGroup(ch);

  // 3) Cut 5V rail to peripherals
  digitalWrite(GATE_5V_PIN, LOW);
  delay(100);

  // 4) Release power hold – system powers off
  digitalWrite(POWER_HOLD_PIN, LOW);

  // If hardware doesn't cut immediately, wait here
  while(1){}
}





// -------------------- SETUP/LOOP --------------------
void setup() {
  // Serial.begin(SERIAL_BAUD);

  Wire.begin();
  Wire.setClock(100000); // keep it conservative and rock solid

  // Power/Control pins
  pinMode(POWER_HOLD_PIN, OUTPUT);
  pinMode(GATE_5V_PIN,   OUTPUT);
  digitalWrite(POWER_HOLD_PIN, HIGH);   // hold power after boot
  digitalWrite(GATE_5V_PIN,   HIGH);    // enable 5V rail for peripherals

  pinMode(BATTERY_PIN, INPUT);
#if defined(analogReadResolution)
  analogReadResolution(ADC_RESOLUTION);
#endif
  analogRead(BATTERY_PIN); // prime ADC for battery readings

  // Bring up rails BEFORE touching OLED
  pinMode(CHARGER_DETECT_PIN, INPUT);       // expects 0/5V from divider
  pinMode(SWITCH_DETECT_PIN,  INPUT_PULLUP);// LOW when switch ON (change to INPUT if externally driven)

  Timer1.initialize(BUTTON_SCAN_INTERVAL_US); // 10ms button scan intervals
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
  delay(1000);

  // RTC
  RTC_SELECT();
  rtc.begin(); // assume time is already set

  // Temperature/humidity and peck detection sensors share the RTC/OLED channel
  RTC_SELECT();
  ahtReady = aht20.begin();
  tcaSelect(SCREEN_CHANNEL);
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(F("AHT20: "));
  display.println(ahtReady ? F("OK") : F("FAIL"));
  display.display();
  delay(400);

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
  delay(400);

  // SD card (SPI)
  pinMode(SD_CHIP_SELECT_PIN, OUTPUT);
  if (SD.begin(SD_CHIP_SELECT_PIN)) {
    sdAvailable = true;
    tcaSelect(SCREEN_CHANNEL);
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("SD card OK"));
    display.display();
    delay(400);
  } else {
    sdAvailable = false;
    tcaSelect(SCREEN_CHANNEL);
    display.clearDisplay();
    display.setCursor(0,0);
    display.println(F("SD init FAIL"));
    display.display();
    delay(800);
  }

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
    display.print(F("CH ")); display.print(ch); display.println(ok ? F(": OK") : F(": FAIL"));
    display.display();
    delay(400);
  }
  

  // Uno R4 on channel 4 with 10s timeout
  bool unoR4Ok = false;
  if (UNO_R4_EXPECTED) {
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
  if (UNO_R4_EXPECTED) {
    display.println(unoR4Ok ? F("OK") : F("FAIL"));
  } else {
    display.println(F("SKIP"));
  }
  display.display();
  delay(1000);
  
  // Ready banner
  tcaSelect(SCREEN_CHANNEL);
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("birdBox GO"));
  display.display();
  delay(1000);

  // Record the completion of the boot sequence so the shutdown debounce
  // ignores any start-up noise on the switch sense line.
  startupMillis = millis();
}



void loop() {
  uint8_t scansToProcess = 0;
  bool performSensorUpdate = false;

  noInterrupts();
  scansToProcess = buttonScanPending;
  buttonScanPending = 0;
  if (sensorUpdateFlag) {
    sensorUpdateFlag = false;
    performSensorUpdate = true;
  }
  interrupts();

  while (scansToProcess > 0) {
    scanButtons();
    scansToProcess--;
  }

  unsigned long now = millis();

  updateEnvironmentReadings(now);
  updatePeckDetection(now);

  bool hasPending = false;

  noInterrupts();
  hasPending = buttonPendingMask != 0;
  interrupts();

  if (hasPending) {
    for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
      uint16_t bit = (uint16_t)1 << i;
      bool shouldUpdate = false;
      uint8_t rawState = HIGH;

      noInterrupts();
      if (buttonPendingMask & bit) {
        unsigned long lastChange = buttonLastChange[i];
        rawState = buttonRawState[i];
        if ((now - lastChange) >= BUTTON_DEBOUNCE_MS) {
          buttonPendingMask &= ~bit;
          shouldUpdate = true;
        }
      }
      interrupts();

      if (shouldUpdate) {
        uint8_t currentState;
        noInterrupts();
        currentState = buttonState[i];
        interrupts();

        if (currentState != rawState) {
          buttonState[i] = rawState;
          noInterrupts();
          buttonEventMask |= bit;
          interrupts();
        }
      }
    }
  }

  uint16_t pendingButtons = 0;

  noInterrupts();
  pendingButtons = buttonEventMask;
  buttonEventMask = 0;
  interrupts();
  handleMenuActivationHold(now);

  if (currentMode != MODE_IDLE && menuAwaitingRelease && allPanel1Released()) {
    menuAwaitingRelease = false;
    showMenuSelectSide();
  }

  updateSequenceTimeouts(now);

  if (pendingButtons) {
    for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
      if (pendingButtons & ((uint16_t)1 << i)) {
        uint8_t state;
        noInterrupts();
        state = buttonState[i];
        interrupts();

        if (prevButtonState[i] == HIGH && state == LOW) {
          handleButtonPress(i, now);
        }
        prevButtonState[i] = state;
      }
    }
  }

  if (!shutdownInitiated && (now - startupMillis) > SWITCH_STARTUP_GRACE_MS) {
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

  if (performSensorUpdate) {
    readAndSend();
  }
}

