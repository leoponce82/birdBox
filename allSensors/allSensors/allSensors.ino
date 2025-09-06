#include <Wire.h>
#include <VL53L1X.h>          // Pololu VL53L1X library
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>
#include <TimerOne.h>
#include <PinChangeInterrupt.h>

// --- Power / control pins ---
#define POWER_HOLD_PIN     26   // output, goes HIGH after boot
#define CHARGER_DETECT_PIN 27   // input, 5V via divider when charger is present
#define SWITCH_DETECT_PIN  25   // input, reads LOW when main switch is ON
#define GATE_5V_PIN        23   // output, goes HIGH after boot


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

volatile bool updateFlag = false;
volatile bool shutdownFlag = false;

void timerISR() { updateFlag = true; }
void switchISR() { if (digitalRead(SWITCH_DETECT_PIN) == HIGH) shutdownFlag = true; }

// -------------------- STEP/DIR DRIVER --------------------
// New driver: STEP on pin 33, DIR on pin 31
#define STEP_PIN       33
#define DIR_PIN        31
#define MICROSTEPS      8     // set to the hardware microstep setting (1,2,4,8,16, ...)
#define STEPS_PER_REV 200     // 1.8° motor
const int STEPS_90 = (STEPS_PER_REV * MICROSTEPS) / 4;  // quarter turn

// pulse timing (adjust for your driver/motor)
const unsigned int STEP_PULSE_US = 600;   // high/low pulse width

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
  digitalWrite(DIR_PIN, dirCW ? HIGH : LOW);
  delayMicroseconds(2); // DIR setup time
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_PULSE_US);
  }
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
  UNO_R4_SELECT();
  delay(3); // TCA settle

  Wire.beginTransmission(UNO_R4_ADDR);
  for (uint8_t i = 0; i < count; i++) {
    uint16_t v = vals[i];
    Wire.write((uint8_t)(v & 0xFF));        // low byte
    Wire.write((uint8_t)((v >> 8) & 0xFF)); // high byte
  }
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
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
    if (err) {
      // show err on OLED if you want
      return false;
    }
    delayMicroseconds(200); // tiny gap
  }
  return true;
}

// Send distances in framed chunks (4 values per frame)
bool sendDistancesFramed(const uint16_t *vals) {
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
    if (err != 0) {
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


void readAndSend() {
  // Keep RTC time updated even if not displayed
  RTC_SELECT();
  rtc.now();

  uint16_t distances[SENSOR_COUNT];
  uint8_t statuses[SENSOR_COUNT];
  for (uint8_t ch = 0; ch < SENSOR_CHANNELS; ch++) {
    tcaSelect(ch);
    for (uint8_t i = 0; i < SENSORS_PER_CHANNEL; i++) {
      uint8_t idx = ch * SENSORS_PER_CHANNEL + i;
      sensors[idx].read();
      distances[idx] = sensors[idx].ranging_data.range_mm;
      statuses[idx]  = sensors[idx].ranging_data.range_status;
    }
  }

  sendDistancesFramed(distances);

  OLED_SELECT();
  display.clearDisplay();
  display.setTextSize(1);
  for (uint8_t i = 0; i < SENSOR_COUNT; i += 2) {
    uint8_t y = (i / 2) * 10;
    display.setCursor(0, y);
    display.print(F("S")); display.print(i + 1); display.print(F(":"));
    if (statuses[i] == 0) display.print(distances[i]);
    else display.print(F("--"));
    display.setCursor(64, y);
    display.print(F("S")); display.print(i + 2); display.print(F(":"));
    if (statuses[i + 1] == 0) display.print(distances[i + 1]);
    else display.print(F("--"));
  }
  display.display();

  bool s1Valid = (statuses[0] == 0);
  if (s1Valid && distances[0] < TRIGGER_MM) {
    if (s1HitCount < 255) s1HitCount++;
  } else {
    s1HitCount = 0;
  }

  bool doMove = (s1HitCount >= REQ_CONSEC) && (millis() - lastMoveMs > moveCooldownMs);
  if (doMove) {
    motorStep(STEPS_90, true);
    lastMoveMs = millis();
    s1HitCount = 0;
  }
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



// --- state for motor trigger & switch edge ---
unsigned long lastMoveMs=0;
const unsigned long moveCooldownMs=1500;
const uint16_t TRIGGER_MM = 150;
const uint8_t  REQ_CONSEC  = 3;
uint8_t s1HitCount = 0;




// -------------------- SETUP/LOOP --------------------
void setup() {
  Wire.begin();
  Wire.setClock(100000); // keep it conservative and rock solid

  // Power/Control pins
  pinMode(POWER_HOLD_PIN, OUTPUT);
  pinMode(GATE_5V_PIN,   OUTPUT);
  digitalWrite(POWER_HOLD_PIN, HIGH);   // hold power after boot
  digitalWrite(GATE_5V_PIN,   HIGH);    // enable 5V rail for peripherals

  // Bring up rails BEFORE touching OLED
  pinMode(CHARGER_DETECT_PIN, INPUT);       // expects 0/5V from divider
  pinMode(SWITCH_DETECT_PIN,  INPUT_PULLUP);// LOW when switch ON (change to INPUT if externally driven)

  attachPCINT(digitalPinToPCINT(SWITCH_DETECT_PIN), switchISR, CHANGE);

  Timer1.initialize(100000); // 100ms intervals
  Timer1.attachInterrupt(timerISR);

  // STEP/DIR outputs
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);

  // Now init OLED on its TCA channel
  
  tcaSelect(SCREEN_CHANNEL);
  delay(500);  // let the mux settle
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  delay(2000);


  // OLED
  tcaSelect(SCREEN_CHANNEL);
  // display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("Booting..."));
  display.display();
  delay(1000);

  // RTC
  RTC_SELECT();
  rtc.begin(); // assume time is already set

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
  unsigned long startAttempt = millis();
  while (millis() - startAttempt < 10000 && !unoR4Ok) {
    unoR4Ok = connectUnoR4();
    if (!unoR4Ok) delay(500);
  }
  tcaSelect(SCREEN_CHANNEL);
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(F("UNO R4: "));
  display.println(unoR4Ok ? F("OK") : F("FAIL"));
  display.display();
  delay(1000);
  
  // Ready banner
  tcaSelect(SCREEN_CHANNEL);
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(F("All sensors ready."));
  display.display();
  delay(1000);
}



void loop() {
  if (shutdownFlag) {
    shutdownFlag = false;
    shutdownSequence();
  }
  if (updateFlag) {
    updateFlag = false;
    readAndSend();
  }
}

