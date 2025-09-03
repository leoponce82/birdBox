#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h> 
#include <Stepper.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3D  // Adjust for your OLED
#define SCREEN_CHANNEL 4 

#define SENSOR_COUNT 6  // Total number of sensors
#define TCA_ADDRESS 0x70  // Default I²C address of the TCA9548A multiplexer
#define CHANNEL_COUNT 2  // Using 4 channels

#define MAGNET_SENSOR_PIN 2  // Change if using a different pin
#define LED_BUILTIN 3

#define TAP_THRESHOLD 1.0

#define PRESENCE_PIN 40

#define MOTOR_STEPS 200 //64 for Small stepper!
#define MOTOR_PIN1 22
#define MOTOR_PIN2 23
#define MOTOR_PIN3 25
#define MOTOR_PIN4 24
Stepper stepper(MOTOR_STEPS, MOTOR_PIN1, MOTOR_PIN2, MOTOR_PIN3, MOTOR_PIN4);


#define LOG_INTERVAL 5000  

const int BATTERY_PIN = A0;

// Button pins
const int buttonPins[4] = {30, 31, 32, 33};
bool buttonStates[4];
unsigned long buttonPressStart = 0;
const unsigned long menuHoldDuration = 3000; // 3 seconds
enum SystemState { ACTIVE_MODE, MENU_MODE };
SystemState currentState = ACTIVE_MODE;

const uint8_t sensorsPerChannel = 3;
// const uint8_t xshutPins[SENSOR_COUNT] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
const uint8_t xshutPins[SENSOR_COUNT] = {13, 12, 11, 10, 9, 8, 6, 5, 4, 3, 2, 1};
// const uint8_t sensorAddresses[SENSOR_COUNT] = {
//   0x30, 0x31, 0x32,  // Channel 0
//   0x33, 0x34, 0x35,  // Channel 1
//   0x36, 0x37, 0x38,  // Channel 2
//   0x39, 0x3A, 0x3B   // Channel 3
// };
const uint8_t sensorAddresses[SENSOR_COUNT] = {
  0x30, 0x31, 0x32 // Channel 1
};

// SD Card Configuration
const int chipSelect = 53; // Adjust based on your board (for MEGA, it's usually 53)
File dataFile;

// RTC Configuration
RTC_DS3231 rtc;  // Initialize RTC module

// Accelerometer
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
// float magnitude = sqrt(x*x + y*y + z*z);

unsigned long lastSDWriteTime = 0;  
String currentLogFile = "";  // Store filename (8.3 format)
int lastLoggedDay = -1;  

VL53L1X sensors[SENSOR_COUNT];
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
void setup() {
  analogReference(INTERNAL1V1);
  // Serial.begin(9600);
  Wire.begin();
  stepper.setSpeed(50);

 //  Initialize presence sensor Pin
  pinMode(PRESENCE_PIN, INPUT_PULLUP);

  // ✅ Initialize Magnetic Sensor Pin
  pinMode(MAGNET_SENSOR_PIN, INPUT_PULLUP);  // Internal pull-up resistor
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  // Ensure LED is off initially

  //  Initialize Buttons
   for (int i = 0; i < 4; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP); // Active-low
  }


  // Initialize OLED
  tcaSelect(SCREEN_CHANNEL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    while (1); // Halt if OLED fails
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Initializing..."));
  display.display();

  delay(500);
  //Initialize accelerometer
  if (!accel.begin()) {
    tcaSelect(SCREEN_CHANNEL);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("No LSM303 found!");
    display.display();
    while (1);
  }
  accel.setRange(LSM303_RANGE_4G);
  accel.setMode(LSM303_MODE_NORMAL);

  // Initialize RTC Module
  if (!rtc.begin()) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("RTC Failed"));
    display.display();
    while (1);
  }

  // Uncomment this line ONCE to set the time, then re-upload without it
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // Initialize SD Card
  if (!SD.begin(chipSelect)) {
    tcaSelect(SCREEN_CHANNEL);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("SD Card Failed"));
    display.display();
    while (1);  
  }
  tcaSelect(SCREEN_CHANNEL);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("SD Card Ready"));
  display.display();
  delay(500);


  // Initialize sensors on each channel
  for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
    tcaSelect(ch);
    delay(50);

    for (uint8_t i = 0; i < sensorsPerChannel; i++) {
      uint8_t sensorIndex_pins = (ch * sensorsPerChannel) + i;
      pinMode(xshutPins[sensorIndex_pins], OUTPUT);
      digitalWrite(xshutPins[sensorIndex_pins], LOW);
    }

    delay(100);

    for (uint8_t i = 0; i < sensorsPerChannel; i++) {
      tcaSelect(ch);
      uint8_t sensorIndex = i;
      uint8_t sensorIndex_pins = (ch * sensorsPerChannel) + i;
      digitalWrite(xshutPins[sensorIndex_pins], HIGH);
      delay(10);

      sensors[sensorIndex_pins].setTimeout(1000);
      if (!sensors[sensorIndex_pins].init()) {
        tcaSelect(SCREEN_CHANNEL);
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print(F("Sensor "));
        display.print(sensorIndex + 1);
        display.println(F(" failed"));
        display.display();
        while (1);
      }

      sensors[sensorIndex_pins].setAddress(sensorAddresses[sensorIndex]);
      // sensors[sensorIndex].setDistanceMode(VL53L1X::Short);
      sensors[sensorIndex_pins].setMeasurementTimingBudget(20000);
      sensors[sensorIndex_pins].startContinuous(20);

      tcaSelect(SCREEN_CHANNEL);
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print(F("Sensor "));
      display.print(sensorIndex + 1);
      display.print(F(" ready"));
      display.display();
      delay(500);
    }
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(F("Channel ready: "));
    display.print(ch);
    display.display();
    delay(500);
  }

  tcaSelect(SCREEN_CHANNEL);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("All sensors initialized."));
  display.display();
  delay(1000);

  // Get the current date and create the first log file
  DateTime now = rtc.now();
  updateLogFile(now);

}

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

void loop() {
  int rawADC = analogRead(BATTERY_PIN);
  float voltageAtPin = rawADC * (1.1 / 1023.0);  // now using 1.1V reference
  float batteryVoltage = voltageAtPin * 12.0;
  int batteryPercent = constrain(map(batteryVoltage * 100, 900, 1260, 0, 100), 0, 100);

  // Serial.print("Battery Voltage: ");
  // Serial.print(batteryVoltage);
  // Serial.println(" V");
  

  static unsigned long lastSDWriteTime = 0;  // Track last SD write time

  tcaSelect(SCREEN_CHANNEL);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  tcaSelect(SCREEN_CHANNEL);
  display.setCursor(SCREEN_WIDTH - 30, 0); // top-right corner
  display.print(String(batteryPercent) + "%");
  display.setCursor(0, 0); // top-left for time (same line)
  
  bool presenceDetected = (digitalRead(PRESENCE_PIN) == HIGH); 
  if (presenceDetected) {
    digitalWrite(LED_BUILTIN, HIGH);  // Turn on LED
    
  }else{
    digitalWrite(LED_BUILTIN, LOW); 
  }

  // --- Tap detection ---
  sensors_event_t event;
  accel.getEvent(&event);

  // Simple magnitude check
  float accelMag = sqrt(event.acceleration.x * event.acceleration.x +
                        event.acceleration.y * event.acceleration.y +
                        event.acceleration.z * event.acceleration.z);

  if (abs(accelMag - 9.8) > TAP_THRESHOLD) {  // Adjust sensitivity
    tcaSelect(SCREEN_CHANNEL);
    display.setCursor(0, SCREEN_HEIGHT - 8);  // Last line
    display.print("TAP DETECTED!");
  }


  // ✅ Magnet Sensor Reading
  bool magnetDetected = (digitalRead(MAGNET_SENSOR_PIN) == LOW);  // LOW means magnet detected
  if (magnetDetected) {
    // digitalWrite(LED_BUILTIN, HIGH);  // Turn on LED
    display.println(F("MAGNET DETECTED"));

    // Shut down sensors for low current draw
    for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
      tcaSelect(ch);
      for (uint8_t i = 0; i < sensorsPerChannel; i++) {
        sensors[i].stopContinuous(); // Stop ranging
      }
    }

    // Rotate stepper motor 90 degrees (50 steps)
    // stepper.step(50); //good for nema17 motor
    stepper.step(50);
    // Resume sensors
    for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
      tcaSelect(ch);
      for (uint8_t i = 0; i < sensorsPerChannel; i++) {
        sensors[i].startContinuous(20);
      }
    }

    delay(500);

    } else {
      // digitalWrite(LED_BUILTIN, LOW);  // Turn off LED
      digitalWrite(MOTOR_PIN1, LOW);
      digitalWrite(MOTOR_PIN2, LOW);
      digitalWrite(MOTOR_PIN3, LOW);
      digitalWrite(MOTOR_PIN4, LOW);
      
    }

  DateTime now = rtc.now();
  // Check if the day has changed and update the log file
  if (now.day() != lastLoggedDay) {
    updateLogFile(now);
  }

  String logData = String(now.year()) + "-" + 
                   String(now.month()) + "-" + 
                   String(now.day()) + " " + 
                   String(now.hour()) + ":" + 
                   String(now.minute()) + ":" + 
                   String(now.second()) + ", ";

  tcaSelect(SCREEN_CHANNEL);
  display.print(F("Time: "));
  display.print(now.hour());
  display.print(F(":"));
  display.print(now.minute());
  display.print(F(":"));
  display.println(now.second());

  for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
    tcaSelect(ch);

    for (uint8_t i = 0; i < sensorsPerChannel; i++) {
      tcaSelect(ch);
      uint16_t distance = sensors[i].read();
      
      tcaSelect(SCREEN_CHANNEL);
      display.print(F("S"));
      display.print(i + 1);
      if (!sensors[i].timeoutOccurred()) {
        tcaSelect(SCREEN_CHANNEL);
        display.print(F(": "));
        display.print(distance);
        display.println(F(" mm"));
        logData += String(distance) + ", ";  
      } else {
        display.println(F(": Timeout"));
        logData += "Timeout, ";  
      }
    }
  }

  tcaSelect(SCREEN_CHANNEL);
  display.display();

  // Write to SD card every 60 seconds
  if (millis() - lastSDWriteTime >= LOG_INTERVAL) {
    saveToSD(logData);
    lastSDWriteTime = millis();
  }
  delay(50); // Faster refresh rate
  
}


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Function to update the log filename when the day changes
void updateLogFile(DateTime now) {
  lastLoggedDay = now.day();
  currentLogFile = String(now.month()) + "_" +
                   String(now.day()) + ".txt";

  // Check if the file exists
  bool fileExists = SD.exists(currentLogFile);

  // Open the file in append mode
  dataFile = SD.open(currentLogFile, FILE_WRITE);
  
  if (dataFile) {
    if (fileExists) {
      // If the file already exists, add a reset marker
      dataFile.println("-----------------------------");
      dataFile.println("RESET DETECTED: Logging Resumed");
      dataFile.println("-----------------------------");
    } else {
      // If it's a new file, write column headers
      dataFile.println("Date, Time, Sensor1 (mm), Sensor2 (mm), Sensor3 (mm), Sensor4 (mm), Sensor5 (mm), Sensor6 (mm)");
    }
    dataFile.close();
  }
}



// Function to save data to SD card
void saveToSD(String data) {
  dataFile = SD.open(currentLogFile, FILE_WRITE);
  if (dataFile) {
    dataFile.println(data);
    dataFile.flush();
    dataFile.close();
  } else {
    tcaSelect(SCREEN_CHANNEL);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("SD Write Failed"));
    display.display();
    delay(1000);
  }
}