#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3D  // Adjust based on your OLED's I²C address

#define SENSOR_COUNT 2  // Number of sensors
const uint8_t xshutPins[SENSOR_COUNT] = {2, 3};  // XSHUT pins for each sensor
const uint8_t sensorAddresses[SENSOR_COUNT] = {0x30, 0x31};  // Unique I²C addresses

VL53L1X sensors[SENSOR_COUNT];
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Wire.begin();

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    // Display error message on OLED
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.setTextColor(SSD1306_WHITE);
    display.println(F("OLED init failed"));
    display.display();
    while (1); // Halt execution
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.setTextColor(SSD1306_WHITE);
  display.println(F("Initializing sensors..."));
  display.display();

  // Initialize each sensor
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);  // Keep sensor in reset
  }

  delay(100);  // Ensure all sensors are in reset

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    digitalWrite(xshutPins[i], HIGH);  // Enable sensor
    delay(10);  // Allow sensor to boot up

    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      // Display error message on OLED
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print(F("Sensor init failed at pin "));
      display.println(xshutPins[i]);
      display.display();
      while (1); // Halt execution
    }

    sensors[i].setAddress(sensorAddresses[i]);
    sensors[i].setMeasurementTimingBudget(20000);
    sensors[i].startContinuous(20);

    // Display sensor initialization status on OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(F("Sensor "));
    display.print(i + 1);
    display.print(F(" initialized at 0x"));
    display.println(sensorAddresses[i], HEX);
    display.display();
    delay(500); // Short delay to read the message
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("All sensors initialized."));
  display.display();
  delay(1000); // Short delay to read the message
}

void loop() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint16_t distance = sensors[i].read();
    if (!sensors[i].timeoutOccurred()) {
      display.print(F("S"));
      display.print(i + 1);
      display.print(F(": "));
      display.print(distance);
      display.println(F(" mm"));
    } else {
      display.print(F("S"));
      display.print(i + 1);
      display.println(F(": Timeout"));
    }
  }

  display.display();
  delay(200);
}
