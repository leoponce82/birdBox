#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_VL53L1X.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3D

#define IRQ_PIN 2
#define XSHUT_PIN 3
#define MAX_VALID_DISTANCE 3500  // Set max valid distance in mm

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Initializing..."));
  display.display();

  // Initialize VL53L1X sensor
  Wire.begin();
  if (!vl53.begin(0x, &Wire)) {
    Serial.println(F("Error initializing VL53L1X"));
    display.println(F("Sensor Error!"));
    display.display();
    while (1) delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));
  
  vl53.setTimingBudget(200);
  vl53.startRanging();
  display.clearDisplay();
}

void loop() {
  int16_t distance;

  if (vl53.dataReady()) {
    if (vl53.vl_status != 0) {
    Serial.println(F("Bad reading, ignoring!"));
    return;
}
    distance = vl53.distance();

        if (distance == -1 || distance > MAX_VALID_DISTANCE) {
      Serial.println(F("No object detected!"));
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(10, 20);
      display.print(F("No Object"));
      display.display();
    } else {
      Serial.print(F("Distance: "));
      Serial.print(distance);
      Serial.println(F(" mm"));

      // Display on OLED
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(10, 20);
      display.print(F("Dist: "));
      display.print(distance);
      display.print(F(" mm"));
      display.display();
    }

  }
}
