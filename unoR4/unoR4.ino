#include <WiFiS3.h>
#include <Wire.h>

const char ssid[] = "YOUR_SSID";      // replace with your network SSID
const char pass[] = "YOUR_PASSWORD";   // replace with your network password

const int I2C_ADDRESS = 0x08;

void onReceive(int numBytes) {
  while (Wire.available()) {
    char c = Wire.read();
    // handle received data
  }
}

void onRequest() {
  const char response[] = "OK";
  Wire.write((const uint8_t*)response, sizeof(response)-1);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_ADDRESS);       // join I2C bus with specified address
  Wire.onReceive(onReceive);     // register event handlers
  Wire.onRequest(onRequest);

  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
  Serial.println("Connected to WiFi");
}

void loop() {
  // main loop can be extended as needed
  delay(1000);
}

