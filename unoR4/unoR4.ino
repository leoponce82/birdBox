#include <WiFiS3.h>
#include <Wire.h>
#include <Arduino_LED_Matrix.h>
#include <ArduinoMqttClient.h>


const char ssid[] = "Ponce";      // replace with your network SSID
const char pass[] = "leoponce82";   // replace with your network password

const char broker[] = "BROKER_IP";    // replace with MQTT broker IP or host
const int  brokerPort = 1883;
const char topic[] = "unoR4/random";

const int I2C_ADDRESS = 0x08;

ArduinoLEDMatrix matrix;
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);


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

void showStatus(const char *msg) {
  matrix.clear();
  matrix.print(msg);
}


void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_ADDRESS);       // join I2C bus with specified address
  Wire.onReceive(onReceive);     // register event handlers
  Wire.onRequest(onRequest);

  matrix.begin();
  showStatus("W");

  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
  Serial.println("Connected to WiFi");

  showStatus("M");
  if (!mqttClient.connect(broker, brokerPort)) {
    Serial.println("MQTT connection failed");
    showStatus("E");
  } else {
    Serial.println("MQTT connected");
    showStatus("C");
  }
}

void loop() {
  mqttClient.poll();
  mqttClient.beginMessage(topic);
  mqttClient.print(random(0, 100));
  mqttClient.endMessage();
}

void loop() {
  // main loop can be extended as needed
  delay(1000);
}

