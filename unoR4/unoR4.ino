#include <WiFiS3.h>
#include <Wire.h>
#include "Arduino_LED_Matrix.h"
#include <ArduinoMqttClient.h>


const char ssid[] = "Ponce";      // replace with your network SSID
const char pass[] = "leoponce82";   // replace with your network password

const char broker[] = "10.60.245.204";    // replace with MQTT broker IP or host
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
  Wire.write((byte *)response, sizeof(response)-1);
}

byte STATUS_W[8][12] = {
  {1,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,1,0,0,0,0,1,0,0,1},
  {1,0,0,1,0,0,0,0,1,0,0,1},
  {1,0,1,0,0,0,0,0,0,1,0,1},
  {1,1,0,0,0,0,0,0,0,0,1,1}
};

byte STATUS_M[8][12] = {
  {1,0,0,0,0,0,0,0,0,0,0,1},
  {1,1,0,0,0,0,0,0,0,0,1,1},
  {1,0,1,0,0,0,0,0,0,1,0,1},
  {1,0,0,1,0,0,0,0,1,0,0,1},
  {1,0,0,0,1,0,0,1,0,0,0,1},
  {1,0,0,0,0,1,1,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,1}
};

byte STATUS_E[8][12] = {
  {0,1,1,1,1,1,1,1,1,1,1,0},
  {0,1,0,0,0,0,0,0,0,0,0,0},
  {0,1,0,0,0,0,0,0,0,0,0,0},
  {0,1,1,1,1,1,1,1,0,0,0,0},
  {0,1,0,0,0,0,0,0,0,0,0,0},
  {0,1,0,0,0,0,0,0,0,0,0,0},
  {0,1,0,0,0,0,0,0,0,0,0,0},
  {0,1,1,1,1,1,1,1,1,1,1,0}
};

byte STATUS_C[8][12] = {
  {0,0,1,1,1,1,1,1,1,1,0,0},
  {0,1,0,0,0,0,0,0,0,0,1,0},
  {1,0,0,0,0,0,0,0,0,0,0,1},
  {1,0,0,0,0,0,0,0,0,0,0,0},
  {1,0,0,0,0,0,0,0,0,0,0,0},
  {1,0,0,0,0,0,0,0,0,0,0,1},
  {0,1,0,0,0,0,0,0,0,0,1,0},
  {0,0,1,1,1,1,1,1,1,1,0,0}
};

void showStatus(char msg) {
  matrix.clear();
  switch (msg) {
    case 'W':
      matrix.renderBitmap(STATUS_W, 8, 12);
      break;
    case 'M':
      matrix.renderBitmap(STATUS_M, 8, 12);
      break;
    case 'E':
      matrix.renderBitmap(STATUS_E, 8, 12);
      break;
    case 'C':
      matrix.renderBitmap(STATUS_C, 8, 12);
      break;
  }
}


void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_ADDRESS);       // join I2C bus with specified address
  Wire.onReceive(onReceive);     // register event handlers
  Wire.onRequest(onRequest);

  matrix.begin();
  showStatus('W');

  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
  Serial.println("Connected to WiFi");

  showStatus('M');
  if (!mqttClient.connect(broker, brokerPort)) {
    Serial.println("MQTT connection failed");
    showStatus('E');
  } else {
    Serial.println("MQTT connected");
    showStatus('C');
  }
}

void loop() {
  mqttClient.poll();
  mqttClient.beginMessage(topic);
  mqttClient.print(random(0, 100));
  mqttClient.endMessage();
  delay(1000);
}

