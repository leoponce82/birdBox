#include <WiFiS3.h>
#include <Wire.h>
#include "Arduino_LED_Matrix.h"
#include <ArduinoMqttClient.h>
#include <Ticker.h>


const char ssid[] = "Ponce";      // replace with your network SSID
const char pass[] = "leoponce82";   // replace with your network password

const char broker[] = "10.60.245.204";    // replace with MQTT broker IP or host
const int  brokerPort = 1883;
const char topic[] = "birdBox/unoR4/sensors";

const int I2C_ADDRESS = 0x08;
#define HANDSHAKE_SIZE 1
String answer = "K";
#define SENSOR_COUNT    12
#define PAYLOAD_BYTES   (SENSOR_COUNT * 2)  // 24
volatile uint16_t rcvDistances[SENSOR_COUNT];
volatile uint8_t st = 0;          // 0: wait 'D', 1: off, 2: cnt, 3: data
volatile uint8_t f_off = 0;
volatile uint8_t f_cnt = 0;
volatile uint8_t f_need = 0;
volatile uint8_t f_idx = 0;
volatile uint8_t dataBuf[2 * 6];   // up to 6 values per frame (we use 4)
// Bitmask to know when all 12 were updated (bits 0..11)
volatile uint16_t gotMask = 0;

// small accumulator buffer
volatile uint8_t rxbuf[PAYLOAD_BYTES];
volatile uint8_t rxidx = 0;

ArduinoLEDMatrix matrix;
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

Ticker publishTicker;
volatile bool publishFlag = false;
volatile bool haveData = false;

void publishISR() { publishFlag = true; }


void onReceive(int) {
  while (Wire1.available()) {
    uint8_t b = (uint8_t)Wire1.read();

    if (st == 0) {                       // wait header
      st = (b == 'D') ? 1 : 0;
    }
    else if (st == 1) {                  // got offset
      f_off = b;
      st = 2;
    }
    else if (st == 2) {                  // got count
      f_cnt = b;
      if (f_cnt == 0 || f_off + f_cnt > SENSOR_COUNT) { st = 0; continue; }
      f_need = 2 * f_cnt;
      f_idx = 0;
      st = 3;
    }
    else if (st == 3) {                  // collect data bytes
      if (f_idx < sizeof(dataBuf)) dataBuf[f_idx++] = b;
      if (f_idx >= f_need) {
        // parse little-endian values into rcvDistances
        for (uint8_t i = 0; i < f_cnt; i++) {
          uint8_t lo = dataBuf[2*i + 0];
          uint8_t hi = dataBuf[2*i + 1];
          rcvDistances[f_off + i] = (uint16_t)lo | ((uint16_t)hi << 8);
          gotMask |= (1u << (f_off + i));
        }
        // all 12 received at least once?
        if (gotMask == 0x0FFF) { haveData = true; gotMask = 0; }

        st = 0; // ready for next frame
      }
    }
  }
}

void onRequest() {
  Wire1.write('K');   // single byte handshake
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
  Wire1.begin(I2C_ADDRESS);       // join I2C bus with specified address
  Wire1.setClock(100000); // keep it conservative and rock solid
  Wire1.onReceive(onReceive);     // register event handlers
  Wire1.onRequest(onRequest);

  matrix.begin();
  showStatus('W');

  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(1000);
  }
  Serial.println("Connected to WiFi");

  showStatus('M');
  if (!mqttClient.connect(broker, brokerPort)) {
    Serial.println("MQTT connection failed");
    showStatus('E');
  } else {
    Serial.println("MQTT connected");
    showStatus('C');
    publishTicker.attach_ms(100, publishISR);
  }
}

void loop() {
  mqttClient.poll();
  if (publishFlag && haveData) {
    publishFlag = false;
    noInterrupts();
    uint16_t snap[SENSOR_COUNT];
    for (int i = 0; i < SENSOR_COUNT; i++) snap[i] = rcvDistances[i];
    interrupts();
    mqttClient.beginMessage(topic);
    for (int i = 0; i < SENSOR_COUNT; i++) {
      if (i) mqttClient.print(',');
      mqttClient.print(snap[i]);
    }
    mqttClient.endMessage();
  }
  delay(100);
  // mqttClient.poll(); 
  // mqttClient.beginMessage(random_topic); 
  // mqttClient.print(random(0, 100)); 
  // mqttClient.endMessage(); 
  // delay(1000);
}

