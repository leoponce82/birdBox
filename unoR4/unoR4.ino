#include <WiFiS3.h>
#include <Wire.h>
#include "Arduino_LED_Matrix.h"
#include <ArduinoMqttClient.h>
#include <Ticker.h>


// ===== WiFi =====
const char WIFI_SSID[] = "PONCE-SNOW";
const char WIFI_PASS[] = "leoponce82";


// const char broker[] = "192.168.137.169";    // replace with MQTT broker IP or host
// const int  brokerPort = 8883;
// const char topic[] = "birdBox/unoR4/sensors";

const char* BROKER_HOST = "192.168.137.169"; // must match SAN (or use "leandro-VirtualBox" if in SAN)
const int   BROKER_PORT = 8883;
const char* MQTT_USER = "leandro";
const char* MQTT_PASS = "admin"; // <-- change to your real password
const char* PUB_TOPIC = "birdBox/unoR4/sensors";

// ===== Root CA (PEM) =====
// Paste /etc/mosquitto/certs/ca.crt here, including BEGIN/END lines.
const char caCert[] = R"CA(
-----BEGIN CERTIFICATE-----
MIIFITCCAwmgAwIBAgIUIhStQKoK6cpJDp71tpCOKfTLRI8wDQYJKoZIhvcNAQEL
BQAwIDEeMBwGA1UEAwwVbGVhbmRyby1WaXJ0dWFsQm94IENBMB4XDTI1MTAyNzE0
MTEyNFoXDTM1MTAyNTE0MTEyNFowIDEeMBwGA1UEAwwVbGVhbmRyby1WaXJ0dWFs
Qm94IENBMIICIjANBgkqhkiG9w0BAQEFAAOCAg8AMIICCgKCAgEAqbjh5ue7kZ4E
XLc/WF8fQb81UafyXO5I4iBT5tFLG3mKX8bTQ2O/fd6W00sZuHnXyq/8vv4UDA+N
Si0FrqNBtuJBtFMbrP8aQWfcj8epop+nIse17sNUxWcBul5oKOqehB+2YmxkgkTw
/+z6sZefOCpL4ZuPT2JsSQmYZWB5T9FjZszDHznC0+qWeE6D0Pz9bEZqTrb+OGPv
azKTGR3Bk/yyOvhJASSlPbWdLbyH1PjUwRdM56vrnq/iRnaIxwG0EGS7hoGWfnZS
M5KGSi1TS5KiXJpbUyNcH23JNZeP75Nxu7Z4WSaV62lMnt05JAx11d1j3qfAcK28
qPao5miCGT1AVV2uSmHbExgrC0Z+66YHPZInvi5EN0pPC/wr+2zXsl1TDUZaX8eh
C/8SthTrFbkdYTZNRtjI3+zyMC7IrdZkmgv6LJNbowtocOHNhR7kHzfDAbeIAGsM
YbGbnN5HIk9Vbb1n5xZ8vXj7iKpBQi6QC43iEyDkbnEonlzLyA0GZOyE4PFAmK2U
kGe5C6J953Zyu5B1PzO55z35UWkppI+pNVmDlw1ntSqDQ0fgweo0RJpI8vn9Q1Bg
2GWVhw48XDfxueMW32GLVDk5urBDYGTD6Ydc2Myahe8dnfYPkgse0EA6i4vadb9c
IpT+xcqZyOJYgepP2OOaQMBJ7NdH9vMCAwEAAaNTMFEwHQYDVR0OBBYEFBEizyGi
vlN73ZRrTyTl3XEdCeaGMB8GA1UdIwQYMBaAFBEizyGivlN73ZRrTyTl3XEdCeaG
MA8GA1UdEwEB/wQFMAMBAf8wDQYJKoZIhvcNAQELBQADggIBAGzus1o1xGqTc5rY
4CVEP5nv9Pxd90NB82xCAA9cKpU6E9DMf0U92EOw7jLlk0646nHLM95QDtT7LwEk
cHtHcz9qtzTQ6+Kf/SDG6ZaIOo6NDdyqJgs4FgMBm4YPCxsL6UpLyzyC+0ElSZRB
+Sr1otaXqqhNpitkXs4nUNLrYSqlwYKWTALsimhUNXw0iL+c4Yq63I1xNZ3AuVZy
tTK0lZUsKYYNJchJOlV1SZFrEcN8j/60zbw8z74Iocv/+LUVQoXgeQGexDNEiDFL
UwoSarHCXxRbaj/byyy/Vbz9nl1XrR+gc6Rk9ZNn8yTUmY+lxpXwbFmFQjXVdHhK
R1W4QqdTf02AvLnoznDJGCPLtPxR2E5J0dZduX1ARpnVzvbAM9BogIbTtnz48cfJ
rI77PTusbmDEMHtmzeXkHNdiagTlUVz17DrSGi7359mK1LW10XwU4DGEbXM1AdoP
Uts9sqaq7Sp5h44l2Qf9ofzzbXj38afk13SY+oyVurYWRpSRGGeoBBPz9IBot81g
yA02dYOfEiWwv5yPiHIwmPXZcFFEetmaCy1tcmjyLkDUNpFvGnS9eRgb8hN/EUlO
hP64u0mR2k6g+0fQlzrzVx5iEar2RY0p/BLxqB2nQKFOA0ymKhLuHMT7ilH4IuxP
AwV+QsP/T+PKd3cmsCWO9/FgVQH5
-----END CERTIFICATE-----
)CA";

// ===== I2C / sensor framing (unchanged) =====
const int I2C_ADDRESS = 0x08;
#define HANDSHAKE_SIZE 1
#define SENSOR_COUNT    12
#define PAYLOAD_BYTES   (SENSOR_COUNT * 2)

volatile uint16_t rcvDistances[SENSOR_COUNT];
volatile uint8_t  st = 0, f_off = 0, f_cnt = 0, f_need = 0, f_idx = 0;
volatile uint8_t  dataBuf[2 * 6];
volatile uint16_t gotMask = 0;
volatile uint8_t  rxbuf[PAYLOAD_BYTES];
volatile uint8_t  rxidx = 0;

ArduinoLEDMatrix matrix;

// ==== Use TLS client (IMPORTANT) ====
WiFiSSLClient sslClient;         // << TLS transport
MqttClient mqttClient(sslClient);

// publish interval in milliseconds
const unsigned long PUBLISH_INTERVAL_MS = 100;
unsigned long lastPublish = 0;
volatile bool haveData = false;

// ----- bitmaps (unchanged) -----
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
    case 'W': matrix.renderBitmap(STATUS_W, 8, 12); break;
    case 'M': matrix.renderBitmap(STATUS_M, 8, 12); break;
    case 'E': matrix.renderBitmap(STATUS_E, 8, 12); break;
    case 'C': matrix.renderBitmap(STATUS_C, 8, 12); break;
  }
}

// ----- I2C handlers (unchanged) -----
void onReceive(int) {
  while (Wire1.available()) {
    uint8_t b = (uint8_t)Wire1.read();

    if (st == 0) { st = (b == 'D') ? 1 : 0; }
    else if (st == 1) { f_off = b; st = 2; }
    else if (st == 2) {
      f_cnt = b;
      if (f_cnt == 0 || f_off + f_cnt > SENSOR_COUNT) { st = 0; continue; }
      f_need = 2 * f_cnt; f_idx = 0; st = 3;
    }
    else if (st == 3) {
      if (f_idx < sizeof(dataBuf)) dataBuf[f_idx++] = b;
      if (f_idx >= f_need) {
        for (uint8_t i = 0; i < f_cnt; i++) {
          uint8_t lo = dataBuf[2*i + 0];
          uint8_t hi = dataBuf[2*i + 1];
          rcvDistances[f_off + i] = (uint16_t)lo | ((uint16_t)hi << 8);
          gotMask |= (1u << (f_off + i));
        }
        if (gotMask == 0x0FFF) { haveData = true; gotMask = 0; }
        st = 0;
      }
    }
  }
}

void onRequest() { Wire1.write('K'); }

// ----- WiFi & MQTT connect helpers -----
void connectWiFi() {
  
  int status = WL_IDLE_STATUS;
  sslClient.setCACert(caCert);          // <-- IMPORTANT
  while (status != WL_CONNECTED) {
    Serial.print("WiFi: connecting to "); Serial.println(WIFI_SSID);
    status = WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(1000);
  }
  Serial.print("WiFi OK, IP="); Serial.println(WiFi.localIP());
}

void connectMQTT() {
  // attach CA so the client can verify the broker
  sslClient.setCACert(caCert);
  Serial.print("caCert length = "); Serial.println(strlen(caCert));
  Serial.print("FIRST 32: ");
  for (int i = 0; i < 32 && caCert[i]; i++) Serial.print(caCert[i]);
  Serial.println();
  int L = strlen(caCert);
  Serial.print("LAST 32: ");
  for (int i = max(0, L-32); i < L; i++) Serial.print(caCert[i]);
  Serial.println();

  // If your core supports it, you can also enforce SNI hostname:
  // sslClient.setSNIHostname(BROKER_HOST);

  // creds + a simple unique client id
  mqttClient.setUsernamePassword(MQTT_USER, MQTT_PASS);
  mqttClient.setId(String("unoR4-") + String((uint32_t)millis(), HEX));

  Serial.print("MQTT (TLS) connecting to ");
  Serial.print(BROKER_HOST); Serial.print(":"); Serial.println(BROKER_PORT);
  // sslClient.setInsecure(); 
  if (!mqttClient.connect(BROKER_HOST, BROKER_PORT)) {
    Serial.print("MQTT connect failed, error = ");
    Serial.println(mqttClient.connectError());
    showStatus('E');
    // For a one-time diagnostic only, you can bypass validation:
    // sslClient.setInsecure(); // <-- TEMP TEST ONLY. REMOVE after confirming CA/host.
    // while (!mqttClient.connect(BROKER_HOST, BROKER_PORT)) { delay(1000); }
    while (1) { delay(1000); } // stop here so you can read the error
  }

  Serial.println("MQTT connected.");
  showStatus('C');
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire1.begin(I2C_ADDRESS);
  Wire1.setClock(100000);
  Wire1.onReceive(onReceive);
  Wire1.onRequest(onRequest);

  matrix.begin();
  showStatus('W');

  connectWiFi();
  unsigned long now = WiFi.getTime();   // many WiFiS3 firmwares support this
  Serial.print("Epoch now="); Serial.println(now);


  showStatus('M');
  connectMQTT();
}

void loop() {
  if (!mqttClient.connected()) {
    showStatus('M');
    connectMQTT();
  }

  mqttClient.poll();

  if (haveData && millis() - lastPublish >= PUBLISH_INTERVAL_MS) {
    lastPublish = millis();

    noInterrupts();
    uint16_t snap[SENSOR_COUNT];
    for (int i = 0; i < SENSOR_COUNT; i++) snap[i] = rcvDistances[i];
    interrupts();

    mqttClient.beginMessage(PUB_TOPIC);
    mqttClient.print("{\"distances\":[");
    for (int i = 0; i < SENSOR_COUNT; i++) {
      if (i) mqttClient.print(',');
      mqttClient.print(snap[i]);
    }
    mqttClient.print("]}");
    mqttClient.endMessage();
  }
}
