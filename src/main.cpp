#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include "secrets.h" // Include Wi-Fi credentials from secrets.h


// -------------------- OLED --------------------
#define SDA_PIN 18
#define SCL_PIN 17
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);


AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// -------------------- Noise Sensor --------------------
#define UltraRX 42
#define UltraTx 46
#define noise_Sensor_BAUD 9600

HardwareSerial SerialNoise(2);
const byte COMMAND_READ_NOISE[] = {0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A};

// MQTT topics
#define DEVICE_ID "ESP32NoiseSensor"
#define STATE_TOPIC "homeassistant/sensor/ESP32NoiseSensor-NoiseLevel_dB/state"
#define CONFIG_TOPIC "homeassistant/sensor/ESP32NoiseSensor-NoiseLevel_dB/config"

#define RSSI_TOPIC "homeassistant/sensor/ESP32NoiseSensor-RSSI/state"
#define RSSI_CONFIG_TOPIC "homeassistant/sensor/ESP32NoiseSensor-RSSI/config"


bool discoverySent = false;
unsigned long startTime = 0;

// -------------------- Wi-Fi & MQTT functions --------------------
void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.print("WiFi connected. IP: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi disconnected, retrying...");
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
    default:
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT!");
  discoverySent = false;
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) xTimerStart(mqttReconnectTimer, 0);
}

void sendDiscovery() {
  if (discoverySent) return;

  String payload = R"({
    "name": "Noise Level",
    "uniq_id": "ESP32NoiseSensor-NoiseLevel_dB",
    "stat_t": "homeassistant/sensor/ESP32NoiseSensor-NoiseLevel_dB/state",
    "dev_cla": "sound_pressure",
    "unit_of_meas": "dB",
    "val_tpl": "{{ value_json.value | float }}",
    "stat_cla": "measurement",
    "device": {
      "ids": ["ESP32NoiseSensor"],
      "mdl": "KE-ZS-BZ-TTL-05",
      "name": "ESP32 Noise Sensor",
      "via_device": "ESP32"
    }
  })";

  uint16_t packetId = mqttClient.publish(CONFIG_TOPIC, 1, true, payload.c_str());
  if (packetId > 0) {
    Serial.println("✅ Discovery payload sent to MQTT (retained).");
    discoverySent = true;
  } else {
    Serial.println("⚠️ Failed to send discovery payload!");
  }
}

void sendRSSIDiscovery() {
  String payload = R"({
    "name": "WiFi RSSI",
    "uniq_id": "ESP32NoiseSensor-RSSI",
    "stat_t": "homeassistant/sensor/ESP32NoiseSensor-RSSI/state",
    "dev_cla": "signal_strength",
    "unit_of_meas": "dBm",
    "val_tpl": "{{ value_json.rssi }}",
    "device": {
      "ids": ["ESP32NoiseSensor"],
      "mdl": "KE-ZS-BZ-TTL-05",
      "name": "ESP32 Noise Sensor",
      "via_device": "ESP32"
    }
  })";

  uint16_t packetId = mqttClient.publish(RSSI_CONFIG_TOPIC, 1, true, payload.c_str());
  if (packetId > 0) {
    Serial.println("✅ RSSI discovery payload sent to MQTT (retained).");
  } else {
    Serial.println("⚠️ Failed to send RSSI discovery payload!");
  }
}

// -------------------- Noise reading --------------------
float getNoiseLevel() {
  uint8_t input[10];
  int count = 0;

  SerialNoise.write(COMMAND_READ_NOISE, sizeof(COMMAND_READ_NOISE));
  delay(10);

  while (SerialNoise.available() > 0 && count < sizeof(input)) {
    input[count++] = SerialNoise.read();
  }

  if (count >= 7 && input[0]==0x01 && input[1]==0x03 && input[2]==0x02) {
    uint16_t val = (input[3]<<8)|input[4];
    return val / 10.0;
  }
  return -1;
}

float getNoiseLevelDiagnostic() {
  uint8_t input[16];
  int count = 0;

  // Send Modbus RTU request
  SerialNoise.write(COMMAND_READ_NOISE, sizeof(COMMAND_READ_NOISE));
  delay(10);

  // Read response
  while (SerialNoise.available() > 0 && count < sizeof(input)) {
    input[count++] = SerialNoise.read();
  }

  // Print raw bytes in hex for analysis
  Serial.print("RAW: ");
  for (int i = 0; i < count; i++) {
    if (input[i] < 0x10) Serial.print('0');
    Serial.print(input[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  // Check response length and header
  if (count >= 7 && input[0] == 0x01 && input[1] == 0x03 && input[2] == 0x02) {
    uint16_t val = (input[3] << 8) | input[4];

    // Compute CRC16 (Modbus)
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < count - 2; pos++) {
      crc ^= (uint16_t)input[pos];
      for (int i = 0; i < 8; i++) {
        if (crc & 0x0001)
          crc = (crc >> 1) ^ 0xA001;
        else
          crc >>= 1;
      }
    }

    uint16_t crcReceived = (input[count - 1] << 8) | input[count - 2];
    if (crc == crcReceived) {
      float db = val / 10.0;
      Serial.printf("Parsed: %.1f dB (CRC OK)\n", db);
      return db;
    } else {
      Serial.printf("⚠️ CRC mismatch: calc=0x%04X recv=0x%04X\n", crc, crcReceived);
      return -1;
    }
  }

  Serial.println("⚠️ Invalid frame");
  return -1;
}


// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);       // initialize I2C for OLED
  u8g2.begin();
  SerialNoise.begin(noise_Sensor_BAUD, SERIAL_8N1, UltraRX, UltraTx);

  Serial.println("Setup done. Starting...");

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, [](TimerHandle_t xTimer){ connectToMqtt(); });
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, [](TimerHandle_t xTimer){ connectToWifi(); });

  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();

  startTime = millis(); // start counting 3 minutes
}

// Display Wi-Fi RSSI on OLED
void displayWiFiRSSI() {
    char buf[32];
    snprintf(buf, sizeof(buf), "WiFi RSSI: %d dBm", WiFi.RSSI());
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 55, buf);  // place below noise value
}

// Optional: print to serial
void printWiFiRSSI() {
    Serial.printf("Wi-Fi RSSI: %d dBm\n", WiFi.RSSI());
}

// -------------------- Loop --------------------
void loop() {
  static unsigned long lastPublish = 0;
  static unsigned long lastDiscovery = 0;
  int rssi = WiFi.RSSI();  // Get Wi-Fi RSSI in dBm

  // read noise
  float noise = getNoiseLevelDiagnostic();
  if (noise >= 0) {
    // Display on OLED for first 3 minutes
    if (millis() - startTime < 3UL * 60UL * 1000UL) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_fub25_tr);
      char buf[32];
      snprintf(buf, sizeof(buf), "%.1f dB", noise);
      u8g2.drawStr(0, 36, buf);
      displayWiFiRSSI();
      u8g2.sendBuffer();
    } else {
        // After 3 minutes, turn off the screen
        u8g2.clearBuffer();
        u8g2.setPowerSave(1); // sleep mode
    }


    // Publish MQTT every 2 seconds
    if (millis() - lastPublish > 2000) {
      char payload[64];
      snprintf(payload, sizeof(payload), "{\"value\": %.1f}", noise);
      mqttClient.publish(STATE_TOPIC, 1, true, payload);
      Serial.printf("Published noise value: %.1f dB\n", noise);

      // Publish RSSI as diagnostic value
      char rssiPayload[32];
      snprintf(rssiPayload, sizeof(rssiPayload), "{\"rssi\": %d}", rssi);
      mqttClient.publish(RSSI_TOPIC, 1, true, rssiPayload);
      Serial.printf("Published Wi-Fi RSSI: %d dBm\n", rssi);

      lastPublish = millis();
    }
  }

  // MQTT discovery every 10s
  if (mqttClient.connected() && !discoverySent && millis() - lastDiscovery > 10000) {
    sendDiscovery();
    sendRSSIDiscovery();
    lastDiscovery = millis();
  }

  delay(500);
}
