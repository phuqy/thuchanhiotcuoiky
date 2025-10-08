/*
 * ESP32-C3 IoT Demo Firmware
 * * Modifications for ESP32-C3 Mini:
 * - Added DHT22 sensor library and integration.
 * - Updated GPIO pin definitions to match your wiring.
 * - Sensor data is now read from the actual DHT22 sensor.
 * - Renamed "Light" to "LED" for clarity.
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHTesp.h> // === THAY ĐỔI 1: Thêm thư viện cho cảm biến DHT ===

// =============================================================================
// CONFIGURATION - THAY ĐỔI CÁC THÔNG SỐ NÀY
// =============================================================================

// WiFi Configuration
const char* WIFI_SSID = "Redmi Note 13 5G";      // Tên WiFi của bạn
const char* WIFI_PASSWORD = "01234557";          // Mật khẩu WiFi

// MQTT Broker Configuration
const char* MQTT_HOST = "10.183.164.241";        // IP của MQTT broker
const int MQTT_PORT = 1883;
const char* MQTT_USERNAME = "";                  // Tên đăng nhập MQTT (nếu có)
const char* MQTT_PASSWORD = "";                  // Mật khẩu MQTT (nếu có)

// Device Configuration
const char* DEVICE_ID = "esp32c3_demo_001";      // Tên định danh duy nhất cho thiết bị
const char* FIRMWARE_VERSION = "demo-c3-1.0.1";  // Phiên bản firmware
const char* TOPIC_NS = "lab/room1";              // Namespace - phải giống với trong app

// === THAY ĐỔI 2: Cập nhật cấu hình chân GPIO cho ESP32-C3 Mini ===
const int LED_PIN = 2;          // Chân GPIO nối với đèn LED
const int FAN_RELAY_PIN = 5;    // Chân GPIO nối với Relay của quạt
const int DHT_PIN = 4;          // Chân GPIO nối với cảm biến DHT22

// Timing Configuration
const unsigned long SENSOR_PUBLISH_INTERVAL = 5000;   // Gửi dữ liệu cảm biến mỗi 5 giây
const unsigned long HEARTBEAT_INTERVAL = 15000;       // Gửi trạng thái thiết bị mỗi 15 giây
const unsigned long WIFI_RECONNECT_INTERVAL = 5000;   // Thử kết nối lại WiFi mỗi 5 giây
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;   // Thử kết nối lại MQTT mỗi 5 giây
const unsigned long COMMAND_DEBOUNCE_DELAY = 500;     // Chống dội cho lệnh, 500ms

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

WiFiClient espClient;
PubSubClient mqttClient(espClient);
DHTesp dhtSensor; // Tạo đối tượng cảm biến DHT

// Device state
bool ledState = false; // Đổi tên từ lightState
bool fanState = false;
bool deviceOnline = false;

// Timing variables
unsigned long lastSensorPublish = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastMqttCheck = 0;
unsigned long lastCommandTime = 0;

// MQTT Topics
String topicSensorState;
String topicDeviceState;
String topicDeviceCmd;
String topicSysOnline;

// Forward declarations of functions to avoid compilation errors
void connectMQTT();
void onMqttMessage(char* topic, byte* payload, unsigned int length);
void publishDeviceState();

// =============================================================================
// SETUP FUNCTION
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== ESP32-C3 IoT Demo Starting ===");
  Serial.printf("Device ID: %s\n", DEVICE_ID);
  Serial.printf("Firmware: %s\n", FIRMWARE_VERSION);
  Serial.printf("Topic Namespace: %s\n", TOPIC_NS);
  
  // Initialize GPIO pins
  initGPIO();

  // === THAY ĐỔI 3: Khởi tạo cảm biến DHT22 ===
  initSensor();
  
  // Initialize MQTT topics
  initTopics();
  
  // Initialize WiFi
  initWiFi();
  
  // Initialize MQTT
  initMQTT();
  
  Serial.println("=== Setup Complete ===\n");
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
  unsigned long currentTime = millis();
  
  // Check WiFi connection
  checkWiFi(currentTime);
  
  // Check MQTT connection
  checkMQTT(currentTime);
  
  // Handle MQTT messages
  if (mqttClient.connected()) {
    mqttClient.loop();
    
    // Publish sensor data
    if (currentTime - lastSensorPublish >= SENSOR_PUBLISH_INTERVAL) {
      publishSensorData();
      lastSensorPublish = currentTime;
    }
    
    // Publish heartbeat (device state)
    if (currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL) {
      publishDeviceState();
      lastHeartbeat = currentTime;
    }
  }
  
  delay(10); // Small delay for stability
}

// =============================================================================
// INITIALIZATION FUNCTIONS
// =============================================================================

void initGPIO() {
  Serial.println("Initializing GPIO pins...");
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(FAN_RELAY_PIN, OUTPUT);
  
  digitalWrite(LED_PIN, LOW);
  digitalWrite(FAN_RELAY_PIN, LOW);
  
  Serial.printf("LED pin: %d\n", LED_PIN);
  Serial.printf("Fan relay pin: %d\n", FAN_RELAY_PIN);
}

void initSensor() {
    Serial.println("Initializing DHT22 sensor...");
    dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
    Serial.printf("DHT22 sensor ready, sample time: %dms\n", dhtSensor.getMinimumSamplingPeriod());
}

void initTopics() {
  Serial.println("Initializing MQTT topics...");
  
  topicSensorState = String(TOPIC_NS) + "/sensor/state";
  topicDeviceState = String(TOPIC_NS) + "/device/state";
  topicDeviceCmd = String(TOPIC_NS) + "/device/cmd";
  topicSysOnline = String(TOPIC_NS) + "/sys/online";
  
  Serial.printf("Sensor topic: %s\n", topicSensorState.c_str());
  Serial.printf("Device state topic: %s\n", topicDeviceState.c_str());
  Serial.printf("Command topic: %s\n", topicDeviceCmd.c_str());
  Serial.printf("Online topic: %s\n", topicSysOnline.c_str());
}

void initWiFi() {
  Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWiFi connection failed!");
  }
}

void initMQTT() {
  Serial.printf("Setting up MQTT client for %s:%d\n", MQTT_HOST, MQTT_PORT);
  
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(onMqttMessage);
  
  connectMQTT();
}

// =============================================================================
// WIFI & MQTT CONNECTION MANAGEMENT
// =============================================================================

void checkWiFi(unsigned long currentTime) {
  if (WiFi.status() != WL_CONNECTED && currentTime - lastWifiCheck >= WIFI_RECONNECT_INTERVAL) {
    Serial.println("WiFi disconnected. Reconnecting...");
    WiFi.reconnect();
    lastWifiCheck = currentTime;
  }
}

void checkMQTT(unsigned long currentTime) {
  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected() && currentTime - lastMqttCheck >= MQTT_RECONNECT_INTERVAL) {
    Serial.println("MQTT disconnected. Reconnecting...");
    connectMQTT();
    lastMqttCheck = currentTime;
  }
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping MQTT connection");
    return;
  }
  
  Serial.printf("Connecting to MQTT broker: %s:%d\n", MQTT_HOST, MQTT_PORT);
  
  String lwt = "{\"online\":false}";
  
  if (mqttClient.connect(DEVICE_ID, MQTT_USERNAME, MQTT_PASSWORD, topicSysOnline.c_str(), 1, true, lwt.c_str())) {
    Serial.println("MQTT connected successfully!");
    
    if (mqttClient.subscribe(topicDeviceCmd.c_str(), 1)) {
      Serial.printf("Subscribed to: %s\n", topicDeviceCmd.c_str());
    } else {
      Serial.println("Failed to subscribe to command topic!");
    }
    
    publishOnlineStatus(true);
    publishDeviceState();
    deviceOnline = true;
  } else {
    Serial.printf("MQTT connection failed! State: %d\n", mqttClient.state());
    deviceOnline = false;
  }
}

// =============================================================================
// MQTT MESSAGE HANDLING & PUBLISHING
// =============================================================================

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  String message;
  message.reserve(length);
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.printf("Received [%s]: %s\n", topic, message.c_str());
  
  if (String(topic) == topicDeviceCmd) {
    handleDeviceCommand(message);
  }
}

void handleDeviceCommand(String message) {
  unsigned long currentTime = millis();
  if (currentTime - lastCommandTime < COMMAND_DEBOUNCE_DELAY) {
    Serial.println("Command ignored due to debounce");
    return;
  }
  lastCommandTime = currentTime;
  
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.printf("JSON parse error: %s\n", error.c_str());
    return;
  }
  
  bool stateChanged = false;
  
  if (doc.containsKey("led")) { // Changed from "light" to "led" for consistency
    String ledCmd = doc["led"].as<String>();
    
    if (ledCmd == "on") {
      ledState = true;
      stateChanged = true;
    } else if (ledCmd == "off") {
      ledState = false;
      stateChanged = true;
    } else if (ledCmd == "toggle") {
      ledState = !ledState;
      stateChanged = true;
    }
    
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    Serial.printf("LED turned %s\n", ledState ? "ON" : "OFF");
  }
  
  if (doc.containsKey("fan")) {
    String fanCmd = doc["fan"].as<String>();
    
    if (fanCmd == "on") {
      fanState = true;
      stateChanged = true;
    } else if (fanCmd == "off") {
      fanState = false;
      stateChanged = true;
    } else if (fanCmd == "toggle") {
      fanState = !fanState;
      stateChanged = true;
    }
    
    digitalWrite(FAN_RELAY_PIN, fanState ? HIGH : LOW);
    Serial.printf("Fan turned %s\n", fanState ? "ON" : "OFF");
  }
  
  if (stateChanged) {
    publishDeviceState();
  }
}


void publishSensorData() {
  if (!mqttClient.connected()) return;
  
  // === THAY ĐỔI 4: Đọc dữ liệu thật từ cảm biến DHT22 ===
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  
  // Kiểm tra xem đọc dữ liệu có thành công không
  if (isnan(data.temperature) || isnan(data.humidity)) {
    Serial.println("Error reading from DHT sensor!");
    return;
  }

  float temperature = data.temperature;
  float humidity = data.humidity;
  
  JsonDocument doc;
  // doc["ts"] = WiFi.getTime(); // This requires NTP setup, simplifying for now
  doc["temp_c"] = round(temperature * 10) / 10.0;
  doc["hum_pct"] = round(humidity * 10) / 10.0;
  
  String payload;
  serializeJson(doc, payload);
  
  if (mqttClient.publish(topicSensorState.c_str(), payload.c_str(), false)) {
    Serial.printf("Sensor data published: %s\n", payload.c_str());
  } else {
    Serial.println("Failed to publish sensor data!");
  }
}

void publishDeviceState() {
  if (!mqttClient.connected()) return;
  
  JsonDocument doc;
  // doc["ts"] = WiFi.getTime();
  doc["led"] = ledState ? "on" : "off"; // Changed from "light"
  doc["fan"] = fanState ? "on" : "off";
  doc["rssi"] = WiFi.RSSI();
  doc["fw"] = FIRMWARE_VERSION;
  
  String payload;
  serializeJson(doc, payload);
  
  if (mqttClient.publish(topicDeviceState.c_str(), payload.c_str(), true)) {
    Serial.printf("Device state published: %s\n", payload.c_str());
  } else {
    Serial.println("Failed to publish device state!");
  }
}

void publishOnlineStatus(bool online) {
  if (!mqttClient.connected()) return;
  
  JsonDocument doc;
  doc["online"] = online;
  
  String payload;
  serializeJson(doc, payload);
  
  if (mqttClient.publish(topicSysOnline.c_str(), payload.c_str(), true)) {
    Serial.printf("Online status published: %s\n", payload.c_str());
  } else {
    Serial.println("Failed to publish online status!");
  }
}
