/*
 * ESP32-S3 IoT Demo Firmware
 * 
 * Features:
 * - WiFi connection with auto-reconnect
 * - MQTT client with LWT (Last Will Testament)
 * - Device control via MQTT commands (Light & Fan)
 * - Sensor data publishing (Temperature, Humidity, Light level)
 * - Retained device state messages for UI synchronization
 * 
 * MQTT Topics:
 * - Publish sensor data: ${TOPIC_NS}/sensor/state
 * - Publish device state: ${TOPIC_NS}/device/state (retained)
 * - Publish online status: ${TOPIC_NS}/sys/online (retained, LWT)
 * - Subscribe commands: ${TOPIC_NS}/device/cmd
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// =============================================================================
// CONFIGURATION - Modify these values for your setup
// =============================================================================

// WiFi Configuration
const char* WIFI_SSID = "YourWiFiName";        // Change to your WiFi SSID
const char* WIFI_PASSWORD = "YourWiFiPassword";  // Change to your WiFi password

// MQTT Broker Configuration
const char* MQTT_HOST = "192.168.1.10";        // Change to your MQTT broker IP
const int MQTT_PORT = 1883;
const char* MQTT_USERNAME = "user1";           // Change to your MQTT username
const char* MQTT_PASSWORD = "pass1";           // Change to your MQTT password

// Device Configuration
const char* DEVICE_ID = "esp32_demo_001";      // Unique device identifier
const char* FIRMWARE_VERSION = "demo1-1.0.0";  // Firmware version
const char* TOPIC_NS = "lab/room1";            // Topic namespace - match with app/web

// GPIO Pin Configuration - Adjust according to your ESP32-S3 board
const int LIGHT_RELAY_PIN = 5;    // GPIO pin for light relay control
const int FAN_RELAY_PIN = 6;      // GPIO pin for fan relay control
const int STATUS_LED_PIN = 2;     // Built-in LED for status indication

// Timing Configuration
const unsigned long SENSOR_PUBLISH_INTERVAL = 3000;   // 3 seconds
const unsigned long HEARTBEAT_INTERVAL = 15000;       // 15 seconds
const unsigned long WIFI_RECONNECT_INTERVAL = 5000;   // 5 seconds
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;   // 5 seconds
const unsigned long COMMAND_DEBOUNCE_DELAY = 500;     // 500ms debounce

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Device state
bool lightState = false;
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

// =============================================================================
// SETUP FUNCTION
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== ESP32-S3 IoT Demo Starting ===");
  Serial.printf("Device ID: %s\n", DEVICE_ID);
  Serial.printf("Firmware: %s\n", FIRMWARE_VERSION);
  Serial.printf("Topic Namespace: %s\n", TOPIC_NS);
  
  // Initialize GPIO pins
  initGPIO();
  
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
  
  // Update status LED
  updateStatusLED();
  
  delay(100); // Small delay to prevent watchdog issues
}

// =============================================================================
// INITIALIZATION FUNCTIONS
// =============================================================================

void initGPIO() {
  Serial.println("Initializing GPIO pins...");
  
  // Configure relay pins as outputs
  pinMode(LIGHT_RELAY_PIN, OUTPUT);
  pinMode(FAN_RELAY_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  // Initialize relays to OFF state
  digitalWrite(LIGHT_RELAY_PIN, LOW);
  digitalWrite(FAN_RELAY_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);
  
  Serial.printf("Light relay pin: %d\n", LIGHT_RELAY_PIN);
  Serial.printf("Fan relay pin: %d\n", FAN_RELAY_PIN);
  Serial.printf("Status LED pin: %d\n", STATUS_LED_PIN);
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
    Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
  } else {
    Serial.println("\nWiFi connection failed!");
  }
}

void initMQTT() {
  Serial.printf("Setting up MQTT client for %s:%d\n", MQTT_HOST, MQTT_PORT);
  
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(onMqttMessage);
  mqttClient.setKeepAlive(30);
  mqttClient.setSocketTimeout(5);
  
  // Connect to MQTT
  connectMQTT();
}

// =============================================================================
// WIFI FUNCTIONS
// =============================================================================

void checkWiFi(unsigned long currentTime) {
  if (currentTime - lastWifiCheck < WIFI_RECONNECT_INTERVAL) return;
  lastWifiCheck = currentTime;
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("\nWiFi reconnected! IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("\nWiFi reconnection failed!");
    }
  }
}

// =============================================================================
// MQTT FUNCTIONS
// =============================================================================

void checkMQTT(unsigned long currentTime) {
  if (currentTime - lastMqttCheck < MQTT_RECONNECT_INTERVAL) return;
  lastMqttCheck = currentTime;
  
  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
    Serial.println("MQTT disconnected. Reconnecting...");
    connectMQTT();
  }
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping MQTT connection");
    return;
  }
  
  Serial.printf("Connecting to MQTT broker: %s:%d\n", MQTT_HOST, MQTT_PORT);
  
  // Set Last Will Testament (LWT)
  String lwt = "{\"online\":false}";
  
  bool connected = mqttClient.connect(
    DEVICE_ID,
    MQTT_USERNAME,
    MQTT_PASSWORD,
    topicSysOnline.c_str(),
    1,              // QoS
    true,           // Retain
    lwt.c_str()     // LWT message
  );
  
  if (connected) {
    Serial.println("MQTT connected successfully!");
    
    // Subscribe to command topic
    if (mqttClient.subscribe(topicDeviceCmd.c_str(), 1)) {
      Serial.printf("Subscribed to: %s\n", topicDeviceCmd.c_str());
    } else {
      Serial.println("Failed to subscribe to command topic!");
    }
    
    // Publish online status
    publishOnlineStatus(true);
    
    // Publish initial device state
    publishDeviceState();
    
    deviceOnline = true;
  } else {
    Serial.printf("MQTT connection failed! State: %d\n", mqttClient.state());
    deviceOnline = false;
  }
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  // Convert payload to string
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.printf("Received [%s]: %s\n", topic, message.c_str());
  
  // Check if it's a command message
  if (String(topic) == topicDeviceCmd) {
    handleDeviceCommand(message);
  }
}

void handleDeviceCommand(String message) {
  // Debounce commands to prevent rapid switching
  unsigned long currentTime = millis();
  if (currentTime - lastCommandTime < COMMAND_DEBOUNCE_DELAY) {
    Serial.println("Command ignored due to debounce");
    return;
  }
  lastCommandTime = currentTime;
  
  // Parse JSON command
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.printf("JSON parse error: %s\n", error.c_str());
    return;
  }
  
  bool stateChanged = false;
  
  // Handle light command
  if (doc.containsKey("light")) {
    String lightCmd = doc["light"].as<String>();
    Serial.printf("Light command: %s\n", lightCmd.c_str());
    
    if (lightCmd == "on") {
      lightState = true;
      stateChanged = true;
    } else if (lightCmd == "off") {
      lightState = false;
      stateChanged = true;
    } else if (lightCmd == "toggle") {
      lightState = !lightState;
      stateChanged = true;
    }
    
    digitalWrite(LIGHT_RELAY_PIN, lightState ? HIGH : LOW);
    Serial.printf("Light turned %s\n", lightState ? "ON" : "OFF");
  }
  
  // Handle fan command
  if (doc.containsKey("fan")) {
    String fanCmd = doc["fan"].as<String>();
    Serial.printf("Fan command: %s\n", fanCmd.c_str());
    
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
  
  // Immediately publish device state after command execution
  if (stateChanged) {
    publishDeviceState();
  }
}

// =============================================================================
// PUBLISHING FUNCTIONS
// =============================================================================

void publishSensorData() {
  if (!mqttClient.connected()) return;
  
  // Generate fake sensor data (replace with real sensor readings)
  float temperature = 20.0 + random(-50, 100) / 10.0;  // 15.0 to 25.0°C
  float humidity = 50.0 + random(-200, 200) / 10.0;    // 30.0 to 70.0%
  int lightLevel = 100 + random(-50, 200);             // 50 to 300 lux
  
  // Create JSON payload
  JsonDocument doc;
  doc["ts"] = WiFi.getTime();
  doc["temp_c"] = round(temperature * 10) / 10.0;  // Round to 1 decimal
  doc["hum_pct"] = round(humidity * 10) / 10.0;
  doc["lux"] = lightLevel;
  
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
  
  // Create JSON payload
  JsonDocument doc;
  doc["ts"] = WiFi.getTime();
  doc["light"] = lightState ? "on" : "off";
  doc["fan"] = fanState ? "on" : "off";
  doc["rssi"] = WiFi.RSSI();
  doc["fw"] = FIRMWARE_VERSION;
  
  String payload;
  serializeJson(doc, payload);
  
  // Publish with retain flag for UI synchronization
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
  
  // Publish with retain flag
  if (mqttClient.publish(topicSysOnline.c_str(), payload.c_str(), true)) {
    Serial.printf("Online status published: %s\n", payload.c_str());
  } else {
    Serial.println("Failed to publish online status!");
  }
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

void updateStatusLED() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  unsigned long currentTime = millis();
  
  if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    // Solid ON when everything is connected
    digitalWrite(STATUS_LED_PIN, HIGH);
  } else if (WiFi.status() == WL_CONNECTED) {
    // Fast blink when WiFi connected but MQTT disconnected
    if (currentTime - lastBlink >= 250) {
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState ? HIGH : LOW);
      lastBlink = currentTime;
    }
  } else {
    // Slow blink when WiFi disconnected
    if (currentTime - lastBlink >= 1000) {
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState ? HIGH : LOW);
      lastBlink = currentTime;
    }
  }
}