/*
 * ESP32-C3 IoT Demo Firmware
 * **PHIÊN BẢN CUỐI CÙNG**
 *
 * Chức năng:
 * - Điều khiển Bật/Tắt motor đơn giản qua L298N (dùng IN1, IN2).
 * - Giả định Jumper ENA và ENB đã được cắm (hoặc nối dây thay thế).
 * - Cấp nguồn cho L298N từ chân 5V của ESP32 (không khuyến khích, có thể gây reset).
 * - Tích hợp cảm biến DHT22 và điều khiển LED.
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHTesp.h>

// =============================================================================
// CONFIGURATION - KIỂM TRA LẠI CÁC THÔNG SỐ NÀY
// =============================================================================

// WiFi Configuration
const char* WIFI_SSID = "Hoi Coffee 1"; // <<-- THAY TÊN WIFI CỦA BẠN
const char* WIFI_PASSWORD = "12356789"; // <<-- THAY MẬT KHẨU WIFI

// MQTT Broker Configuration
const char* MQTT_HOST = "broker.hivemq.com";
const int MQTT_PORT = 1883;
// === SỬA LỖI: Thêm lại các dòng khai báo bị thiếu ===
const char* MQTT_USERNAME = "";
const char* MQTT_PASSWORD = "";

// Device Configuration
const char* DEVICE_ID = "esp32c3_demo_001";
const char* FIRMWARE_VERSION = "demo-c3-l298n-final-1.0";
const char* TOPIC_NS = "lab/test_esp32"; // <<-- ĐẢM BẢO KHỚP VỚI WEB/APP

// === Cấu hình chân GPIO cho L298N (Bật/Tắt) ===
const int LED_PIN = 2;       // Chân GPIO cho LED
const int DHT_PIN = 4;       // Chân GPIO cho DHT22
const int FAN_IN1_PIN = 6;   // Chân IN1 của L298N
const int FAN_IN2_PIN = 7;   // Chân IN2 của L298N

// Timing Configuration
const unsigned long SENSOR_PUBLISH_INTERVAL = 5000;
const unsigned long HEARTBEAT_INTERVAL = 15000;
const unsigned long WIFI_RECONNECT_INTERVAL = 5000;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;
const unsigned long COMMAND_DEBOUNCE_DELAY = 500;

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

WiFiClient espClient;
PubSubClient mqttClient(espClient);
DHTesp dhtSensor;

// Device state
bool ledState = false;
bool fanState = false;

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

// Forward declarations
void connectMQTT();
void onMqttMessage(char* topic, byte* payload, unsigned int length);
void publishDeviceState();
void handleDeviceCommand(String message);
void initGPIO();
void initSensor();
void initTopics();
void initWiFi();
void initMQTT();
void checkWiFi(unsigned long currentTime);
void checkMQTT(unsigned long currentTime);
void publishSensorData();
void publishOnlineStatus(bool online);


// =============================================================================
// SETUP FUNCTION
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== L298N Simple On/Off Demo Starting ===");
    Serial.printf("Device ID: %s\n", DEVICE_ID);
    Serial.printf("Firmware: %s\n", FIRMWARE_VERSION);
    Serial.printf("Topic Namespace: %s\n", TOPIC_NS);
    
    initGPIO();
    initSensor();
    initTopics();
    initWiFi();
    initMQTT();
    
    Serial.println("=== Setup Complete ===\n");
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
    unsigned long currentTime = millis();
    
    checkWiFi(currentTime);
    checkMQTT(currentTime);
    
    if (mqttClient.connected()) {
        mqttClient.loop();
        
        if (currentTime - lastSensorPublish >= SENSOR_PUBLISH_INTERVAL) {
            publishSensorData();
            lastSensorPublish = currentTime;
        }
        
        if (currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL) {
            publishDeviceState();
            lastHeartbeat = currentTime;
        }
    }
    
    delay(10);
}

// =============================================================================
// INITIALIZATION FUNCTIONS
// =============================================================================

void initGPIO() {
    Serial.println("Initializing GPIO pins...");
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    Serial.printf("LED pin: %d\n", LED_PIN);

    // Cấu hình 2 chân cho L298N
    pinMode(FAN_IN1_PIN, OUTPUT);
    pinMode(FAN_IN2_PIN, OUTPUT);
    
    // Đảm bảo motor tắt khi khởi động (trạng thái phanh)
    digitalWrite(FAN_IN1_PIN, LOW);
    digitalWrite(FAN_IN2_PIN, LOW);

    Serial.printf("Fan Control Pins: IN1=%d, IN2=%d\n", FAN_IN1_PIN, FAN_IN2_PIN);
}

void initSensor() {
    Serial.println("Initializing DHT22 sensor...");
    dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
}

void initTopics() {
    Serial.println("Initializing MQTT topics...");
    topicSensorState = String(TOPIC_NS) + "/sensor/state";
    topicDeviceState = String(TOPIC_NS) + "/device/state";
    topicDeviceCmd = String(TOPIC_NS) + "/device/cmd";
    topicSysOnline = String(TOPIC_NS) + "/sys/online";
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
    if (WiFi.status() != WL_CONNECTED) return;
    
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
    } else {
        Serial.printf("MQTT connection failed! State: %d\n", mqttClient.state());
    }
}

// =============================================================================
// MQTT MESSAGE HANDLING & PUBLISHING
// =============================================================================

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    String message;
    message.reserve(length);
    for (unsigned int i = 0; i < length; i++) {
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
    
    if (doc.containsKey("light")) {
        String lightCmd = doc["light"].as<String>();
        if (lightCmd == "toggle") {
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState ? HIGH : LOW);
            Serial.printf("LED turned %s\n", ledState ? "ON" : "OFF");
            stateChanged = true;
        }
    }
    
    if (doc.containsKey("fan")) {
        String fanCmd = doc["fan"].as<String>();
        if (fanCmd == "toggle") {
            fanState = !fanState;
            stateChanged = true;

            if (fanState) { // Nếu lệnh là BẬT quạt
                Serial.println("Received Fan ON command");
                digitalWrite(FAN_IN1_PIN, HIGH); // Quay thuận
                digitalWrite(FAN_IN2_PIN, LOW);
            } else { // Nếu lệnh là TẮT quạt
                Serial.println("Received Fan OFF command");
                digitalWrite(FAN_IN1_PIN, LOW); // Phanh động cơ
                digitalWrite(FAN_IN2_PIN, LOW);
            }
        }
    }
    
    if (stateChanged) {
        publishDeviceState();
    }
}

void publishSensorData() {
    if (!mqttClient.connected()) return;
    
    TempAndHumidity data = dhtSensor.getTempAndHumidity();
    
    if (isnan(data.temperature) || isnan(data.humidity)) {
        Serial.println("Error reading from DHT sensor!");
        return;
    }

    JsonDocument doc;
    doc["temp_c"] = round(data.temperature * 10) / 10.0;
    doc["hum_pct"] = round(data.humidity * 10) / 10.0;
    
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
    doc["light"] = ledState ? "on" : "off";
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