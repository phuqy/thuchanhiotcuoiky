# ESP32-S3 IoT Device Firmware

## 📋 **Overview**

Firmware cho ESP32-S3 development board để tạo một IoT device hoàn chỉnh với khả năng:
- **WiFi connectivity** với auto-reconnect
- **MQTT communication** với Last Will Testament (LWT)
- **Sensor data collection** (Temperature, Humidity, Light)
- **Device control** (Light, Fan relays) via MQTT commands
- **Real-time synchronization** với Web Dashboard và Flutter App
- **Status LED indicators** cho connection states
- **Retained messages** cho UI synchronization

---

## 🛠️ **Hardware Requirements**

### 📦 **Components List**
| Component | Quantity | Description |
|-----------|----------|-------------|
| ESP32-S3 Dev Board | 1 | Main microcontroller |
| DHT22 | 1 | Temperature & Humidity sensor |
| BH1750 | 1 | Light intensity sensor |
| Relay Module 2-channel | 1 | Device control (Light & Fan) |
| LED (3mm/5mm) | 2 | Status indicators |
| Resistor 220Ω | 2 | LED current limiting |
| Breadboard | 1 | Prototyping |
| Jumper Wires | 20+ | Connections |

### ⚡ **Pin Configuration**\n\n```cpp\n// Sensor Pins\n#define DHT_PIN 4          // DHT22 data pin\n#define I2C_SDA 21         // BH1750 SDA pin  \n#define I2C_SCL 22         // BH1750 SCL pin\n\n// Control Pins\n#define LIGHT_RELAY_PIN 18 // Light relay control\n#define FAN_RELAY_PIN 19   // Fan relay control\n#define STATUS_LED_PIN 2   // Status LED\n#define WIFI_LED_PIN 15    // WiFi status LED (optional)\n```\n\n---\n\n## 🔌 **Wiring Diagram**\n\n### 📀 **Complete Connections**\n\n```\nESP32-S3 Development Board\n┌─────────────────────────────┐\n│  ┌─────┐ ┌─────┐ ┌─────┐   │\n│  │ USB │ │ RST │ │ BOOT│   │\n│  └─────┘ └─────┘ └─────┘   │\n├─────────────────────────────┤\n│ 3V3  ●●● GND               │ → DHT22 (VCC, GND)\n│ RST  ●●● GPIO4             │ → DHT22 (DATA)\n│ GPIO15●●● GPIO21           │ → BH1750 (SDA)  \n│ GPIO2 ●●● GPIO22           │ → BH1750 (SCL)\n│ GPIO0 ●●● GPIO18           │ → Relay1 (IN1) - Light\n│ GPIO1 ●●● GPIO19           │ → Relay2 (IN2) - Fan\n│ GPIO3 ●●● 5V               │ → Relay Module (VCC)\n│ GND   ●●● GND              │ → Relay Module (GND)\n└─────────────────────────────┘\n```\n\n### 🌡️ **DHT22 Sensor Connection**\n```\nDHT22 Temperature/Humidity Sensor\n┌─────────────┐\n│  ┌─┐ ┌─┐    │\n│  │1│ │2│    │ 1: VCC → ESP32 3V3\n│  └─┘ └─┘    │ 2: DATA → ESP32 GPIO4  \n│  ┌─┐ ┌─┐    │ 3: NC (not connected)\n│  │3│ │4│    │ 4: GND → ESP32 GND\n│  └─┘ └─┘    │\n└─────────────┘\n```\n\n### 💡 **BH1750 Light Sensor (I2C)**\n```\nBH1750 Light Sensor\n┌─────────────┐\n│ VCC  SDA    │ VCC → ESP32 3V3\n│ GND  SCL    │ GND → ESP32 GND\n│     ADD     │ SDA → ESP32 GPIO21  \n└─────────────┘ SCL → ESP32 GPIO22\n                ADD → GND (address 0x23)\n```
- Any ESP32-S3 development board (ESP32-S3-DevKitC-1, etc.)
- USB-C cable for programming and power

### Optional Components
- 2x Relay modules (5V or 3.3V) for controlling AC devices
- Temperature/Humidity sensor (DHT22, SHT30, etc.)
- Light sensor (BH1750, photoresistor, etc.)
- Breadboard and jumper wires
- External power supply if controlling high-power devices

### GPIO Pin Assignment (Default)
- **GPIO 5**: Light relay control
- **GPIO 6**: Fan relay control  
- **GPIO 2**: Status LED (built-in on most boards)

*Note: Adjust pin assignments in code based on your specific board and wiring*

## Software Requirements

### Arduino IDE Setup
1. **Install ESP32 Board Package:**
   - Open Arduino IDE
   - Go to File > Preferences
   - Add to Additional Board Manager URLs:
     ```
     https://espressif.github.io/arduino-esp32/package_esp32_index.json
     ```
   - Go to Tools > Board > Boards Manager
   - Search "ESP32" and install "esp32 by Espressif Systems"

2. **Install Required Libraries:**
   - Go to Tools > Manage Libraries
   - Install these libraries:
     - `PubSubClient` by Nick O'Leary
     - `ArduinoJson` by Benoit Blanchon (v7.x)

### PlatformIO Setup (Alternative)
Create `platformio.ini`:
```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
lib_deps = 
    knolleary/PubSubClient@^2.8
    bblanchon/ArduinoJson@^7.0.0
monitor_speed = 115200
```

## Configuration

### 1. WiFi Settings
Edit these lines in `main.cpp`:
```cpp
const char* WIFI_SSID = "YourWiFiName";        // Your WiFi network name
const char* WIFI_PASSWORD = "YourWiFiPassword"; // Your WiFi password
```

### 2. MQTT Broker Settings
```cpp
const char* MQTT_HOST = "192.168.1.10";        // Your MQTT broker IP
const int MQTT_PORT = 1883;                    // MQTT port (usually 1883)
const char* MQTT_USERNAME = "user1";           // MQTT username
const char* MQTT_PASSWORD = "pass1";           // MQTT password
```

### 3. Device Settings
```cpp
const char* DEVICE_ID = "esp32_demo_001";      // Unique device ID
const char* FIRMWARE_VERSION = "demo1-1.0.0";  // Version identifier
const char* TOPIC_NS = "lab/room1";            // MQTT topic namespace
```

### 4. GPIO Pin Configuration
Adjust these based on your wiring:
```cpp
const int LIGHT_RELAY_PIN = 5;    // GPIO pin for light relay
const int FAN_RELAY_PIN = 6;      // GPIO pin for fan relay
const int STATUS_LED_PIN = 2;     // Status LED pin
```

## Installation Steps

1. **Prepare Hardware:**
   - Connect ESP32-S3 to computer via USB-C
   - Optionally connect relay modules to control pins
   - Optionally connect sensors for real readings

2. **Configure Arduino IDE:**
   - Select Board: "ESP32S3 Dev Module"
   - Select Port: (Your ESP32's COM port)
   - Set Flash Size: "4MB (32Mb)"
   - Set Partition Scheme: "Default 4MB with spiffs"

3. **Upload Firmware:**
   - Open `main.cpp` in Arduino IDE
   - Modify configuration constants as needed
   - Click Upload button
   - Monitor Serial output at 115200 baud

## Operation

### Status LED Behavior
- **Solid ON**: WiFi and MQTT both connected (normal operation)
- **Fast Blink** (250ms): WiFi connected, MQTT disconnected
- **Slow Blink** (1000ms): WiFi disconnected

### Serial Monitor Output
Connect at 115200 baud to see:
- Startup information and configuration
- WiFi connection status and IP address
- MQTT connection status and subscriptions
- Received commands and responses
- Published sensor data and device states
- Error messages and reconnection attempts

### MQTT Topics

**Published by ESP32:**
- `${TOPIC_NS}/sensor/state` - Sensor readings every 3 seconds
  ```json
  {"ts":1695890000,"temp_c":23.5,"hum_pct":60.2,"lux":150}
  ```

- `${TOPIC_NS}/device/state` - Device status every 15 seconds + after commands (retained)
  ```json
  {"ts":1695890000,"light":"on","fan":"off","rssi":-57,"fw":"demo1-1.0.0"}
  ```

- `${TOPIC_NS}/sys/online` - Online status (retained, LWT)
  ```json
  {"online":true}
  ```

**Subscribed by ESP32:**
- `${TOPIC_NS}/device/cmd` - Control commands (QoS 1)
  ```json
  {"light":"on"}      // "on" | "off" | "toggle"
  {"fan":"toggle"}    // "on" | "off" | "toggle"
  {"light":"on","fan":"off"}  // Multiple commands
  ```

## Testing

### 1. Basic Connection Test
```bash
# Monitor device status
mosquitto_sub -h 192.168.1.10 -u user1 -P pass1 -t "lab/room1/+/+"

# Test light control
mosquitto_pub -h 192.168.1.10 -u user1 -P pass1 -t "lab/room1/device/cmd" -m '{"light":"toggle"}'

# Test fan control
mosquitto_pub -h 192.168.1.10 -u user1 -P pass1 -t "lab/room1/device/cmd" -m '{"fan":"on"}'
```

### 2. Web/App Integration Test
1. Start the web dashboard and verify sensor data appears
2. Use Flutter app to control devices
3. Verify status updates in real-time
4. Test offline/online detection by unplugging ESP32

## Troubleshooting

### Connection Issues

**WiFi won't connect:**
- Check SSID and password spelling
- Ensure ESP32 is within WiFi range
- Verify 2.4GHz network (ESP32 doesn't support 5GHz)
- Check router MAC filtering settings

**MQTT won't connect:**
- Verify broker IP address and port
- Test broker with MQTT client tools
- Check username/password if authentication is enabled
- Ensure broker is configured to accept WebSocket connections

### Command Issues

**Commands not working:**
- Check if ESP32 shows "MQTT connected" in serial monitor
- Verify command topic matches exactly
- Test with mosquitto_pub command first
- Check JSON format is correct

**Relays not switching:**
- Verify GPIO pin numbers in code match your wiring
- Check relay module power supply (5V vs 3.3V)
- Test relay module with direct GPIO control
- Check relay module signal voltage requirements

### Development Issues

**Upload fails:**
- Press and hold BOOT button on ESP32 while uploading
- Check USB cable supports data transfer
- Verify correct board and port selection
- Try different USB port or cable

**Serial monitor shows garbage:**
- Set baud rate to 115200
- Check if ESP32 is in bootloader mode
- Try pressing RESET button on ESP32

## Customization

### Adding Real Sensors

Replace the fake sensor data generation with actual sensor readings:

```cpp
// Example for DHT22 temperature/humidity sensor
#include <DHT.h>
#define DHT_PIN 4
#define DHT_TYPE DHT22
DHT dht(DHT_PIN, DHT_TYPE);

void publishSensorData() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  // ... rest of function
}
```

### Modifying GPIO Pins

Update pin assignments for your board:
```cpp
// Example for different ESP32-S3 board
const int LIGHT_RELAY_PIN = 10;   // Change to available GPIO
const int FAN_RELAY_PIN = 11;     // Change to available GPIO
```

### Adjusting Timing

Modify publish intervals as needed:
```cpp
const unsigned long SENSOR_PUBLISH_INTERVAL = 5000;   // 5 seconds instead of 3
const unsigned long HEARTBEAT_INTERVAL = 30000;       // 30 seconds instead of 15
```

## Production Notes

- Use secure MQTT (TLS/SSL) for production deployments
- Implement proper error handling and watchdog timers
- Add OTA (Over-The-Air) update capability
- Consider using deep sleep for battery-powered applications
- Implement sensor calibration and filtering
- Add configuration via web interface or mobile app
- Use hardware timers for precise timing requirements