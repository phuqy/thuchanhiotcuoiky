# Web Dashboard - Read-Only IoT Monitor

Real-time web interface for monitoring IoT sensor data and device status via MQTT WebSocket.

## Features

- **Read-only monitoring** - No control commands sent
- **Real-time updates** via MQTT WebSocket
- **Responsive design** - Works on desktop and mobile
- **Auto-reconnection** - Handles network disconnections
- **Status indicators** - Broker connection and device online status

## What's Displayed

### Sensor Data
- Temperature (°C)
- Humidity (%)
- Light level (lux)

### Device Status
- Light: ON/OFF
- Fan: ON/OFF
- WiFi Signal strength (RSSI)

### System Info
- Firmware version
- Device online status
- Last update timestamp

## Quick Start

### Option 1: Using npx serve (Recommended)
```bash
cd web
npx serve src
```
Then open http://localhost:3000

### Option 2: Using Live Server (VS Code Extension)
1. Install "Live Server" extension in VS Code
2. Right-click on `web/src/index.html`
3. Select "Open with Live Server"

### Option 3: Using Python HTTP Server
```bash
cd web/src
python -m http.server 8000
```
Then open http://localhost:8000

## Configuration

Edit the `CONFIG` object in `index.html` to match your MQTT broker settings:

```javascript
const CONFIG = {
    MQTT_HOST_WS: 'ws://192.168.1.10:9001',  // Your broker WebSocket URL
    MQTT_USERNAME: 'user1',                   // MQTT username
    MQTT_PASSWORD: 'pass1',                   // MQTT password
    TOPIC_NS: 'lab/room1',                    // Topic namespace
    RECONNECT_PERIOD: 5000                    // Reconnect delay in ms
};
```

## Supported Browsers

- Chrome 60+
- Firefox 55+
- Safari 11+
- Edge 79+

## MQTT Topics Subscribed

- `${TOPIC_NS}/sensor/state` - Sensor readings (QoS 0)
- `${TOPIC_NS}/device/state` - Device status (QoS 1, retained)
- `${TOPIC_NS}/sys/online` - Online status (QoS 1, retained, LWT)

## Troubleshooting

### Connection Issues
1. Check MQTT broker is running and WebSocket port (9001) is open
2. Verify username/password in CONFIG match broker settings
3. Check browser console for error messages
4. Test broker connection with MQTT client tool

### Data Not Updating
1. Verify ESP32 device is publishing to correct topics
2. Check topic namespace matches CONFIG.TOPIC_NS
3. Monitor browser console for MQTT message logs

### WebSocket Connection Failed
1. Ensure broker has WebSocket listener enabled on port 9001
2. Check firewall settings allow WebSocket connections
3. Try accessing from same network as broker first

## Development

The web app uses vanilla HTML/CSS/JavaScript with mqtt.js library loaded from CDN. No build process required.

To modify:
1. Edit `index.html` directly
2. Refresh browser to see changes
3. Use browser developer tools for debugging