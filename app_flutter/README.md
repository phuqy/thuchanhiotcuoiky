# IoT Controller Flutter App

Flutter mobile application for controlling IoT devices (lights and fans) via MQTT TCP connection.

## Features

- **Real-time control** of lights and fans via MQTT
- **Device status monitoring** with live updates
- **Connection status indicators** for broker and device
- **Auto-reconnection** handling
- **Responsive Material Design** UI

## Screenshots

The app displays:
- Broker connection status (Connected/Disconnected)
- Device online status (Online/Offline) 
- Light control switch with status
- Fan control switch with status
- Device information (WiFi signal, firmware version, last update)

## Prerequisites

- Flutter SDK 3.10.0 or higher
- Dart SDK 3.0.0 or higher
- Android Studio or VS Code with Flutter extensions
- MQTT broker running and accessible from mobile device

## Installation

1. **Clone and navigate to project:**
```bash
cd app_flutter
```

2. **Install dependencies:**
```bash
flutter pub get
```

3. **Run the app:**
```bash
flutter run --dart-define=MQTT_HOST_TCP=192.168.1.10 --dart-define=MQTT_PORT_TCP=1883 --dart-define=MQTT_USERNAME=user1 --dart-define=MQTT_PASSWORD=pass1 --dart-define=TOPIC_NS=lab/room1
```

## Configuration

The app uses environment variables passed via `--dart-define` flags:

| Variable | Description | Default |
|----------|-------------|---------|
| `MQTT_HOST_TCP` | MQTT broker IP address | `192.168.1.10` |
| `MQTT_PORT_TCP` | MQTT broker TCP port | `1883` |
| `MQTT_USERNAME` | MQTT username | `user1` |
| `MQTT_PASSWORD` | MQTT password | `pass1` |
| `TOPIC_NS` | MQTT topic namespace | `lab/room1` |

### Example Run Commands

**Local development:**
```bash
flutter run --dart-define=MQTT_HOST_TCP=192.168.1.100 --dart-define=MQTT_USERNAME=myuser --dart-define=MQTT_PASSWORD=mypass
```

**Different room/namespace:**
```bash
flutter run --dart-define=TOPIC_NS=lab/room2
```

## Building

### Debug APK
```bash
flutter build apk --debug --dart-define=MQTT_HOST_TCP=192.168.1.10 --dart-define=MQTT_USERNAME=user1 --dart-define=MQTT_PASSWORD=pass1
```

### Release APK
```bash
flutter build apk --release --dart-define=MQTT_HOST_TCP=192.168.1.10 --dart-define=MQTT_USERNAME=user1 --dart-define=MQTT_PASSWORD=pass1 --dart-define=TOPIC_NS=lab/room1
```

The APK will be generated at: `build/app/outputs/flutter-apk/app-release.apk`

## Android Permissions

The app automatically includes internet permission in `android/app/src/main/AndroidManifest.xml`:
```xml
<uses-permission android:name="android.permission.INTERNET" />
```

## MQTT Topics

### Published Topics (by app):
- `${TOPIC_NS}/device/cmd` - Device commands (QoS 1)
  ```json
  {"light": "toggle"}
  {"fan": "toggle"}
  ```

### Subscribed Topics:
- `${TOPIC_NS}/device/state` - Device status updates (QoS 1, retained)
- `${TOPIC_NS}/sys/online` - Device online status (QoS 1, retained, LWT)

## App Architecture

- **Provider pattern** for state management
- **MqttController** handles all MQTT communication
- **Reactive UI** updates automatically when device state changes
- **Auto-reconnect** on connection loss

## Troubleshooting

### Connection Issues

1. **Cannot connect to broker:**
   - Verify MQTT broker IP address is correct
   - Check if broker is running and accessible from mobile network
   - Ensure ports 1883 is not blocked by firewall
   - Test connection with MQTT client tool first

2. **Authentication failed:**
   - Verify username/password match broker configuration
   - Check broker logs for authentication errors

3. **App crashes on startup:**
   - Check Flutter and Dart SDK versions
   - Run `flutter clean && flutter pub get`
   - Check device logs: `flutter logs`

### Control Issues

1. **Switches don't work:**
   - Check if device is shown as "Online"
   - Verify ESP32 device is subscribed to command topic
   - Monitor MQTT broker logs for published commands

2. **Status not updating:**
   - Check if ESP32 is publishing to device/state topic
   - Verify topic namespace matches configuration
   - Ensure retained messages are enabled on broker

### Development

**Hot reload during development:**
```bash
flutter run --dart-define=MQTT_HOST_TCP=192.168.1.10
# Press 'r' for hot reload, 'R' for hot restart
```

**Debug output:**
```bash
flutter run --verbose
```

**Check dependencies:**
```bash
flutter doctor
flutter pub deps
```

## Dependencies

- `mqtt_client: ^10.2.0` - MQTT client for TCP connection
- `provider: ^6.1.1` - State management
- `cupertino_icons: ^1.0.6` - iOS-style icons

## Network Requirements

- Mobile device must be on same network as MQTT broker, OR
- MQTT broker must be accessible from mobile network (port forwarding/VPN)
- No special router configuration needed for local network usage