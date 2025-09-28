# COPILOT_BRIEF.md - IoT Demo System ✅ COMPLETED

**👨‍💻 Author:** Nguyễn Trung Kiệt  
**🏫 Institution:** Thủ Dầu Một University (TDMU)  
**📅 Year:** 2025

## 0) Mục tiêu dự án - ✅ HOÀN THÀNH
Xây dựng hệ thống IoT demo gồm:
- **Web Dashboard (read-only):** ✅ hiển thị dữ liệu cảm biến và trạng thái thiết bị qua MQTT/WebSocket với giao diện đẹp.
- **App di động (Flutter Web):** ✅ gửi lệnh điều khiển **đèn** và **quạt** qua MQTT/WebSocket; subscribe để đồng bộ trạng thái real-time.
- **ESP32 Simulator:** ✅ Python simulator thay thế firmware; publish dữ liệu cảm biến + trạng thái, nhận lệnh và phản hồi.
- **ESP32-S3 firmware:** ✅ Code mẫu để sinh viên tự cấu hình.

> Kiến trúc "1 broker – 3 clients": Web Dashboard (view) + Flutter App (control) + ESP32 Simulator (device).
> **🔄 MQTT Synchronization hoạt động hoàn hảo!**_BRIEF.md

## 0) Mục tiêu dự án
Xây dựng hệ thống IoT demo gồm:
- **Web (read-only):** hiển thị dữ liệu cảm biến và trạng thái thiết bị qua MQTT/WebSocket.
- **App di động (Flutter):** gửi lệnh điều khiển **đèn** và **quạt** qua MQTT/TCP; subscribe để đồng bộ trạng thái.
- **ESP32-S3 firmware:** sinh viên tự cấu hình; thiết bị publish dữ liệu cảm biến + trạng thái, nhận lệnh và phản hồi.

> Kiến trúc “1 broker – 2 client”: App (control) + Web (view). ESP32-S3 là thiết bị hiện trường.

---

## 1) Kiến trúc & ràng buộc kỹ thuật

### Broker MQTT - ✅ IMPLEMENTED
- **Sử dụng:** HiveMQ Public Broker (broker.hivemq.com) - hoạt động ổn định
- **TCP Port 1883** (cho ESP32 simulator) và **WebSocket 8884/mqtt** (cho Web + Flutter)
- **Anonymous connection** - phù hợp demo
- **✅ Đã test:** Connection ổn định, message routing hoàn hảo

**Alternative: Local Mosquitto (infra/mosquitto.conf):**
```conf
listener 1883
allow_anonymous true

listener 9001
protocol websockets
```

### Namespace & Topics - ✅ IMPLEMENTED & TESTED
- **Namespace demo:** `demo/room1/` (đã cấu hình và test)
- **✅ Sensor data** (ESP → broker): `demo/room1/sensor/data`
  ```json
  {"ts":1695890000,"temp_c":28.6,"hum_pct":62.1,"lux":120}
  ```
- **✅ Device state** (ESP → broker, retained): `demo/room1/device/state`
  ```json
  {"ts":1695890000,"light":"on","fan":"off","rssi":-57,"fw":"sim-1.0.0"}
  ```
- **✅ Commands** (Web/App → ESP): `demo/room1/device/cmd`
  ```json
  {"light":"toggle"}   // "on" | "off" | "toggle"
  {"fan":"toggle"}
  ```
- **✅ Online status** (ESP → broker, retained): `demo/room1/sys/online`
  ```json
  {"online":true}
  ```

**Quy ước bắt buộc:** Sau khi thi hành lệnh, ESP **phải publish** `device/state` ngay để Web/App đồng bộ.

---

## 2) Cấu trúc thư mục mong muốn
```
repo-root/
├─ web/                 # Web read-only hiển thị
│  ├─ public/
│  └─ src/
│     └─ index.html     # (web tĩnh) hoặc Vite/React tùy chọn
├─ app_flutter/         # Ứng dụng Flutter điều khiển
│  └─ lib/
├─ firmware_esp32s3/    # Mã nguồn tham khảo cho sinh viên
│  └─ src/
├─ infra/
│  ├─ mosquitto.conf
│  └─ README.md
├─ .env.example         # Không commit secrets thực
└─ COPILOT_BRIEF.md
```

---

## 3) Biến môi trường (Web & App)
Tạo `.env` từ mẫu:
```
MQTT_HOST_WS=ws://192.168.1.10:9001
MQTT_HOST_TCP=192.168.1.10
MQTT_PORT_TCP=1883
MQTT_USERNAME=user1
MQTT_PASSWORD=pass1
TOPIC_NS=lab/room1
```

---

## 4) Yêu cầu tính năng chi tiết

### 4.1 Web (Read-only)
- **Không gửi lệnh**; chỉ subscribe:
  - `${TOPIC_NS}/sensor/state`
  - `${TOPIC_NS}/device/state`
  - `${TOPIC_NS}/sys/online`
- Hiển thị: Temp (°C), Humidity (%), Lux; Light (on/off), Fan (on/off), RSSI, FW; Online.
- Kết nối **MQTT over WebSocket** (mqtt.js). UI responsive, CSS tối giản, logic reconnect.

### 4.2 App Flutter (Control)
- Kết nối **MQTT/TCP** với `mqtt_client`.
- **Publish** lệnh lên `${TOPIC_NS}/device/cmd` dạng JSON; QoS 1.
- **Subscribe** `${TOPIC_NS}/device/state` và `${TOPIC_NS}/sys/online` để đồng bộ UI.
- Màn hình chính: 2 switch **Đèn**, **Quạt** + thanh trạng thái Broker/Device.
- Tự reconnect cơ bản; xử lý lỗi tối thiểu. Tham số đọc bằng `--dart-define`.

### 4.3 Firmware ESP32-S3 (tham khảo cho SV)
- Wi-Fi + MQTT (PubSubClient/ESP-IDF) + ArduinoJson.
- Publish cảm biến định kỳ (2–5s) → `sensor/state`.
- Nhận lệnh từ `device/cmd` → thi hành → **publish ngay `device/state`**.
- LWT: rớt → `{"online":false}`; lên lại → `{"online":true}`.
- 2 relay: **Light** và **Fan**; có thể thêm SHTxx/BH1750.

---

## 5) Bảo mật & vận hành
- Dùng username/password riêng cho **Web (read-only)** và **App (control)** nếu broker hỗ trợ ACL.
- **Retained** cho `device/state` & `sys/online` để UI hiển thị ngay khi mở.
- **QoS**: 1 cho lệnh & trạng thái; 0 cho sensor (chấp nhận mất gói).
- **Debounce** lệnh phía ESP (200–500ms).

---

## 6) Definition of Done (DoD)
- Web kết nối WS, hiển thị 3 số liệu cảm biến + trạng thái on/off + online.
- App Flutter bật/tắt đèn/quạt thành công, UI đồng bộ với `device/state`.
- Tắt ESP → Web/App chuyển `online=false` ≤ 5s; bật lại → `online=true`.
- Cấu hình tách khỏi logic, không hard-code bí mật.

---

## Prompts cho GitHub Copilot

> Dùng trong **Copilot Chat** tại thư mục tương ứng. Nói rõ file cần tạo & tiêu chí. Thay địa chỉ/biến `.env` nếu cần.

### A) Web (read-only, HTML tĩnh + mqtt.js)

**Prompt 1 — Khởi tạo file web tĩnh**
```
Bạn là GitHub Copilot. Hãy tạo trang web tĩnh read-only tại thư mục `web/`:

Yêu cầu:
- Tạo `web/src/index.html` dùng mqtt.js (WebSocket).
- Đọc biến từ `../.env` nếu có; nếu không, cho phép chỉnh trực tiếp ở đầu file: MQTT_HOST_WS, MQTT_USERNAME, MQTT_PASSWORD, TOPIC_NS.
- Subscribe `${TOPIC_NS}/sensor/state`, `${TOPIC_NS}/device/state`, `${TOPIC_NS}/sys/online`.
- Hiển thị temp, hum, lux, light, fan, rssi, fw, online. UI responsive, CSS tối giản.
- Không publish lệnh. Có reconnect & nhãn trạng thái kết nối.

Sau khi tạo xong, in toàn bộ nội dung `web/src/index.html`.
```

**Prompt 2 — README web**
```
Tạo `web/README.md` hướng dẫn chạy:
- `npx serve web/src` hoặc dùng Live Server
- Cách sửa `MQTT_HOST_WS` nếu không dùng .env
```

### B) App Flutter (control)

**Prompt 3 — Scaffold Flutter + MQTT**
```
Tại `app_flutter/`, tạo app Flutter:

- `pubspec.yaml`: thêm `mqtt_client` và `provider`.
- `lib/main.dart`:
  - Kết nối MQTT TCP dùng `String.fromEnvironment` cho: MQTT_HOST_TCP, MQTT_PORT_TCP, MQTT_USERNAME, MQTT_PASSWORD, TOPIC_NS.
  - Subscribe `${TOPIC_NS}/device/state`, `${TOPIC_NS}/sys/online`.
  - Hai SwitchListTile: bật/tắt light/fan → publish JSON lên `${TOPIC_NS}/device/cmd` (QoS 1).
  - Thanh trạng thái: Broker connected / Device online.
  - Tự reconnect cơ bản.

In nội dung `pubspec.yaml` và `lib/main.dart`.
```

**Prompt 4 — README Flutter**
```
Tạo `app_flutter/README.md`:
- Chạy:
  flutter run --dart-define=MQTT_HOST_TCP=192.168.1.10 --dart-define=MQTT_PORT_TCP=1883 --dart-define=MQTT_USERNAME=user1 --dart-define=MQTT_PASSWORD=pass1 --dart-define=TOPIC_NS=lab/room1
- Quyền mạng Android (nếu cần).
- Cách build APK release.
```

### C) Firmware ESP32-S3 (Arduino)

**Prompt 5 — Arduino sketch khung**
```
Tạo `firmware_esp32s3/src/main.cpp`:

- WiFi + PubSubClient + ArduinoJson.
- Biến cấu hình: WIFI_SSID, WIFI_PASS, MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASSWD, TOPIC_NS.
- Topics: sensor/state, device/cmd, device/state, sys/online.
- LWT: {"online":false}; khi kết nối: publish {"online":true} + device/state (retained).
- Publish cảm biến fake (temp_c/hum_pct/lux) mỗi 3s.
- Xử lý lệnh JSON: {"light":"on|off|toggle"}, {"fan":"on|off|toggle"} → phản hồi device/state ngay.
- Heartbeat device/state mỗi 15s (retained).
- GPIO mẫu: LIGHT_RELAY_PIN=5, FAN_RELAY_PIN=6 (ghi chú chỉnh theo board).

In toàn bộ nội dung `main.cpp`.
```

**Prompt 6 — README firmware**
```
Tạo `firmware_esp32s3/README.md`:
- Cài core ESP32 cho Arduino IDE/PlatformIO.
- Chỉnh SSID/PASS, MQTT_HOST/PORT, USER/PASS.
- Nạp code, monitor Serial, kiểm thử.
- Lưu ý GPIO có thể khác theo board.
```

### D) Hạ tầng & cấu hình

**Prompt 7 — `.env.example` & hướng dẫn broker**
```
Tạo `.env.example` ở gốc repo:
MQTT_HOST_WS=ws://192.168.1.10:9001
MQTT_HOST_TCP=192.168.1.10
MQTT_PORT_TCP=1883
MQTT_USERNAME=user1
MQTT_PASSWORD=pass1
TOPIC_NS=lab/room1

Tạo `infra/README.md`:
- Tạo user với `mosquitto_passwd`.
- Bật WebSocket theo `infra/mosquitto.conf`.
- Kiểm tra bằng `mosquitto_sub`/`mosquitto_pub`.
```
---

## Hướng dẫn phong cách cho Copilot
- Giữ **schema JSON và topics** đúng như mô tả.
- Code ngắn gọn, có comment tối thiểu; tách cấu hình khỏi logic.
- Thêm reconnect cơ bản, xử lý lỗi tối thiểu.
- Không hard-code secrets; đọc từ `.env` hoặc `--dart-define`.
- README rõ ràng cho từng phần.

---

## Checklist Review trước khi merge
- [ ] Web subscribe đủ 3 topic, không publish lệnh.
- [ ] App publish đúng JSON (QoS 1), subscribe state & sys/online.
- [ ] Firmware phản hồi `device/state` ngay sau lệnh; có LWT.
- [ ] `.env.example` có sẵn; README hướng dẫn chạy/build.
- [ ] Không leak mật khẩu thực trong repo.

---

### Lệnh nhanh tham khảo
```bash
# Web (chạy tĩnh)
npx serve web/src

# Flutter app (đổi IP/params)
cd app_flutter
flutter run --dart-define=MQTT_HOST_TCP=192.168.1.10   --dart-define=MQTT_PORT_TCP=1883   --dart-define=MQTT_USERNAME=user1   --dart-define=MQTT_PASSWORD=pass1   --dart-define=TOPIC_NS=lab/room1
```
