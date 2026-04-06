# Hướng dẫn Lắp Ráp & Cài Đặt Hệ thống IoT ESP32

## 1. Sơ đồ kết nối phần cứng

```
┌─────────────────────────────────────────────────────┐
│                    ESP32 DevKit V1                   │
│                                                     │
│  3V3 ──────────────────── VCC (HC-SR04)             │
│  GND ──────────────────── GND (Chung tất cả)        │
│                                                     │
│  GPIO5  (D5) ──────────── TRIG (HC-SR04)            │
│  GPIO18 (D18) ─────────── ECHO (HC-SR04)            │
│                                  [cần phân áp nếu   │
│                                   HC-SR04 dùng 5V]  │
│                                                     │
│  GPIO19 (D19) ─────────── Signal (Servo MG90S)      │
│  5V ───────────────────── VCC (Servo MG90S)         │
│  GND ──────────────────── GND (Servo MG90S)         │
│                                                     │
│  GPIO21 (SDA) ─────────── SDA (LCD I2C)             │
│  GPIO22 (SCL) ─────────── SCL (LCD I2C)             │
│  3V3 ──────────────────── VCC (LCD I2C)             │
│  GND ──────────────────── GND (LCD I2C)             │
│                                                     │
│  GPIO2  (D2) ──[330Ω]──── Anode LED (+)             │
│  GND ──────────────────── Cathode LED (-)           │
│                                                     │
│  GPIO34 (ADC) ─────────── Chân giữa Biến trở       │
│  3V3 ──────────────────── Chân 1 Biến trở           │
│  GND ──────────────────── Chân 3 Biến trở           │
│                                                     │
│  GPIO35 (D35) ─────────── Nút bấm (bên A)          │
│  GND ──────────────────── Nút bấm (bên B)           │
│         [INPUT_PULLUP nội – không cần trở ngoài]    │
└─────────────────────────────────────────────────────┘
```

> ⚠️ **Lưu ý quan trọng:**  
> - HC-SR04 5V: Echo output là 5V, ESP32 chỉ chịu 3.3V. Dùng **phân áp** (10kΩ + 20kΩ) hoặc module phiên bản 3.3V.  
> - Servo MG90S cần dòng cao: cấp nguồn **5V riêng từ USB** hoặc nguồn ngoài, không dùng pin 3V3 trên ESP32.  
> - GPIO34, 35 là **input-only**, không có pull-up nội – biến trở đã có phân áp nên OK; nút bấm dùng `INPUT_PULLUP` của GPIO35 (nội bộ ESP32 có hỗ trợ).

---

## 2. Phân áp cho Echo HC-SR04 (nếu dùng HC-SR04 5V)

```
ECHO (5V) ──[10kΩ]──┬── GPIO18 (ESP32)
                    │
                  [20kΩ]
                    │
                   GND
```

Điện áp tại GPIO18 = 5 × 20/(10+20) ≈ **3.33V** ✓

---

## 3. Cài đặt phần mềm

### 3.1 Arduino IDE – Cài Board ESP32

1. Mở Arduino IDE → **File > Preferences**
2. Thêm vào "Additional boards manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. **Tools > Board > Boards Manager** → tìm "esp32" → cài **esp32 by Espressif Systems**
4. Chọn board: **Tools > Board > ESP32 Arduino > ESP32 Dev Module**

### 3.2 Cài thư viện (Library Manager)

Vào **Sketch > Include Library > Manage Libraries**, tìm và cài:

| Thư viện | Tác giả |
|---|---|
| `PubSubClient` | Nick O'Leary |
| `LiquidCrystal I2C` | Frank de Brabander |
| `ESP32Servo` | Kevin Harrington |

### 3.3 Cài Mosquitto MQTT Broker

**Windows:**
```powershell
# Tải từ https://mosquitto.org/download/
# Sau khi cài, chạy:
mosquitto -v
```

**Ubuntu/Debian:**
```bash
sudo apt install mosquitto mosquitto-clients
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
```

**Tạo file cấu hình** (`/etc/mosquitto/conf.d/local.conf`):
```
listener 1883
allow_anonymous true
```

**Kiểm tra broker:**
```bash
# Terminal 1: Subscribe
mosquitto_sub -h localhost -t "iot/#" -v

# Terminal 2: Test publish
mosquitto_pub -h localhost -t "iot/test" -m "hello"
```

### 3.4 Cài Node-RED

```bash
# Cài Node.js (nếu chưa có)
# https://nodejs.org

# Cài Node-RED
npm install -g --unsafe-perm node-red

# Chạy Node-RED
node-red

# Truy cập: http://localhost:1880
```

**Cài thêm node dashboard:**
```bash
# Trong Node-RED: Menu > Manage Palette > Install
# Tìm: node-red-dashboard
```

### 3.5 Import flow vào Node-RED

1. Mở **http://localhost:1880**
2. Menu (☰) → **Import**
3. Chọn file `nodered_flows.json` hoặc dán nội dung vào
4. Click **Import** → **Deploy**
5. Dashboard tại: **http://localhost:1880/ui**

---

## 4. Cấu hình code Arduino

Mở `ESP32_IoT_System.ino`, sửa các dòng sau:

```cpp
const char* WIFI_SSID     = "TEN_WIFI_CUA_BAN";    // ← WiFi của bạn
const char* WIFI_PASSWORD = "MAT_KHAU_WIFI";         // ← Mật khẩu WiFi

const char* MQTT_BROKER   = "192.168.1.100";          // ← IP máy chạy Mosquitto
```

> **Tìm IP máy tính:**  
> - Windows: `ipconfig` → IPv4 Address  
> - Linux/Mac: `ip addr` hoặc `ifconfig`  
> - ESP32 và máy tính phải cùng mạng WiFi!

**Nạp code:**
1. Cắm ESP32 qua USB
2. **Tools > Port** → chọn cổng COM/ttyUSB
3. **Upload** (Ctrl+U)
4. Mở **Serial Monitor** (115200 baud) để xem log

---

## 5. Kiểm tra hệ thống

### Luồng dữ liệu hoạt động đúng khi:

```
[Serial Monitor]
=== ESP32 IoT System Boot ===
[WiFi] Đang kết nối: TEN_WIFI...
[WiFi] OK – IP: 192.168.1.xyz
[MQTT] Đang kết nối broker 192.168.1.100:1883 ...
[MQTT] Kết nối thành công!
```

### Test MQTT từ terminal:

```bash
# Xem toàn bộ dữ liệu ESP32 gửi lên
mosquitto_sub -h localhost -t "iot/#" -v

# Gửi lệnh điều khiển servo (góc 90°)
mosquitto_pub -h localhost -t "iot/control/servo" -m "90"

# Bật LED
mosquitto_pub -h localhost -t "iot/control/led" -m "ON"

# Gửi tin nhắn LCD
mosquitto_pub -h localhost -t "iot/control/lcd" -m "Hello World|From PC"

# Servo AUTO mode
mosquitto_pub -h localhost -t "iot/control/servo" -m "AUTO"
```

---

## 6. Cấu trúc MQTT Topics

| Topic | Hướng | Nội dung |
|---|---|---|
| `iot/sensor/distance` | ESP32 → Broker | Khoảng cách (float, đơn vị cm) |
| `iot/sensor/potval` | ESP32 → Broker | Biến trở (0-100, đơn vị %) |
| `iot/sensor/button` | ESP32 → Broker | `PRESSED` / `RELEASED` |
| `iot/device/status` | ESP32 → Broker | `online` / `offline` (retain=true) |
| `iot/control/servo` | Broker → ESP32 | Góc 0-180 hoặc `AUTO` |
| `iot/control/led` | Broker → ESP32 | `ON` / `OFF` / `BLINK` |
| `iot/control/lcd` | Broker → ESP32 | `Dòng1\|Dòng2` |

---

## 7. Logic tự động (Auto Mode)

Khi **Servo ở chế độ AUTO** (mặc định khi khởi động):

```
Khoảng cách < 30cm  →  Servo xoay 90°  +  LED cảnh báo (tùy cấu hình)
Khoảng cách ≥ 30cm  →  Servo về 0°
```

Có thể chỉnh ngưỡng trong code:
```cpp
#define DIST_THRESHOLD_CM   30   // ← thay đổi ngưỡng tại đây
```

---

## 8. Mở rộng thêm

Một số ý tưởng để nâng cấp hệ thống:

- **Thêm cảm biến DHT22**: nhiệt độ/độ ẩm → publish `iot/sensor/temperature`, `iot/sensor/humidity`
- **WebSocket**: thêm thư viện `AsyncTCP` + `ESPAsyncWebServer` để có Web UI trực tiếp trên ESP32
- **HTTPS/MQTTS**: dùng `WiFiClientSecure` + HiveMQ Cloud (miễn phí) để kết nối qua Internet
- **OTA Update**: thêm `ArduinoOTA` để cập nhật firmware qua WiFi không cần cắm dây
- **Data logging**: trong Node-RED dùng node `node-red-node-sqlite` hoặc `InfluxDB` để lưu lịch sử
- **Grafana**: kết hợp InfluxDB + Grafana để vẽ biểu đồ chuyên nghiệp

---

*Phiên bản: 1.0 | Arduino IDE 2.x | Node-RED 3.x | ESP32 Arduino Core 2.x*
