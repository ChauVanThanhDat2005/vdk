/*
 * ================================================================
 *  HỆ THỐNG NHÚNG ESP32 - IoT Environment Monitor & Controller
 * ================================================================
 *  Linh kiện:
 *    - ESP32 DevKit V1
 *    - HC-SR04  : cảm biến siêu âm đo khoảng cách
 *    - MG90S    : micro servo
 *    - LCD 16x2 : hiển thị qua I2C
 *    - LED       : đèn báo trạng thái
 *    - Biến trở  : điều chỉnh ngưỡng
 *    - Nút bấm  : điều khiển thủ công
 *
 *  Giao thức: MQTT qua WiFi
 *  IDE: Arduino IDE 2.x
 *  Thư viện cần cài:
 *    - PubSubClient  (Nick O'Leary)
 *    - LiquidCrystal_I2C (Frank de Brabander)
 *    - ESP32Servo
 * ================================================================
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>

// ─── WiFi & MQTT Configuration ────────────────────────────────
const char* WIFI_SSID     = "Dat";
const char* WIFI_PASSWORD = "87654321";

const char* MQTT_BROKER   = "192.168.1.12";   // IP máy chạy Mosquitto / broker
const int   MQTT_PORT     = 1883;
const char* MQTT_CLIENT_ID = "ESP32_IoT_01";
// const char* MQTT_USER   = "";                // bỏ comment nếu broker yêu cầu auth
// const char* MQTT_PASS   = "";

// ─── MQTT Topics ──────────────────────────────────────────────
// Publish  (ESP32 → Broker)
const char* TOPIC_DISTANCE      = "iot/sensor/distance";     // khoảng cách (cm)
const char* TOPIC_POTVAL        = "iot/sensor/potval";       // giá trị biến trở (0-100%)
const char* TOPIC_BTN1_STATE    = "iot/sensor/btn1";         // trạng thái nút 1
const char* TOPIC_BTN2_STATE    = "iot/sensor/btn2";         // trạng thái nút 2
const char* TOPIC_DOOR_STATUS   = "iot/device/door_status";  // "CLOSED", "OPEN", "LOCKED"
const char* TOPIC_LED1_STATE    = "iot/device/led1_state";   // "ON" / "OFF"
const char* TOPIC_LED2_STATE    = "iot/device/led2_state";   // "ON" / "OFF"
const char* TOPIC_STATUS        = "iot/device/status";       // "online" / "offline"

// Subscribe (Broker → ESP32)
const char* TOPIC_SERVO_CMD     = "iot/control/servo";       // 0-180 (góc)
const char* TOPIC_LED1_CMD      = "iot/control/led1";        // "ON" / "OFF" / "BLINK"
const char* TOPIC_LED2_CMD      = "iot/control/led2";        // "ON" / "OFF" / "BLINK"
const char* TOPIC_DOOR_LOCK_CMD = "iot/control/lock";        // "LOCK" / "UNLOCK"
const char* TOPIC_VOL_MODE_CMD  = "iot/control/vol_mode";    // "ON" / "OFF"
const char* TOPIC_LCD_MSG       = "iot/control/lcd";         // văn bản hiển thị LCD

// ─── Pin Definitions ──────────────────────────────────────────
#define PIN_TRIG        5    // HC-SR04 Trigger
#define PIN_ECHO        18   // HC-SR04 Echo
#define PIN_SERVO       19   // Servo signal
#define PIN_LED1        2    // LED nút 1: Khóa cửa
#define PIN_LED2        27   // LED nút 2: Chế độ điều chỉnh
#define PIN_POT         34   // Biến trở (ADC1_CH6 – input only)
#define PIN_BTN1        35   // Nút bấm 1: Khóa cửa (ADC1_CH7 – input only)
#define PIN_BTN2        33   // Nút bấm 2: Chế độ điều chỉnh

#define SDA_PIN         21
#define SCL_PIN         22

// ─── Thresholds & Timing ──────────────────────────────────────
#define DIST_THRESHOLD_CM   30    // Servo tự động mở khi vật < 30cm
#define MQTT_PUBLISH_MS     500   // Gửi dữ liệu cảm biến mỗi 500ms
#define RECONNECT_MS        5000  // Thử lại kết nối mỗi 5s

// ─── Objects ──────────────────────────────────────────────────
WiFiClient        wifiClient;
PubSubClient      mqttClient(wifiClient);
LiquidCrystal_I2C lcd(0x27, 16, 2);   // địa chỉ I2C phổ biến: 0x27 hoặc 0x3F
Servo             servo;

// ─── State Variables ──────────────────────────────────────────
float    lastDistance        = -1;
int      lastPotVal          = -1;
bool     lastBtn1State       = false;
bool     lastBtn2State       = false;
bool     ledBlinking1        = false;
bool     ledBlinking2        = false;
bool     autoMode            = true;      // auto: servo theo cảm biến; manual: theo lệnh MQTT
int      servoAngle          = 0;

// ─── Door Control ─────────────────────────────────────────────
bool     doorLocked          = false;     // trạng thái khóa
bool     volumeAdjustMode    = false;     // chế độ điều chỉnh bằng biến trở
String   doorStatus          = "OPEN";    // "OPEN", "CLOSED", "LOCKED"

String   lcdLine1            = "ESP32 IoT";
String   lcdLine2            = "Initializing...";

unsigned long lastPublishTime  = 0;
unsigned long lastReconnectTime = 0;
unsigned long lastBlinkTime1    = 0;
unsigned long lastBlinkTime2    = 0;
bool          ledBlinkState1    = false;
bool          ledBlinkState2    = false;

// ──────────────────────────────────────────────────────────────
// HC-SR04: Đo khoảng cách
// ──────────────────────────────────────────────────────────────
float measureDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH, 30000);  // timeout 30ms (~5m)
  if (duration == 0) return -1;                     // không nhận được echo
  return (duration * 0.0343) / 2.0;
}

// ──────────────────────────────────────────────────────────────
// LCD: Cập nhật hiển thị
// ──────────────────────────────────────────────────────────────
void updateLCD(String line1, String line2) {
  lcdLine1 = line1;
  lcdLine2 = line2;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1.substring(0, 16));
  lcd.setCursor(0, 1);
  lcd.print(line2.substring(0, 16));
}

// ──────────────────────────────────────────────────────────────
// Servo: Đặt góc
// ──────────────────────────────────────────────────────────────
void setServoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  servoAngle = angle;
  servo.write(angle);
  
  // Cập nhật trạng thái cửa
  if (angle > 120) {
    doorStatus = doorLocked ? "LOCKED" : "OPEN";
  } else {
    doorStatus = "CLOSED";
  }
}

// ──────────────────────────────────────────────────────────────
// Door Lock: Khóa cửa
// ──────────────────────────────────────────────────────────────
void lockDoor() {
  if (servoAngle == 0) {
    doorLocked = true;
    doorStatus = "LOCKED";
    digitalWrite(PIN_LED1, HIGH);
    mqttClient.publish(TOPIC_LED1_STATE, "ON", true);
    Serial.println("[Door] Đã khóa cửa");
    mqttClient.publish(TOPIC_DOOR_STATUS, "LOCKED", true);
    updateLCD("Door Status", "LOCKED");
  } else {
    Serial.println("[Door] Không thể khóa - servo phải ở 0°!");
    updateLCD("LOCK FAILED!", "Servo must be 0°");
  }
}

void unlockDoor() {
  doorLocked = false;
  doorStatus = "CLOSED";
  digitalWrite(PIN_LED1, LOW);
  mqttClient.publish(TOPIC_LED1_STATE, "OFF", true);
  Serial.println("[Door] Đã mở khóa cửa");
  mqttClient.publish(TOPIC_DOOR_STATUS, "CLOSED", true);
}

// ──────────────────────────────────────────────────────────────
// Volume Adjust Mode: Bật/Tắt chế độ điều chỉnh
// ──────────────────────────────────────────────────────────────
void toggleVolumeMode() {
  volumeAdjustMode = !volumeAdjustMode;
  if (volumeAdjustMode) {
    digitalWrite(PIN_LED2, HIGH);
    mqttClient.publish(TOPIC_LED2_STATE, "ON", true);
    Serial.println("[Volume] Bật chế độ điều chỉnh");
    mqttClient.publish(TOPIC_VOL_MODE_CMD, "ON", true);
    updateLCD("Volume Adjust", "Mode: ON");
  } else {
    digitalWrite(PIN_LED2, LOW);
    mqttClient.publish(TOPIC_LED2_STATE, "OFF", true);
    Serial.println("[Volume] Tắt chế độ điều chỉnh");
    mqttClient.publish(TOPIC_VOL_MODE_CMD, "OFF", true);
    updateLCD("Volume Adjust", "Mode: OFF");
  }
}

// ──────────────────────────────────────────────────────────────
// MQTT: Callback nhận lệnh từ Node-RED / Dashboard
// ──────────────────────────────────────────────────────────────
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  Serial.printf("[MQTT IN] Topic: %s | Msg: %s\n", topic, msg.c_str());

  // ── Điều khiển Servo ─────────────────────────────────
  if (topicStr == TOPIC_SERVO_CMD) {
    if (msg == "AUTO") {
      autoMode = true;
      Serial.println("[Servo] Chuyển sang AUTO mode");
    } else {
      autoMode = false;
      int angle = msg.toInt();
      setServoAngle(angle);
      Serial.printf("[Servo] Manual: %d°\n", angle);
    }
  }

  // ── Điều khiển Khóa cửa ──────────────────────────────
  else if (topicStr == TOPIC_DOOR_LOCK_CMD) {
    if (msg == "LOCK") {
      lockDoor();
    } else if (msg == "UNLOCK") {
      unlockDoor();
    }
  }

  // ── Chế độ điều chỉnh volume ─────────────────────────
  else if (topicStr == TOPIC_VOL_MODE_CMD) {
    if (msg == "ON") {
      if (!volumeAdjustMode) {
        volumeAdjustMode = true;
        digitalWrite(PIN_LED2, HIGH);
        mqttClient.publish(TOPIC_VOL_MODE_CMD, "ON", true);
      }
    } else if (msg == "OFF") {
      if (volumeAdjustMode) {
        volumeAdjustMode = false;
        digitalWrite(PIN_LED2, LOW);
        mqttClient.publish(TOPIC_VOL_MODE_CMD, "OFF", true);
      }
    }
  }

  // ── Điều khiển LED 1 ────────────────────────────────
  else if (topicStr == TOPIC_LED1_CMD) {
    if (msg == "ON") {
      ledBlinking1 = false;
      digitalWrite(PIN_LED1, HIGH);
    } else if (msg == "OFF") {
      ledBlinking1 = false;
      digitalWrite(PIN_LED1, LOW);
    } else if (msg == "BLINK") {
      ledBlinking1 = true;
    }
  }

  // ── Điều khiển LED 2 ────────────────────────────────
  else if (topicStr == TOPIC_LED2_CMD) {
    if (msg == "ON") {
      ledBlinking2 = false;
      digitalWrite(PIN_LED2, HIGH);
    } else if (msg == "OFF") {
      ledBlinking2 = false;
      digitalWrite(PIN_LED2, LOW);
    } else if (msg == "BLINK") {
      ledBlinking2 = true;
    }
  }

  // ── Tin nhắn hiển thị LCD ────────────────────────────
  else if (topicStr == TOPIC_LCD_MSG) {
    // Định dạng: "Line1|Line2" hoặc chỉ "Line1"
    int sep = msg.indexOf('|');
    if (sep >= 0) {
      updateLCD(msg.substring(0, sep), msg.substring(sep + 1));
    } else {
      updateLCD(msg, lcdLine2);
    }
  }
}

// ──────────────────────────────────────────────────────────────
// WiFi: Kết nối
// ──────────────────────────────────────────────────────────────
void connectWiFi() {
  Serial.printf("\n[WiFi] Đang kết nối: %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  updateLCD("Connecting WiFi", WIFI_SSID);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] OK – IP: %s\n", WiFi.localIP().toString().c_str());
    updateLCD("WiFi OK", WiFi.localIP().toString());
  } else {
    Serial.println("\n[WiFi] Thất bại! Tiếp tục offline...");
    updateLCD("WiFi FAILED", "Offline mode");
  }
}

// ──────────────────────────────────────────────────────────────
// MQTT: Kết nối / Reconnect
// ──────────────────────────────────────────────────────────────
bool connectMQTT() {
  Serial.printf("[MQTT] Đang kết nối broker %s:%d ...\n", MQTT_BROKER, MQTT_PORT);
  updateLCD("MQTT Connecting", MQTT_BROKER);

  // Last Will Testament – tự động báo offline khi ESP32 mất kết nối
  bool ok = mqttClient.connect(
    MQTT_CLIENT_ID,
    nullptr, nullptr,         // user/pass (bỏ comment MQTT_USER/PASS nếu cần)
    TOPIC_STATUS, 1, true,    // will topic, QoS, retain
    "offline"
  );

  if (ok) {
    Serial.println("[MQTT] Kết nối thành công!");
    mqttClient.publish(TOPIC_STATUS, "online", true);

    // Subscribe các topic điều khiển
    mqttClient.subscribe(TOPIC_SERVO_CMD);
    mqttClient.subscribe(TOPIC_DOOR_LOCK_CMD);
    mqttClient.subscribe(TOPIC_VOL_MODE_CMD);
    mqttClient.subscribe(TOPIC_LED1_CMD);
    mqttClient.subscribe(TOPIC_LED2_CMD);
    mqttClient.subscribe(TOPIC_LCD_MSG);

    updateLCD("MQTT Connected", MQTT_CLIENT_ID);
    return true;
  } else {
    Serial.printf("[MQTT] Thất bại, rc=%d\n", mqttClient.state());
    return false;
  }
}

// ──────────────────────────────────────────────────────────────
// Publish: Gửi dữ liệu cảm biến lên Broker
// ──────────────────────────────────────────────────────────────
void publishSensorData() {
  if (!mqttClient.connected()) return;

  // Khoảng cách
  float dist = measureDistance();
  if (dist > 0) {
    char buf[16];
    dtostrf(dist, 5, 1, buf);
    mqttClient.publish(TOPIC_DISTANCE, buf, false);  // QoS 0, no retain
    lastDistance = dist;

    // Auto mode: servo tự động dựa trên khoảng cách
    if (autoMode && !doorLocked && !volumeAdjustMode) {
      int angle = (dist < DIST_THRESHOLD_CM) ? 90 : 0;
      setServoAngle(angle);
      if (dist < DIST_THRESHOLD_CM) {
        doorStatus = "OPEN";
      } else {
        doorStatus = "CLOSED";
      }
    }
  }

  // Chế độ điều chỉnh volume: Biến trở điều khiển góc servo
  if (volumeAdjustMode) {
    int raw = analogRead(PIN_POT);
    int angle = map(raw, 0, 4095, 0, 180);
    if (angle != servoAngle) {
      setServoAngle(angle);
    }
  } else {
    // Biến trở (0-4095 → 0-100%)
    int raw = analogRead(PIN_POT);
    int pot = map(raw, 0, 4095, 0, 100);
    if (abs(pot - lastPotVal) > 1) {   // chỉ gửi khi thay đổi > 1%
      char buf[8];
      itoa(pot, buf, 10);
      mqttClient.publish(TOPIC_POTVAL, buf);
      lastPotVal = pot;
    }
  }

  // Nút bấm 1
  bool btn1Pressed = (digitalRead(PIN_BTN1) == LOW);
  if (btn1Pressed != lastBtn1State) {
    mqttClient.publish(TOPIC_BTN1_STATE, btn1Pressed ? "PRESSED" : "RELEASED");
    lastBtn1State = btn1Pressed;
    if (btn1Pressed) {
      if (doorLocked) {
        unlockDoor();
      } else {
        lockDoor();
      }
    }
  }

  // Nút bấm 2
  bool btn2Pressed = (digitalRead(PIN_BTN2) == LOW);
  if (btn2Pressed != lastBtn2State) {
    mqttClient.publish(TOPIC_BTN2_STATE, btn2Pressed ? "PRESSED" : "RELEASED");
    lastBtn2State = btn2Pressed;
    if (btn2Pressed) {
      toggleVolumeMode();
    }
  }

  // Cập nhật LCD
  if (dist > 0) {
    String l1 = "Dist: " + String(dist, 1) + "cm S:" + String(servoAngle) + (char)223;
    String l2 = doorStatus;
    if (volumeAdjustMode) l2 += " [VOL]";
    if (doorLocked) l2 += " [LOCK]";
    updateLCD(l1, l2);
  }
}

// ──────────────────────────────────────────────────────────────
// Setup
// ──────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 IoT System Boot ===");

  // GPIO
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);
  analogReadResolution(12);

  // LCD
  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init();
  lcd.backlight();
  updateLCD("ESP32 IoT", "Booting...");

  // Servo
  servo.attach(PIN_SERVO, 500, 2400);
  setServoAngle(0);

  // WiFi
  connectWiFi();

  // MQTT
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60);
  mqttClient.setBufferSize(512);
  connectMQTT();
}

// ──────────────────────────────────────────────────────────────
// Loop
// ──────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── Giữ kết nối WiFi ─────────────────────────────────
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Mất kết nối! Đang kết nối lại...");
    connectWiFi();
  }

  // ── Giữ kết nối MQTT ─────────────────────────────────
  if (!mqttClient.connected()) {
    if (now - lastReconnectTime > RECONNECT_MS) {
      lastReconnectTime = now;
      connectMQTT();
    }
  } else {
    mqttClient.loop();    // xử lý incoming messages
  }

  // ── Publish dữ liệu cảm biến định kỳ ─────────────────
  if (now - lastPublishTime > MQTT_PUBLISH_MS) {
    lastPublishTime = now;
    publishSensorData();
  }

  // ── LED 1 Blink ──────────────────────────────────────
  if (ledBlinking1 && now - lastBlinkTime1 > 300) {
    lastBlinkTime1 = now;
    ledBlinkState1 = !ledBlinkState1;
    digitalWrite(PIN_LED1, ledBlinkState1);
  }

  // ── LED 2 Blink ──────────────────────────────────────
  if (ledBlinking2 && now - lastBlinkTime2 > 300) {
    lastBlinkTime2 = now;
    ledBlinkState2 = !ledBlinkState2;
    digitalWrite(PIN_LED2, ledBlinkState2);
  }
}
