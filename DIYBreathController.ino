#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include <esp_bt.h>

#define ADS_CHANNEL 1
#define SCL_PIN     12
#define SDA_PIN     13
long maxAdc = 14000;
const int MAPPED_MAX = 127;

Adafruit_ADS1115 ads;
Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

long zeroOffset = 0;
bool calibrated = false;
unsigned long calibrateStart;
byte ccNum = 2;

// ==== 自动调零核心参数 ====
const int AUTO_ZERO_LIMIT = 30;
const int STABLE_DURATION = 10000;    // 10秒
const int STABLE_RANGE = 2;           // 允许的小波动
const int BUFFER_LEN = 15;            // 采样缓冲帧数
int mappingBuffer[BUFFER_LEN] = {0};
int bufferPos = 0;
unsigned long stableStartTime = 0;
bool autoZeroActive = false;

void handleSerialProtocol();

void setup() {
  TinyUSBDevice.setManufacturerDescriptor("BarryAllen");
  TinyUSBDevice.setProductDescriptor("BarrysBreathController");
  pinMode(48, OUTPUT);
  digitalWrite(48, LOW);     // 关闭 GPIO48 的灯

  WiFi.mode(WIFI_OFF);
  btStop();
  esp_bt_controller_disable();

  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  ads.setGain(GAIN_TWOTHIRDS);
  if (!ads.begin()) {
    Serial.println("ADS1115 初始化失败");
    while (1);
  }
  calibrateStart = millis();
  zeroOffset = 0;
  MIDI.begin(MIDI_CHANNEL_OMNI);
}

void loop() {
  handleSerialProtocol();
  static long zeroSum = 0;
  static int zeroCount = 0;
  int16_t raw = ads.readADC_SingleEnded(ADS_CHANNEL);

  // 调零（200ms均值）
  if (!calibrated) {
    zeroSum += raw;
    zeroCount++;
    if (millis() - calibrateStart >= 200) {
      zeroOffset = zeroSum / zeroCount;
      calibrated = true;
      Serial.print("CALIBRATE ");
      Serial.println(zeroOffset);
    }
    return;
  }

  long adjusted = raw - zeroOffset;
  if (adjusted < 0) adjusted = 0;
  int mapped = map(adjusted, 0, maxAdc, 0, MAPPED_MAX);
  mapped = constrain(mapped, 0, MAPPED_MAX);

  // MIDI输出（只在变化时发送）
  static int prevMapped = -1;
  if (mapped != prevMapped) {
    MIDI.sendControlChange(ccNum, mapped, 1);
    Serial.print("DATA ");
    Serial.println(mapped);
    prevMapped = mapped;
  }

  // ========== 自动调零核心逻辑 ==========
  // 更新环形buffer
  mappingBuffer[bufferPos] = mapped;
  bufferPos = (bufferPos + 1) % BUFFER_LEN;

  // 只有 mapped < AUTO_ZERO_LIMIT 时才检测静止
  if (mapped < AUTO_ZERO_LIMIT) {
    // 判断 buffer 是否“全部一样”或“基本一样”
    int ref = mappingBuffer[0];
    bool allSame = true;
    for (int i = 1; i < BUFFER_LEN; i++) {
      if (abs(mappingBuffer[i] - ref) > STABLE_RANGE) {
        allSame = false;
        break;
      }
    }
    if (allSame) {
      if (!autoZeroActive) {
        stableStartTime = millis();
        autoZeroActive = true;
      } else if (millis() - stableStartTime > STABLE_DURATION) {
        // 静止超过10秒，执行自动调零
        zeroSum = 0;
        zeroCount = 0;
        calibrateStart = millis();
        calibrated = false;
        autoZeroActive = false;
        Serial.println("AUTO ZERO START");
      }
    } else {
      autoZeroActive = false;
    }
  } else {
    autoZeroActive = false;
  }
}

// =======================
// 辅助函数：串口协议处理
// =======================
void handleSerialProtocol() {
  static String rx;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      rx.trim();
      if (!rx.length()) { rx = ""; continue; }
      int idx1 = rx.indexOf(' ');
      String cmd = (idx1 < 0) ? rx : rx.substring(0, idx1);
      cmd.toUpperCase();

      if (cmd == "HELLO" || cmd == "HELLO PC") {
        Serial.println("HELLO BREATH");
        rx = ""; continue;
      }
      if (cmd == "SET") {
        int idx2 = rx.indexOf(' ', idx1 + 1);
        if (idx2 > 0) {
          String param = rx.substring(idx1 + 1, idx2);
          String value = rx.substring(idx2 + 1);
          if (param == "MAX") { maxAdc = value.toInt(); Serial.println("ACK"); rx = ""; continue; }
          if (param == "CC")  { ccNum = value.toInt();  Serial.println("ACK"); rx = ""; continue; }
          Serial.println("ERR 02"); rx = ""; continue;
        }
        Serial.println("ERR 01"); rx = ""; continue;
      }
      if (cmd == "GET") {
        String param = rx.substring(idx1 + 1);
        if (param == "MAX") { Serial.printf("MAX %ld\n", maxAdc); rx = ""; continue; }
        if (param == "CC")  { Serial.printf("CC %d\n", ccNum);    rx = ""; continue; }
        Serial.println("ERR 03"); rx = ""; continue;
      }
      Serial.println("ERR 00");
      rx = "";
    } else {
      rx += c;
    }
  }
}
