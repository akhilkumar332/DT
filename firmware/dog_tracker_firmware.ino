// === Libraries ===
#define TINY_GSM_MODEM_A7670
#define TINY_GSM_RX_BUFFER 1024

#include <TinyGsmClient.h>
#include <Wire.h>
#include <MAX30105.h>
#include "heartRate.h"
#include <BMI160Gen.h>
#include <driver/i2s.h>
#include <ArduinoJson.h>
#include <math.h>
#include <algorithm>

// === GSM Modem Pins ===
#define MODEM_PWRKEY 5
#define MODEM_RST    4
#define MODEM_TX     18
#define MODEM_RX     17
#define SerialAT     Serial1

// === I2C Pins ===
#define BMI_SDA 21
#define BMI_SCL 11
#define HR_SDA  36
#define HR_SCL  37

// === I2S Pins ===
#define I2S_SD   9
#define I2S_WS   8
#define I2S_SCK  7
#define I2S_PORT I2S_NUM_0

// === I2S Audio Config ===
const uint32_t SAMPLE_RATE = 16000;
#define SAMPLE_BITS     I2S_BITS_PER_SAMPLE_32BIT
#define CHANNEL_FORMAT  I2S_CHANNEL_FMT_ONLY_LEFT
#define RECORD_SECONDS  4
#define SAMPLE_COUNT    (SAMPLE_RATE * RECORD_SECONDS)
#define BUFFER_SAMPLES  256
#define TRIGGER_THRESHOLD 20000

// === GSM Config ===
const char apn[] = "INTERNET";
const char* server = "63.177.227.146";

// TODO: load from EEPROM in production
const char* deviceId = "TEST_DEVICE_ID";

const char motionPath[] = "/api/motion";
const char heartPath[] = "/api/heart";
const char soundPath[] = "/api/sound";

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

// === Sensors ===
MAX30105 max3010x;
TwoWire HRWire(1);
int adpcm_index = 0;
int predsample = 0;

const int indexTable[16] = {
  -1, -1, -1, -1, 2, 4, 6, 8,
  -1, -1, -1, -1, 2, 4, 6, 8
};

const int stepsizeTable[89] = {
  7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
  19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
  50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
  130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
  337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
  876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
  2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
  5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
  15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

// === BPM Tracking ===
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;

// === Setup ===
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n🔧 Booting unified sensor system...");

  Wire.begin(BMI_SDA, BMI_SCL);
  HRWire.begin(HR_SDA, HR_SCL);

  setupBMI160();
  setupMAX3010X();
  setupI2S();
  setupGSM();

  Serial.println("✅ All sensors initialized");
}

// === Loop ===
void loop() {
  readAndSendMotion();
  readAndSendHeartRate();
  listenAndSendAudio();
}

// === BMI160 ===
void setupBMI160() {
  if (!BMI160.begin(BMI160GenClass::I2C_MODE, 0x69)) {
    Serial.println("❌ BMI160 init failed"); while (1);
  }
  Serial.println("✅ BMI160 initialized");
}

void readAndSendMotion() {
  int gx, gy, gz, ax, ay, az;
  BMI160.readGyro(gx, gy, gz);
  BMI160.readAccelerometer(ax, ay, az);

  float fax = ax / 16384.0, fay = ay / 16384.0, faz = az / 16384.0;
  float tilt = atan2(fay, faz) * 180.0 / PI;
  String orientation = (abs(tilt) < 30) ? "horizontal" : "vertical";
  float speed = sqrt(fax * fax + fay * fay + faz * faz);
  String movement = speed > 1.8 ? "running" : speed > 1.2 ? "walking" : "still";

  StaticJsonDocument<256> sensorDoc;
  sensorDoc["gyroscope"]["x"] = gx;
  sensorDoc["gyroscope"]["y"] = gy;
  sensorDoc["gyroscope"]["z"] = gz;
  sensorDoc["accelerometer"]["x"] = fax;
  sensorDoc["accelerometer"]["y"] = fay;
  sensorDoc["accelerometer"]["z"] = faz;
  sensorDoc["tilt"] = tilt;
  sensorDoc["orientation"] = orientation;
  sensorDoc["movement"] = movement;
  sensorDoc["timestamp"] = millis();

  StaticJsonDocument<512> root;
  root["deviceId"] = deviceId;
  root["data"] = sensorDoc;

  String payload;
  serializeJson(root, payload);
  Serial.println("📦 Motion: "); Serial.println(payload);

  if (client.connect(server, 80)) {
    client.println(String("POST ") + motionPath + " HTTP/1.1");
    client.println("Host: " + String(server));
    client.println("Content-Type: application/json");
    client.println("Content-Length: " + payload.length());
    client.println(); client.print(payload);

    unsigned long timeout = millis();
    while (client.connected() && millis() - timeout < 3000) {
      if (client.available()) {
        String line = client.readStringUntil('\n');
        Serial.println("📨 Server: " + line);
      }
    }

    client.stop();
  }
  delay(100);
}

// === MAX3010X ===
void setupMAX3010X() {
  if (!max3010x.begin(HRWire, I2C_SPEED_STANDARD)) {
    Serial.println("❌ MAX3010X not found"); while (1);
  }
  max3010x.setup();
  max3010x.setPulseAmplitudeIR(0x3F);
  max3010x.setPulseAmplitudeRed(0x00);
  Serial.println("✅ MAX3010X initialized");
}

void readAndSendHeartRate() {
  long irValue = max3010x.getIR();
  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    if (beatsPerMinute > 10 && beatsPerMinute < 255) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;
      beatAvg = 0; for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  StaticJsonDocument<128> root;
  root["deviceId"] = deviceId;
  root["bpm"] = beatAvg;
  root["ir"] = irValue;

  String payload;
  serializeJson(root, payload);
  Serial.println("❤️ Heart: " + payload);

  if (client.connect(server, 80)) {
    client.println(String("POST ") + heartPath + " HTTP/1.1");
    client.println("Host: " + String(server));
    client.println("Content-Type: application/json");
    client.println("Content-Length: " + payload.length());
    client.println(); client.print(payload);

    unsigned long timeout = millis();
    while (client.connected() && millis() - timeout < 3000) {
      if (client.available()) {
        String line = client.readStringUntil('\n');
        Serial.println("📨 Server: " + line);
      }
    }

    client.stop();
  }
  delay(100);
}

// === I2S Microphone ===
void writeWavHeader(uint8_t* wav, uint32_t dataSize) {
  memcpy(wav, "RIFF", 4);
  uint32_t chunkSize = 36 + dataSize;
  memcpy(wav + 4, &chunkSize, 4);
  memcpy(wav + 8, "WAVEfmt ", 8);
  uint32_t subChunk1Size = 20;
  memcpy(wav + 16, &subChunk1Size, 4);
  uint16_t format = 0x11;
  memcpy(wav + 20, &format, 2);
  uint16_t channels = 1;
  memcpy(wav + 22, &channels, 2);
  memcpy(wav + 24, &SAMPLE_RATE, 4);
  uint32_t byteRate = SAMPLE_RATE / 2;
  memcpy(wav + 28, &byteRate, 4);
  uint16_t blockAlign = 256;
  memcpy(wav + 32, &blockAlign, 2);
  uint16_t bitsPerSample = 4;
  memcpy(wav + 34, &bitsPerSample, 2);
  uint16_t extraSize = 2;
  memcpy(wav + 36, &extraSize, 2);
  uint16_t samplesPerBlock = 505;
  memcpy(wav + 38, &samplesPerBlock, 2);
  memcpy(wav + 40, "data", 4);
  memcpy(wav + 44, &dataSize, 4);
}

uint8_t adpcm_encode(int16_t sample) {
  int delta = sample - predsample;
  int step = stepsizeTable[adpcm_index];
  int code = 0;

  if (delta < 0) { code = 8; delta = -delta; }
  if (delta >= step) { code |= 4; delta -= step; }
  if (delta >= step / 2) { code |= 2; delta -= step / 2; }
  if (delta >= step / 4) code |= 1;

  int diff = step >> 3;
  if (code & 4) diff += step;
  if (code & 2) diff += step >> 1;
  if (code & 1) diff += step >> 2;
  if (code & 8) diff = -diff;

  predsample += diff;
  predsample = constrain(predsample, -32768, 32767);
  adpcm_index += indexTable[code];
  adpcm_index = constrain(adpcm_index, 0, 88);

  return code & 0x0F;
}

void setupI2S() {
  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = SAMPLE_BITS,
    .channel_format = CHANNEL_FORMAT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };
  i2s_driver_install(I2S_PORT, &config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
  i2s_zero_dma_buffer(I2S_PORT);
  Serial.println("✅ I2S initialized");
}

void recordAndSendCompressedWav() {
  static uint32_t raw_buffer[BUFFER_SAMPLES];
  int16_t* samples = (int16_t*) malloc(SAMPLE_COUNT * sizeof(int16_t));
  if (!samples) return;

  size_t bytes_read;
  int idx = 0;

  while (idx < SAMPLE_COUNT) {
    if (i2s_read(I2S_PORT, &raw_buffer, sizeof(raw_buffer), &bytes_read, portMAX_DELAY) == ESP_OK) {
      int s = bytes_read / sizeof(uint32_t);
      for (int i = 0; i < s && idx < SAMPLE_COUNT; i++) {
        samples[idx++] = (int16_t)(raw_buffer[i] >> 14);
      }
    }
  }

  uint32_t ADPCM_SIZE = SAMPLE_COUNT / 2;
  uint8_t* adpcm_data = (uint8_t*) malloc(44 + ADPCM_SIZE);
  writeWavHeader(adpcm_data, ADPCM_SIZE);

  for (int i = 0; i < SAMPLE_COUNT; i += 2) {
    uint8_t b = (adpcm_encode(samples[i]) & 0x0F) | ((adpcm_encode(samples[i + 1]) & 0x0F) << 4);
    adpcm_data[44 + i / 2] = b;
  }

  Serial.println("📤 Uploading WAV...");

  if (!client.connect(server, 80)) {
    Serial.println("❌ Server connect failed");
    free(samples);
    free(adpcm_data);
    return;
  }

  uint32_t totalLen = 44 + ADPCM_SIZE;
  String boundary = "----DogTrackerBoundary";
  String head = "--" + boundary + "\r\n";
  head += "Content-Disposition: form-data; name=\"deviceId\"\r\n\r\n";
  head += deviceId;
  head += "\r\n--" + boundary + "\r\n";
  head += "Content-Disposition: form-data; name=\"file\"; filename=\"audio.wav\"\r\n";
  head += "Content-Type: audio/wav\r\n\r\n";
  String tail = "\r\n--" + boundary + "--\r\n";
  uint32_t contentLength = head.length() + totalLen + tail.length();

  client.println(String("POST ") + soundPath + " HTTP/1.1");
  client.println("Host: " + String(server));
  client.println("Content-Type: multipart/form-data; boundary=" + boundary);
  client.print("Content-Length: ");
  client.println(contentLength);
  client.println();

  client.print(head);
  unsigned long start = millis();
  for (uint32_t i = 0; i < totalLen;) {
    uint32_t len = std::min((uint32_t)256, totalLen - i);
    client.write(&adpcm_data[i], len);
    i += len;

    uint8_t pct = (100 * i) / totalLen;
    Serial.print("\rUploading: [");
    for (int j = 0; j < 20; j++) {
      if (j < pct / 5) Serial.print("#");
      else if (j == pct / 5) Serial.print(">");
      else Serial.print("-");
    }
    Serial.print("] ");
    Serial.print(pct);
    Serial.print("%");
  }
  client.print(tail);
  Serial.println();

  while (client.connected()) {
    if (client.available()) {
      String line = client.readStringUntil('\n');
      Serial.println("📨 Server: " + line);
    }
  }

  Serial.printf("📶 Upload time: %lu ms\n", millis() - start);
  client.stop();
  free(samples);
  free(adpcm_data);
}

void listenAndSendAudio() {
  static uint32_t buffer[BUFFER_SAMPLES];
  size_t bytes_read;

  if (i2s_read(I2S_PORT, buffer, sizeof(buffer), &bytes_read, portMAX_DELAY) == ESP_OK) {
    int samples = bytes_read / sizeof(uint32_t);
    for (int i = 0; i < samples; i++) {
      int32_t sample = buffer[i] >> 8;
      if (abs(sample) > TRIGGER_THRESHOLD) {
        Serial.println("🔊 Sound triggered, recording...");
        recordAndSendCompressedWav();
        Serial.println("🎤 Done. Listening...");
        break;
      }
    }
  }
}

// === GSM ===
void setupGSM() {
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, LOW); delay(1000);
  digitalWrite(MODEM_PWRKEY, HIGH); delay(2000);
  digitalWrite(MODEM_PWRKEY, LOW); delay(3000);

  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(300);
  modem.restart();
  modem.setNetworkMode(38); delay(2000);
  modem.sendAT("+CPSI?");
  String res = modem.stream.readStringUntil('\n');
  Serial.println("📶 CPSI: " + res);

  if (res.indexOf("LTE") == -1) {
    modem.setNetworkMode(2); delay(2000);
  }

  if (modem.gprsConnect(apn)) {
    Serial.println("✅ GPRS connected");
  } else {
    Serial.println("❌ GPRS failed");
    while (1);
  }
}
