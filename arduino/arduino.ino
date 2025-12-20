#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include "esp_camera.h"
#include "board_config.h"
#include <HTTPClient.h>

// Declarations from the stock camera server (app_httpd.cpp)
void startCameraServer();
void setupLedFlash();

// ---------- WiFi ----------
const char *ssid     = "Taa_Deco";
const char *password = "Taa2025!";

// ---------- AI Model Server ----------
const char *serverUrl = "http://192.168.68.107:5000/upload";

// ---------- Sensors ----------
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ---------- I2C ----------
#define SDA_PIN 14
#define SCL_PIN 15
Adafruit_ADS1115 ads;

// ---------- LCD ----------
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------- Relay / Pump ----------
const int relayPin = 13;
const bool RELAY_ACTIVE_LOW = true;

// ---------- Moisture calibration ----------
const float Vdry = 2.50f;
const float Vwet = 1.00f;

// ---------- Control thresholds ----------
const float moistureOnThreshold  = 45.0f;
const float moistureOffThreshold = 55.0f;

// ---------- Timing ----------
unsigned long lastSampleMs = 0;
unsigned long lastLcdMs    = 0;
unsigned long lastUploadMs = 0;
const unsigned long sampleIntervalMs = 1000;
const unsigned long lcdIntervalMs    = 2000;
const unsigned long uploadIntervalMs = 10000;  // send image every 10s

// ---------- State ----------
float temperatureC = NAN;
float humidityPct  = NAN;
float moisturePct  = NAN;
float moistureVoltage = NAN;
bool pumpOn = false;

// ---------- Camera ----------
sensor_t *camSensor = nullptr;

String pumpStatusText() { return pumpOn ? "ON" : "OFF"; }

void setRelay(bool on) {
  pumpOn = on;
  digitalWrite(relayPin, RELAY_ACTIVE_LOW ? (on ? LOW : HIGH) : (on ? HIGH : LOW));
}

void sampleSensors() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) temperatureC = t;
  if (!isnan(h)) humidityPct  = h;

  int16_t raw = ads.readADC_SingleEnded(0);
  moistureVoltage = raw * 0.000125f;

  float pct = (Vdry - moistureVoltage) / (Vdry - Vwet) * 100.0f;
  moisturePct = constrain(pct, 0, 100);

  if (!pumpOn && moisturePct < moistureOnThreshold) setRelay(true);
  else if (pumpOn && moisturePct > moistureOffThreshold) setRelay(false);

  Serial.printf("Temp: %.2f C  Hum: %.2f %%  Moist: %.2f %%  V: %.3f V  Pump: %s\n",
                temperatureC, humidityPct, moisturePct, moistureVoltage, pumpStatusText().c_str());
}

void updateLcd() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.printf("T:%.1fC H:%.0f%%", temperatureC, humidityPct);
  lcd.setCursor(0,1);
  lcd.printf("M:%.0f%% P:%s", moisturePct, pumpOn ? "ON" : "OFF");
}

void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  // Pins from board_config.h
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;   // keep working format
  config.frame_size   = FRAMESIZE_QVGA;
  config.fb_count     = 1;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true) { delay(1000); }
  }

  camSensor = esp_camera_sensor_get();

#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif
}

// ---------- Upload image to AI server ----------
void uploadImage() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Convert RGB565 frame to JPEG using software encoder
  size_t jpg_buf_len = 0;
  uint8_t * jpg_buf = nullptr;
  bool converted = frame2jpg(fb, 80, &jpg_buf, &jpg_buf_len); // quality 80

  if (converted) {
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "image/jpeg");

    int httpResponseCode = http.POST(jpg_buf, jpg_buf_len);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("AI Response: " + response);
    } else {
      Serial.printf("Error sending image: %d\n", httpResponseCode);
    }

    http.end();
    free(jpg_buf);
  } else {
    Serial.println("JPEG conversion failed");
  }

  esp_camera_fb_return(fb);
}

void setup() {
  Serial.begin(115200);

  pinMode(relayPin, OUTPUT);
  setRelay(false);

  dht.begin();
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!ads.begin(0x48)) {
    Serial.println("ADS1115 not found!");
    while (1) { delay(1000); }
  }
  ads.setGain(GAIN_ONE);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Smart Garden");
  lcd.setCursor(0,1);
  lcd.print("Booting...");

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println();
  Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());

  if (MDNS.begin("esp32")) {
    Serial.println("mDNS: http://esp32.local/");
  }

  setupCamera();
  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  sampleSensors();
  updateLcd();
  lastSampleMs = millis();
  lastLcdMs    = millis();
  lastUploadMs = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastSampleMs >= sampleIntervalMs) {
    lastSampleMs = now;
    sampleSensors();
  }
  if (now - lastLcdMs >= lcdIntervalMs) {
    lastLcdMs = now;
    updateLcd();
  }
  if (now - lastUploadMs >= uploadIntervalMs) {
    lastUploadMs = now;
    uploadImage();   // send image to AI model
  }
}
