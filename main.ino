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
#include "CameraWebServer.h"   // our camera init helper

// ---------- WiFi ----------
const char *ssid     = "Airbox-B261";
const char *password = "d5n6s8X3";

// ---------- Sensors ----------
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ---------- I2C ----------
#define SDA_PIN 14
#define SCL_PIN 15
Adafruit_ADS1115 ads; // ADDR=GND -> 0x48

// ---------- LCD ----------
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------- Relay / Pump ----------
const int relayPin = 13;   // SAFE pin for ESP32-CAM; active-LOW relay input
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
const unsigned long sampleIntervalMs = 1000;
const unsigned long lcdIntervalMs    = 2000;

// ---------- State ----------
float temperatureC = NAN;
float humidityPct  = NAN;
float moisturePct  = NAN;
float moistureVoltage = NAN;
bool pumpOn = false;

// ---------- Web ----------
WebServer server(80);

String pumpStatusText() {
  return pumpOn ? "ON" : "OFF";
}

void setRelay(bool on) {
  pumpOn = on;
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(relayPin, on ? LOW : HIGH);
  } else {
    digitalWrite(relayPin, on ? HIGH : LOW);
  }
}

void handleRoot() {
  char msg[2000];
  snprintf(msg, sizeof(msg),
    "<html>\
    <head>\
      <meta http-equiv='refresh' content='4'/>\
      <meta name='viewport' content='width=device-width, initial-scale=1'>\
      <title>ESP32 Smart Garden</title>\
    </head>\
    <body>\
      <h2>ESP32 Smart Garden</h2>\
      <p>Temp: %.2f &deg;C</p>\
      <p>Humidity: %.2f %%</p>\
      <p>Moisture: %.2f %%</p>\
      <p>Pump: <span style='color:%s;'>%s</span></p>\
      <p>Moisture voltage: %.3f V</p>\
      <p><a href='http://%s:81/'>Camera Stream</a></p>\
    </body>\
    </html>",
    isnan(temperatureC) ? -1.0f : temperatureC,
    isnan(humidityPct)  ? -1.0f : humidityPct,
    isnan(moisturePct)  ? -1.0f : moisturePct,
    pumpOn ? "#d40000" : "#006600",
    pumpStatusText().c_str(),
    isnan(moistureVoltage) ? -1.0f : moistureVoltage,
    WiFi.localIP().toString().c_str()
  );
  server.send(200, "text/html", msg);
}

void sampleSensors() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) temperatureC = t;
  if (!isnan(h)) humidityPct  = h;

  int16_t raw = ads.readADC_SingleEnded(0);
  moistureVoltage = raw * 0.000125f;
  float pct = (Vdry - moistureVoltage) / (Vdry - Vwet) * 100.0f;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  moisturePct = pct;

  if (!pumpOn && moisturePct < moistureOnThreshold) {
    setRelay(true);
  } else if (pumpOn && moisturePct > moistureOffThreshold) {
    setRelay(false);
  }

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
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  if (MDNS.begin("esp32")) {
    Serial.println("mDNS responder started: http://esp32.local/");
  }
  server.on("/", handleRoot);
  server.begin();
  Serial.println("HTTP server started");

  // ---------- Camera init ----------
  if (!initCamera()) {
    Serial.println("Camera init failed");
    while(true);
  }
  startCameraServer();  // camera stream on port 81
  Serial.printf("Camera Ready! http://%s:81/\n", WiFi.localIP().toString().c_str());

  // Initial sample and display
  sampleSensors();
  updateLcd();
  lastSampleMs = millis();
  lastLcdMs    = millis();
}

void loop() {
  server.handleClient();

  unsigned long now = millis();
  if (now - lastSampleMs >= sampleIntervalMs) {
    lastSampleMs = now;
    sampleSensors();
  }
  if (now - lastLcdMs >= lcdIntervalMs) {
    lastLcdMs = now;
    updateLcd();
  }
}
