# Intership25
IoT‑Based Real‑Time Air Quality Monitoring and Automated Dust Suppression System for Mining Environments
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <PMS.h>

#define MQ135_PIN   34
#define MQ136_PIN   35
#define RELAY_PUMP   4
#define RELAY_FAN   27
#define LED_PIN      2
#define LORA_SS      5
#define LORA_RST    14
#define LORA_DIO0   26

HardwareSerial pmsSerial(2); // UART2: RX2 = GPIO16, TX2 = GPIO17
PMS pms(pmsSerial);
PMS::DATA pmsData;

const char* ssid = "Navyalaxmi13";
const char* password = "navya#13";
const char* serverURL = "https://mining-innovation-lab.onrender.com/air-monitor-data";

const float RL = 10000.0;
float R0_MQ135 = 10000.0;
bool mq135Calibrated = false;
unsigned long mq135CalibStart = 0;
float rsSumMQ135 = 0;
int sampleCountMQ135 = 0;

unsigned long lastFanTime = 0;
const unsigned long fanInterval = 60000;
const unsigned long fanBlastTime = 30000;
unsigned long mistEndTime = 0;
bool mistActive = false;
const unsigned long mistDuration = 30000;

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PUMP, OUTPUT);
  pinMode(RELAY_FAN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RELAY_PUMP, LOW);
  digitalWrite(RELAY_FAN, LOW);
  digitalWrite(LED_PIN, LOW);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("❌ LoRa init failed");
    while (1);
  }
  Serial.println("✅ LoRa TX Ready");

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected");
  } else {
    Serial.println("\n❌ WiFi not connected");
  }

  mq135CalibStart = millis();
  pmsSerial.begin(9600, SERIAL_8N1, 16, 17);  // PMS UART
  pms.passiveMode();
  pms.wakeUp();
}

void loop() {
  if (!mq135Calibrated) {
    calibrateMQ135();
    return;
  }

  float nh3_ppm = 0, co2_ppm = 0, so2_ppm = 0;
  readMQ135(nh3_ppm, co2_ppm);
  so2_ppm = readMQ136();
  int pm25 = readPMS_PM25();
  if (pm25 < 0) pm25 = 0;

  Serial.printf("NH3: %.1f ppm | CO2: %.1f ppm | SO2: %.2f ppm | PM2.5: %d µg/m³\n",
                nh3_ppm, co2_ppm, so2_ppm, pm25);

  bool danger = (nh3_ppm > 25 || co2_ppm > 1000 || so2_ppm > 0.5 || pm25 > 60);
  unsigned long now = millis();

  if (danger && !mistActive) {
    digitalWrite(RELAY_PUMP, HIGH);
    digitalWrite(RELAY_FAN, HIGH);
    mistEndTime = now + mistDuration;
    mistActive = true;
  }

  if (mistActive && now >= mistEndTime) {
    digitalWrite(RELAY_PUMP, LOW);
    digitalWrite(RELAY_FAN, LOW);
    mistActive = false;
  }

  if (now - lastFanTime > fanInterval) {
    digitalWrite(RELAY_FAN, HIGH);
    delay(fanBlastTime);
    digitalWrite(RELAY_FAN, LOW);
    lastFanTime = now;
  }

  sendLoRaPacket(nh3_ppm, co2_ppm, so2_ppm, pm25);
  sendSensorData(nh3_ppm, co2_ppm, so2_ppm, pm25);

  digitalWrite(LED_PIN, HIGH); delay(100); digitalWrite(LED_PIN, LOW);
  delay(1000);
}

// === MQ135 Calibration ===
void calibrateMQ135() {
  int adc = analogRead(MQ135_PIN);
  float voltage = adc * (3.3 / 4095.0);
  float rs = (voltage > 0.1) ? (3.3 - voltage) * RL / voltage : RL;
  rsSumMQ135 += rs;
  sampleCountMQ135++;

  if (millis() - mq135CalibStart > 30000) {
    R0_MQ135 = (rsSumMQ135 / sampleCountMQ135) / 3.6;
    mq135Calibrated = true;
    Serial.printf("✅ MQ135 Calibrated: R0 = %.1f\n", R0_MQ135);
  } else {
    delay(500);
  }
}

// === MQ135 Reading ===
void readMQ135(float &nh3_ppm, float &co2_ppm) {
  int adc = analogRead(MQ135_PIN);
  float voltage = adc * (3.3 / 4095.0);
  if (voltage < 0.1) {
    nh3_ppm = 0;
    co2_ppm = 400;
    return;
  }

  float rs = (3.3 - voltage) * RL / voltage;
  float ratio = rs / R0_MQ135;
  nh3_ppm = 102.2 * pow(ratio, -2.473);
  float co2_raw = 116.6020682 * pow(ratio, -2.769034857);
  co2_ppm = 400 + (co2_raw * 25); // adjusted scale
}

// === MQ136 Reading ===
float readMQ136() {
  int adc = analogRead(MQ136_PIN);
  float voltage = adc * (3.3 / 4095.0);
  if (voltage < 0.1) return 0;

  float rs = (3.3 - voltage) * RL / voltage;
  float ratio = rs / 3000.0;
  return 3.027 * pow(ratio, -1.69);
}

// === PMS7003 PM2.5 Reading ===
int readPMS_PM25() {
  pms.requestRead();
  if (pms.readUntil(pmsData)) {
    return pmsData.PM_AE_UG_2_5;
  } else {
    Serial.println("⚠️ PMS read failed");
    return -1;
  }
}

// === LoRa Transmit ===
void sendLoRaPacket(float nh3, float co2, float so2, int pm25) {
  LoRa.beginPacket();
  LoRa.printf("%.1f,%.0f,%.2f,%d", nh3, co2, so2, pm25);
  LoRa.endPacket();
}

// === HTTP POST to Cloud ===
void sendSensorData(float nh3, float co2, float so2, int pm25) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<200> doc;
    doc["nh3"] = nh3;
    doc["co2"] = co2;
    doc["so2"] = so2;
    doc["pm25"] = pm25;

  String jsonString;
    serializeJson(doc, jsonString);

  int httpResponseCode = http.POST(jsonString);
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("🌐 HTTP " + String(httpResponseCode) + " → " + response);
    } else {
      Serial.println("❌ HTTP Error: " + String(httpResponseCode));
    }

   http.end();
  } else {
    Serial.println("❌ WiFi not connected!");
  }
}
