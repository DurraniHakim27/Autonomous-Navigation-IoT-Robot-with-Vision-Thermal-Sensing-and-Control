// dbf: MQTT + DHT11 + Blynk — consistent updates to both LCD and Blynk

#define BLYNK_TEMPLATE_ID "TMPL6zN5WnJF3"
#define BLYNK_TEMPLATE_NAME "Temp Control System"
#define BLYNK_AUTH_TOKEN "UKkt5xRMRxs2rc2JW7GRiHDFWMHT3Qff"
#define BLYNK_PRINT Serial

#include <LiquidCrystal.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <BlynkSimpleEsp32.h>

// WiFi + Blynk
char blynkAuth[] = BLYNK_AUTH_TOKEN;
const char* ssid = "Durrani";
const char* pass = "87654321";

// MQTT
const char* mqtt_server = "172.20.10.2";
const int mqttPort = 1883;
const char* mqtt_topic = "sensors/temperature";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
BlynkTimer timer;

// LCD setup
LiquidCrystal lcd(16,17,5,18,19,21);
byte customChar[8] = {0b01110,0b01010,0b01110,0,0,0,0,0};

#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define BUZZER_PIN   4
#define FAN_PWM_PIN 14
#define RELAY_PIN   12
const int PWM_FREQ = 25000, PWM_RES = 8;

int currentPWM = 0;
String StationStatus = "Normal";
float temperature = NAN, lastAppliedTemp = NAN;

StaticJsonDocument<200> jsonDoc;
const int MAX_MQTT_TRIES = 5;
int mqttTries = 0;

void clearLine(int row) {
  lcd.setCursor(0, row);
  lcd.print(F("                "));
  lcd.setCursor(0, row);
}

void mqtt_callback(char* topic, byte* payload, unsigned int len) {
  String msg;
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  auto err = deserializeJson(jsonDoc, msg);
  if (!err && jsonDoc.containsKey("average_c")) {
    temperature = jsonDoc["average_c"].as<float>();
    Serial.printf("➡️ MQTT temp: %.2f°C\n", temperature);
  } else {
    Serial.println("JSON parse error or missing key");
  }
}

void mqtt_reconnect(){
  mqttTries = 0;
  while (!mqttClient.connected() && mqttTries < MAX_MQTT_TRIES) {
    Serial.printf("MQTT attempt %d\n", mqttTries + 1);
    if (mqttClient.connect("ESP32Client")) {
      Serial.println("MQTT connected");
      mqttClient.subscribe(mqtt_topic);
      return;
    }
    mqttTries++;
    delay(2000);
  }
  if (mqttTries >= MAX_MQTT_TRIES) {
    Serial.println("MQTT failed — fallback on DHT, but keep WiFi");
  }
}

void updateFanPWM(int targetPWM) {
  digitalWrite(RELAY_PIN, targetPWM > 0 ? HIGH : LOW);
  currentPWM = targetPWM;
  ledcWrite(FAN_PWM_PIN, currentPWM);
}

void applyControl() {
  float t = temperature;
  if (isnan(t)) {
    t = dht.readTemperature();
    if (isnan(t)) {
      Serial.println("❌ DHT read failed");
      return;
    }
    Serial.printf("📟 Using DHT temp: %.1f°C\n", t);
  } else {
    Serial.printf("🟢 Using MQTT temp: %.1f°C\n", t);
  }

  bool changed = false;
  if (t != lastAppliedTemp) changed = true;
  if (changed) {
    lastAppliedTemp = t;
    Serial.println("🔁 Applying new control");
    StationStatus = "NORMAL";
    noTone(BUZZER_PIN);

    int targetPWM = 0;
    if (t > 39) {
      targetPWM = 255; StationStatus = "OVERHEATING"; tone(BUZZER_PIN, 3000);
    } else if (t > 30) targetPWM = 180;
    else if (t > 28) targetPWM = 85;

    updateFanPWM(targetPWM);

    lcd.setCursor(0, 0);
    lcd.print("T:"); lcd.print(t, 1);
    clearLine(1);
    lcd.print("SS: "); lcd.print(StationStatus);

    Serial.printf("🔥 Temp=%.1f°C | PWM=%d | %s\n", t, currentPWM, StationStatus.c_str());

    sendToBlynk();  // Push to Blynk immediately on change
  }
}

void sendToBlynk() {
  if (!isnan(lastAppliedTemp)) {
    Blynk.virtualWrite(V0, lastAppliedTemp);
    Blynk.virtualWrite(V1,
      StationStatus == "OVERHEATING" ? "MAX" :
      (currentPWM == 180 ? "MEDIUM" :
      (currentPWM == 85 ? "LOW" : "OFF"))
    );
    Blynk.virtualWrite(V2, currentPWM);
    Blynk.virtualWrite(V3, (currentPWM / 255.0) * 100.0);
    Blynk.virtualWrite(V4, StationStatus);
  }
}

void setup(){
  Serial.begin(115200);
  dht.begin();

  WiFi.begin(ssid, pass);
  Serial.print("Connecting Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  Blynk.begin(blynkAuth, ssid, pass);

  mqttClient.setServer(mqtt_server, mqttPort);
  mqttClient.setCallback(mqtt_callback);

  lcd.begin(16, 2);
  lcd.createChar(0, customChar);

  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, LOW);

  ledcAttach(FAN_PWM_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(FAN_PWM_PIN, currentPWM);

  // Timer ensures periodic data in case no change for long
  timer.setInterval(5000L, sendToBlynk);
}

void loop(){
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) mqtt_reconnect();
    mqttClient.loop();
  }
  Blynk.run();
  timer.run();

  applyControl();
  delay(200);
}
