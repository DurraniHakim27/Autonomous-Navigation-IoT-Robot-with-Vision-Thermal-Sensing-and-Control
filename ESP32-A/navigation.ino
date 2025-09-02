#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Ewma.h>  

// ====== EWMA Filter ======
Ewma depthFilter(0.08);  // smoothing factor

void resetDepthFilter() {
  depthFilter = Ewma(0.08);  // Reset filter for new object
}

// ====== MLX90614 Setup ======
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
unsigned long lastTempPublish = 0;
bool publishingTemp = false;

// ====== Wi-Fi & MQTT Configuration ======
const char* ssid = "Durrani";
const char* password = "87654321";
const char* mqttServer = "172.20.10.2";
const int mqttPort = 1883;
WiFiClient espClient;
PubSubClient client(espClient);

// ====== MQTT Topics ======
const char* topicDetections = "bbox/all_centers";  // no longer subscribed
const char* topicControl    = "bbox/control";
const char* topicArrived    = "arrived";
const char* topicTemp       = "sensors/temperature";
const char* topicDebug      = "debug/logs";

void logMessage(const String& msg) {
  Serial.println(msg);
  if (client.connected()) {
    client.publish(topicDebug, msg.c_str());
  }
}

// ====== Motor pin definitions ======
#define MF_PWMA 19
#define MF_AI1  32
#define MF_AI2  23
#define MF_PWMB 26
#define MF_BI1  33
#define MF_BI2  25
#define MR_PWMA 27
#define MR_AI1  12
#define MR_AI2  14
#define MR_PWMB 4
#define MR_BI1  13
#define MR_BI2  2

// ====== PWM Settings ======
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;
const int CH_FR = 0;
const int CH_FL = 1;
const int CH_RR = 2;
const int CH_RL = 3;

// ====== Motor direction inversion ======
const int INV_FR = -1;
const int INV_FL = +1;
const int INV_RR = -1;
const int INV_RL = -1;
const int SPEED = 55;

// ====== Tracking state ======
bool running = false;
bool waitingGoHome = false;
bool onReturnPath = false;
bool hasStarted = false;
int currentTargetIndex = 0;
int consecutiveCount = 0;

// ====== Rotation control ======
bool rotating = false;
unsigned long rotateEndMillis = 0;
const unsigned long ROTATE_DURATION = 1000;

// ====== Centering Hysteresis ======
bool adjustingCenter = false;
int adjustingDirection = 0;

// ====== Target object sequences ======
const char* targetsOut[] = {"Cuboid", "Sphere", "Cone", "Cylinder", "plant"};
const int targetsOutCount = 5;
const char* targetsReturn[] = {"Cylinder", "Cone", "Sphere", "Cuboid"};
const int targetsReturnCount = 4;

// ====== Motor function declarations ======
void moveForward();
void moveForwardSlow();
void rotateClockwise();
void rotateAntiClockwise();
void stopMotors();
void setupMotors();
void stopAll();

// ====== Wi-Fi Setup ======
void setup_wifi() {
  delay(10);
  logMessage("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  logMessage("WiFi connected. IP: " + WiFi.localIP().toString());
}

// ====== Handle bbox JSON from Serial ======
void handleBBoxMessage(const String& messageTemp) {
  if (!running) return;

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, messageTemp);
  if (error) {
    logMessage("JSON error: " + String(error.c_str()));
    return;
  }

  const char* currentTarget = onReturnPath
    ? targetsReturn[currentTargetIndex]
    : targetsOut[currentTargetIndex];

  int targetCount = onReturnPath ? targetsReturnCount : targetsOutCount;

  bool found = false;
  float cy = 0.0;
  float cx = 0.0;
  float rawDepth = 0.0;

  JsonArray detections = doc.as<JsonArray>();
  for (JsonObject obj : detections) {
    const char* cls = obj["class"];
    if (String(cls) == currentTarget) {
      found = true;
      cy = obj["cy"];
      cx = obj["cx"];
      rawDepth = obj["depth"];
      break;
    }
  }

  static bool inSlowApproach = false;
  static unsigned long stopTime = 0;

  if (found) {
    float depth = depthFilter.filter(rawDepth);

    char buffer[128];
    snprintf(buffer, sizeof(buffer), "Target: %s | cx: %.1f | cy: %.1f | depth: %.1f (raw: %.1f)", currentTarget, cx, cy, depth, rawDepth);
    logMessage(buffer);

    if (!adjustingCenter) {
      if (cx < 180) {
        rotateClockwise();
        adjustingCenter = true;
        adjustingDirection = -1;
        logMessage("Adjusting: clockwise until cx >= 320");
        return;
      } else if (cx > 460) {
        rotateAntiClockwise();
        adjustingCenter = true;
        adjustingDirection = +1;
        logMessage("Adjusting: counterclockwise until cx <= 320");
        return;
      }
    } else {
      if ((adjustingDirection == -1 && cx >= 320) ||
          (adjustingDirection == +1 && cx <= 320)) {
        stopMotors();
        adjustingCenter = false;
        adjustingDirection = 0;
        logMessage("Adjustment complete. cx centered.");
      } else {
        logMessage("Still adjusting to center...");
        return;
      }
    }

    if (depth > 27.0) {
      moveForward();
      inSlowApproach = false;
      consecutiveCount = 0;
    }
    else if (depth <= 27.0 && depth > 24.0) {
      moveForwardSlow();
      inSlowApproach = true;
      logMessage("Depth ≤ 26 → slow approach");
    }
    else if (depth <= 24.0) {
      if (inSlowApproach) {
        stopMotors();
        logMessage("Depth ≤ 24 → stopping for 5s");
        inSlowApproach = false;
        stopTime = millis();
      }

      if (millis() - stopTime >= 5000) {
        snprintf(buffer, sizeof(buffer), "Arrived at %s", currentTarget);
        logMessage(buffer);

        if (!onReturnPath && currentTargetIndex == targetCount - 1) {
          running = false;
          waitingGoHome = true;
          publishingTemp = true;
          lastTempPublish = millis();
          logMessage("Reached plant. Start temperature monitoring.");
        } else if (onReturnPath && currentTargetIndex == targetCount - 1) {
          client.publish(topicArrived, "home");
          logMessage("Published: home");
          running = false;
        } else {
          currentTargetIndex++;
          resetDepthFilter();
          consecutiveCount = 0;
          snprintf(buffer, sizeof(buffer), "Next target index: %d", currentTargetIndex);
          logMessage(buffer);
        }
      }
    }
  } else {
    unsigned long now = millis();
    static bool pausedAfterRotate = false;
    static unsigned long pauseStart = 0;

    if (!rotating && !pausedAfterRotate) {
      rotateClockwise();
      rotateEndMillis = now + ROTATE_DURATION;
      rotating = true;
      char buffer[100];
      snprintf(buffer, sizeof(buffer), "Target '%s' not found → rotating for 1 second", currentTarget);
      logMessage(buffer);

    }
    else if (rotating && now >= rotateEndMillis) {
      stopMotors();
      rotating = false;
      pausedAfterRotate = true;
      pauseStart = now;
      logMessage("Rotation done → pausing for 2 seconds to scan");
    }
    else if (pausedAfterRotate && now - pauseStart >= 2000) {
      pausedAfterRotate = false;
      logMessage("Pause complete → scanning again");
    }
  }
}

// ====== MQTT Callback (only for control) ======
void callback(char* topic, byte* payload, unsigned int length) {
  String messageTemp;
  for (unsigned int i = 0; i < length; i++) {
    messageTemp += (char)payload[i];
  }
  messageTemp.trim();

  if (strcmp(topic, topicControl) == 0) {
    if (messageTemp == "START" && !running && !waitingGoHome) {
      running = true;
      if (!hasStarted) {
        hasStarted = true;
        onReturnPath = false;
        currentTargetIndex = 0;
        consecutiveCount = 0;
        logMessage("START: Begin outward sequence");
      } else {
        logMessage("START: Resuming sequence");
      }
    } else if (messageTemp == "PAUSE" && running) {
      running = false;
      stopMotors();
      logMessage("PAUSE: Movement halted");
    } else if (messageTemp == "GO_HOME" && waitingGoHome) {
      waitingGoHome = false;
      running = true;
      onReturnPath = true;
      currentTargetIndex = 0;
      consecutiveCount = 0;
      logMessage("GO_HOME: Begin return sequence");
    }
  }
}

// ====== MQTT Reconnect ======
void reconnect() {
  while (!client.connected()) {
    logMessage("Connecting to MQTT...");
    if (client.connect("ESP32Client")) {
      logMessage("MQTT connected");
      client.subscribe(topicControl);  // only control now
    } else {
      logMessage("Failed MQTT, rc=" + String(client.state()) + ". Retry in 5s");
      delay(5000);
    }
  }
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  setup_wifi();
  setupMotors();
  mlx.begin();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
}

// ====== Loop ======
void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  // ==== Read JSON messages from Raspberry Pi via USB Serial ====
  static String serialBuffer = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      serialBuffer.trim();
      if (serialBuffer.length() > 0) {
        handleBBoxMessage(serialBuffer);
      }
      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }

  if (publishingTemp && millis() - lastTempPublish >= 1000) {
    lastTempPublish = millis();
    float tempC = mlx.readObjectTempC();
    if (!isnan(tempC)) {
      StaticJsonDocument<64> doc;
      doc["average_c"] = tempC;
      doc["timestamp"] = millis() / 1000;
      char buffer[64];
      serializeJson(doc, buffer);
      client.publish(topicTemp, buffer);
      logMessage("Published Temp: " + String(buffer));

      if (tempC <= 25.0) {
        publishingTemp = false;
        client.publish(topicArrived, "temperature");
        logMessage("Temp ≤ 25°C → Start return trip");
        waitingGoHome = false;
        running = true;
        onReturnPath = true;
        currentTargetIndex = 0;
        resetDepthFilter();
        consecutiveCount = 0;
      }
    } else {
      logMessage("Failed to read from MLX90614");
    }
  }
}

// ====== Motor Control Implementation ======
void setupMotors() {
  pinMode(MF_AI1, OUTPUT); pinMode(MF_AI2, OUTPUT);
  pinMode(MF_BI1, OUTPUT); pinMode(MF_BI2, OUTPUT);
  pinMode(MR_AI1, OUTPUT); pinMode(MR_AI2, OUTPUT);
  pinMode(MR_BI1, OUTPUT); pinMode(MR_BI2, OUTPUT);

  ledcSetup(CH_FR, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(MF_PWMA, CH_FR);
  ledcSetup(CH_FL, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(MF_PWMB, CH_FL);
  ledcSetup(CH_RR, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(MR_PWMA, CH_RR);
  ledcSetup(CH_RL, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(MR_PWMB, CH_RL);

  stopAll();
}

void setMotorRaw(int pinIn1, int pinIn2, int pwmChannel, int dir, int speed) {
  digitalWrite(pinIn1, dir > 0 ? HIGH : LOW);
  digitalWrite(pinIn2, dir < 0 ? HIGH : LOW);
  ledcWrite(pwmChannel, dir == 0 ? 0 : speed);
}

void setMotor(int pinIn1, int pinIn2, int pwmChannel, int intendedDir, int speed, int invFactor) {
  setMotorRaw(pinIn1, pinIn2, pwmChannel, intendedDir * invFactor, speed);
}

void stopAll() {
  setMotor(MF_AI1, MF_AI2, CH_FR, 0, 0, INV_FR);
  setMotor(MF_BI1, MF_BI2, CH_FL, 0, 0, INV_FL);
  setMotor(MR_AI1, MR_AI2, CH_RR, 0, 0, INV_RR);
  setMotor(MR_BI1, MR_BI2, CH_RL, 0, 0, INV_RL);
}

void moveForward() {
  setMotor(MF_AI1, MF_AI2, CH_FR, +1, SPEED, INV_FR);
  setMotor(MF_BI1, MF_BI2, CH_FL, +1, SPEED, INV_FL);
  setMotor(MR_AI1, MR_AI2, CH_RR, +1, SPEED, INV_RR);
  setMotor(MR_BI1, MR_BI2, CH_RL, +1, SPEED, INV_RL);
}

void moveForwardSlow() {
  int slowSpeed = SPEED / 2;
  setMotor(MF_AI1, MF_AI2, CH_FR, +1, slowSpeed, INV_FR);
  setMotor(MF_BI1, MF_BI2, CH_FL, +1, slowSpeed, INV_FL);
  setMotor(MR_AI1, MR_AI2, CH_RR, +1, slowSpeed, INV_RR);
  setMotor(MR_BI1, MR_BI2, CH_RL, +1, slowSpeed, INV_RL);
}

void rotateClockwise() {
  setMotor(MF_AI1, MF_AI2, CH_FR, +1, SPEED, INV_FR);
  setMotor(MF_BI1, MF_BI2, CH_FL, -1, SPEED, INV_FL);
  setMotor(MR_AI1, MR_AI2, CH_RR, +1, SPEED, INV_RR);
  setMotor(MR_BI1, MR_BI2, CH_RL, -1, SPEED, INV_RL);
}

void rotateAntiClockwise() {
  setMotor(MF_AI1, MF_AI2, CH_FR, -1, SPEED, INV_FR);
  setMotor(MF_BI1, MF_BI2, CH_FL, +1, SPEED, INV_FL);
  setMotor(MR_AI1, MR_AI2, CH_RR, -1, SPEED, INV_RR);
  setMotor(MR_BI1, MR_BI2, CH_RL, +1, SPEED, INV_RL);
}

void stopMotors() {
  stopAll();
}

