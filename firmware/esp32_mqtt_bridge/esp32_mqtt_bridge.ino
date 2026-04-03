#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// =========================================================
// WIFI + MQTT
// =========================================================
const char* WIFI_AP_NAME = "ESP32-Car-Setup";
const char* WIFI_AP_PASSWORD = "setup1234";
const unsigned long WIFI_PORTAL_TIMEOUT_S = 180;

const char* MQTT_HOST = "minipc-tc";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USERNAME = "";
const char* MQTT_PASSWORD = "";

const char* MQTT_CLIENT_ID = "esp32-car-bridge";
const char* MQTT_SERVO_TOPIC = "car/servo/angle";
const char* MQTT_BASE_COMMAND_TOPIC = "car/base/command";
const char* MQTT_STATUS_TOPIC = "car/status";

// =========================================================
// PINOUT
// =========================================================
const int SERVO_PIN = 19;
const int BASE_OUT1_PIN = 17;
const int BASE_OUT2_PIN = 18;
const int BASE_OUT3_PIN = 21;

// =========================================================
// SERVO MAPPING
// =========================================================
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const float LOCAL_LEFT_LIMIT = -65.0f;
const float LOCAL_CENTER_ANGLE = -8.0f;
const float LOCAL_RIGHT_LIMIT = 60.0f;

// Vision side default output range from Car_Calib.
// If you publish signed angles directly, change these to -65 / -8 / 60.
const float REMOTE_INPUT_MIN_ANGLE = -65.0f;
const float REMOTE_INPUT_CENTER_ANGLE = -8.0f;
const float REMOTE_INPUT_MAX_ANGLE =  60.0f;

const unsigned long MQTT_RECONNECT_DELAY_MS = 3000;
const unsigned long STATUS_INTERVAL_MS = 5000;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WiFiManager wifiManager;
Servo steeringServo;

float lastServoAngle = LOCAL_CENTER_ANGLE;
unsigned long lastReconnectAttempt = 0;
unsigned long lastStatusAt = 0;


float clampFloat(float value, float low, float high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}


int angleToPulseMicros(float localAngle) {
  float clampedAngle = clampFloat(localAngle, LOCAL_LEFT_LIMIT, LOCAL_RIGHT_LIMIT);
  float ratio = (clampedAngle - LOCAL_LEFT_LIMIT) / (LOCAL_RIGHT_LIMIT - LOCAL_LEFT_LIMIT);
  return static_cast<int>(SERVO_MIN_US + ratio * (SERVO_MAX_US - SERVO_MIN_US));
}


float mapRemoteAngle(float remoteAngle) {
  remoteAngle = clampFloat(remoteAngle, REMOTE_INPUT_MIN_ANGLE, REMOTE_INPUT_MAX_ANGLE);

  if (remoteAngle <= REMOTE_INPUT_CENTER_ANGLE) {
    float remoteSpan = REMOTE_INPUT_CENTER_ANGLE - REMOTE_INPUT_MIN_ANGLE;
    if (remoteSpan <= 0.0f) {
      return LOCAL_CENTER_ANGLE;
    }
    float ratio = (remoteAngle - REMOTE_INPUT_CENTER_ANGLE) / remoteSpan;
    return LOCAL_CENTER_ANGLE + ratio * (LOCAL_CENTER_ANGLE - LOCAL_LEFT_LIMIT);
  }

  float remoteSpan = REMOTE_INPUT_MAX_ANGLE - REMOTE_INPUT_CENTER_ANGLE;
  if (remoteSpan <= 0.0f) {
    return LOCAL_CENTER_ANGLE;
  }
  float ratio = (remoteAngle - REMOTE_INPUT_CENTER_ANGLE) / remoteSpan;
  return LOCAL_CENTER_ANGLE + ratio * (LOCAL_RIGHT_LIMIT - LOCAL_CENTER_ANGLE);
}


void applyBaseCommand(const String& command) {
  if (command == "FORWARD") {
    digitalWrite(BASE_OUT1_PIN, LOW);
    digitalWrite(BASE_OUT2_PIN, LOW);
    digitalWrite(BASE_OUT3_PIN, HIGH);
  } else if (command == "BACKWARD") {
    digitalWrite(BASE_OUT1_PIN, LOW);
    digitalWrite(BASE_OUT2_PIN, HIGH);
    digitalWrite(BASE_OUT3_PIN, LOW);
  } else if (command == "LOCK") {
    digitalWrite(BASE_OUT1_PIN, HIGH);
    digitalWrite(BASE_OUT2_PIN, LOW);
    digitalWrite(BASE_OUT3_PIN, HIGH);
  } else if (command == "UNLOCK") {
    digitalWrite(BASE_OUT1_PIN, HIGH);
    digitalWrite(BASE_OUT2_PIN, HIGH);
    digitalWrite(BASE_OUT3_PIN, LOW);
  } else {
    digitalWrite(BASE_OUT1_PIN, LOW);
    digitalWrite(BASE_OUT2_PIN, LOW);
    digitalWrite(BASE_OUT3_PIN, LOW);
  }

  Serial.print("BASE: ");
  Serial.println(command);
}


void applyServoAngle(float remoteAngle) {
  float localAngle = mapRemoteAngle(remoteAngle);
  int pulseUs = angleToPulseMicros(localAngle);
  steeringServo.writeMicroseconds(pulseUs);
  lastServoAngle = localAngle;

  Serial.print("SERVO remote=");
  Serial.print(remoteAngle, 4);
  Serial.print(" local=");
  Serial.print(localAngle, 4);
  Serial.print(" pulse=");
  Serial.println(pulseUs);
}


void publishStatus() {
  if (!mqttClient.connected()) {
    return;
  }

  String payload = "online servo=" + String(lastServoAngle, 2);
  mqttClient.publish(MQTT_STATUS_TOPIC, payload.c_str(), false);
}


void handleMessage(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; ++i) {
    message += static_cast<char>(payload[i]);
  }
  message.trim();

  String incomingTopic(topic);
  if (incomingTopic == MQTT_SERVO_TOPIC) {
    applyServoAngle(message.toFloat());
    return;
  }

  if (incomingTopic == MQTT_BASE_COMMAND_TOPIC) {
    message.toUpperCase();
    applyBaseCommand(message);
  }
}


void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  Serial.println("WiFi not connected. Starting WiFiManager portal if needed...");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  wifiManager.setConfigPortalBlocking(true);
  wifiManager.setConfigPortalTimeout(WIFI_PORTAL_TIMEOUT_S);
  wifiManager.setHostname(MQTT_CLIENT_ID);

  if (!wifiManager.autoConnect(WIFI_AP_NAME, WIFI_AP_PASSWORD)) {
    Serial.println("WiFiManager portal timed out. Restarting...");
    delay(1000);
    ESP.restart();
  }

  Serial.print("WiFi connected, IP=");
  Serial.println(WiFi.localIP());
}


bool connectMqtt() {
  if (mqttClient.connected()) {
    return true;
  }

  Serial.print("Connecting MQTT...");
  bool connected;
  if (String(MQTT_USERNAME).length() > 0) {
    connected = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD);
  } else {
    connected = mqttClient.connect(MQTT_CLIENT_ID);
  }

  if (!connected) {
    Serial.print("failed, rc=");
    Serial.println(mqttClient.state());
    return false;
  }

  mqttClient.subscribe(MQTT_SERVO_TOPIC);
  mqttClient.subscribe(MQTT_BASE_COMMAND_TOPIC);
  mqttClient.publish(MQTT_STATUS_TOPIC, "online", false);
  Serial.println("connected");
  return true;
}


void setup() {
  Serial.begin(115200);

  pinMode(BASE_OUT1_PIN, OUTPUT);
  pinMode(BASE_OUT2_PIN, OUTPUT);
  pinMode(BASE_OUT3_PIN, OUTPUT);
  applyBaseCommand("STOP");

  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  applyServoAngle(REMOTE_INPUT_CENTER_ANGLE);

  connectWifi();

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(handleMessage);
}


void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi();
  }

  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt >= MQTT_RECONNECT_DELAY_MS) {
      lastReconnectAttempt = now;
      connectMqtt();
    }
  } else {
    mqttClient.loop();
  }

  unsigned long now = millis();
  if (now - lastStatusAt >= STATUS_INTERVAL_MS) {
    lastStatusAt = now;
    publishStatus();
  }
}
