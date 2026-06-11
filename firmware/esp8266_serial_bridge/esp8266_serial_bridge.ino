/*
 * car-calib ESP8266 serial actuator bridge
 * -----------------------------------------
 * Same USB-serial line protocol as the ESP32 bridge, but implemented for
 * ESP8266/NodeMCU with Servo.h PWM and watchdog feeding.
 *
 * MiniPC -> ESP8266:
 *   WHO                         -> CARCALIB-ESP8266 v1
 *   CFG {json}                  -> CFGOK
 *   SERVO <angle_deg>
 *   BASE <STATE>                -> FORWARD|BACKWARD|STOP|LOCK|UNLOCK|TURN_LEFT|TURN_RIGHT
 *   RELAY ON|OFF
 *   ESTOP_RESET
 *   PING                        -> PONG
 * ESP8266 -> MiniPC:
 *   CARCALIB-ESP8266 v1
 *   CFGOK / PONG
 *   TEL {json}
 *   ESTOP {json}
 *   LOG <msg>
 */

#include <Arduino.h>
#include <Servo.h>

struct Config {
  int   servoPin       = D5;   // GPIO14, PWM-capable on NodeMCU
  int   minPulseUs     = 500;
  int   maxPulseUs     = 2500;
  float centerAngle    = -8.0f;
  float maxAngleDeg    = 45.0f;
  float deadbandDeg    = 1.0f;
  int   out1           = D1;   // GPIO5
  int   out2           = D2;   // GPIO4
  int   out3           = D6;   // GPIO12
  int   relayPin       = D7;   // GPIO13
  int   powerRelayPin  = D8;   // GPIO15 ignition pulse-to-toggle (CFG overrides)
  unsigned long powerOnPulseMs  = 100;
  unsigned long powerOffPulseMs = 3000;
  int   estopPin       = D3;   // GPIO0, input pull-up friendly
  bool  estopActiveLow = true;
  unsigned long estopStableMs = 20;
  unsigned long blinkRelayMs  = 5000;
  unsigned long telemetryMs = 1000;
};

Config cfg;
Servo servo;
bool servoAttached = false;
bool cfgReceived = false;
float steerAngle = 0.0f;
float lastWrittenAngle = 1e9f;
String baseState = "STOP";
bool relayOn = false;
bool powerPulseActive = false;
unsigned long powerPulseUntilMs = 0;
bool estopActive = false;
unsigned long estopLatchedAtMs = 0;
unsigned long bootMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long blinkUntilMs = 0;
unsigned long lastBlinkToggleMs = 0;
bool blinkRelayState = false;
String rxBuf;

static const float SERVO_ABS_MIN_DEG = -90.0f;
static const float SERVO_ABS_MAX_DEG =  90.0f;

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static float leftLimit() { return cfg.centerAngle - cfg.maxAngleDeg; }
static float rightLimit() { return cfg.centerAngle + cfg.maxAngleDeg; }

static int angleToPulseUs(float angle) {
  float clamped = clampf(angle, leftLimit(), rightLimit());
  float ratio = (clamped - SERVO_ABS_MIN_DEG) / (SERVO_ABS_MAX_DEG - SERVO_ABS_MIN_DEG);
  ratio = clampf(ratio, 0.0f, 1.0f);
  return (int)(cfg.minPulseUs + ratio * (cfg.maxPulseUs - cfg.minPulseUs));
}

static void servoSetup() {
  if (servoAttached) servo.detach();
  servo.attach(cfg.servoPin, cfg.minPulseUs, cfg.maxPulseUs);
  servoAttached = true;
}

static void servoWriteAngle(float angle) {
  if (!servoAttached) return;
  servo.writeMicroseconds(angleToPulseUs(angle));
  steerAngle = angle;
  lastWrittenAngle = angle;
}

static void servoRelease() {
  if (!servoAttached) return;
  servo.detach();
  servoAttached = false;
}

static void baseWrite(int b1, int b2, int b3, const char* label) {
  digitalWrite(cfg.out1, b1 ? HIGH : LOW);
  digitalWrite(cfg.out2, b2 ? HIGH : LOW);
  digitalWrite(cfg.out3, b3 ? HIGH : LOW);
  baseState = label;
}

static void baseStop() { baseWrite(0, 0, 0, "STOP"); }

static bool baseCommand(const String& cmd) {
  if (estopActive && cmd != "STOP") return false;
  if (cmd == "FORWARD")          baseWrite(0, 1, 0, "FORWARD");
  else if (cmd == "BACKWARD")    baseWrite(0, 0, 1, "BACKWARD");
  else if (cmd == "STOP")        baseWrite(0, 0, 0, "STOP");
  else if (cmd == "LOCK")        baseWrite(1, 0, 1, "LOCK");
  else if (cmd == "UNLOCK")      baseWrite(1, 1, 0, "UNLOCK");
  else if (cmd == "TURN_LEFT")   baseWrite(1, 0, 0, "TURN_LEFT");
  else if (cmd == "TURN_RIGHT")  baseWrite(0, 1, 1, "TURN_RIGHT");
  else return false;
  return true;
}

static void relaySet(bool on) {
  relayOn = on;
  digitalWrite(cfg.relayPin, on ? HIGH : LOW);
}

// power relay: non-blocking pulse-to-toggle (ON=short, OFF=long).
static void powerPulse(bool on) {
  unsigned long pulseMs = on ? cfg.powerOnPulseMs : cfg.powerOffPulseMs;
  digitalWrite(cfg.powerRelayPin, HIGH);
  powerPulseActive = true;
  powerPulseUntilMs = millis() + pulseMs;
  Serial.print("LOG power pulse ");
  Serial.print(on ? "ON" : "OFF");
  Serial.print(" ms=");
  Serial.println(pulseMs);
}

static void powerPoll() {
  if (powerPulseActive && millis() >= powerPulseUntilMs) {
    digitalWrite(cfg.powerRelayPin, LOW);
    powerPulseActive = false;
  }
}

static bool estopPinActive() {
  int level = digitalRead(cfg.estopPin);
  return cfg.estopActiveLow ? (level == LOW) : (level == HIGH);
}

static void emitEstop(bool active) {
  Serial.print("ESTOP {\"active\":");
  Serial.print(active ? "true" : "false");
  Serial.print(",\"latched_at_ms\":");
  Serial.print(estopLatchedAtMs);
  Serial.println("}");
}

static void estopLatch() {
  if (estopActive) return;
  estopActive = true;
  estopLatchedAtMs = millis();
  baseStop();
  if (!servoAttached) servoSetup();
  servoWriteAngle(cfg.centerAngle);
  servoRelease();
  blinkUntilMs = millis() + cfg.blinkRelayMs;
  emitEstop(true);
  Serial.println("LOG estop latched");
}

static void estopTryReset() {
  if (!estopActive) { emitEstop(false); return; }
  if (estopPinActive()) {
    Serial.println("LOG estop reset rejected - button still active");
    emitEstop(true);
    return;
  }
  estopActive = false;
  estopLatchedAtMs = 0;
  relaySet(false);
  servoSetup();
  servoWriteAngle(cfg.centerAngle);
  emitEstop(false);
  Serial.println("LOG estop cleared");
}

static void estopPoll() {
  static bool pendingActive = false;
  static unsigned long pendingSince = 0;
  if (estopActive) {
    if (millis() < blinkUntilMs) {
      if (millis() - lastBlinkToggleMs >= 120) {
        lastBlinkToggleMs = millis();
        blinkRelayState = !blinkRelayState;
        digitalWrite(cfg.relayPin, blinkRelayState ? HIGH : LOW);
        relayOn = blinkRelayState;
      }
    } else if (relayOn && blinkRelayState) {
      digitalWrite(cfg.relayPin, LOW);
      relayOn = false;
      blinkRelayState = false;
    }
    return;
  }
  if (estopPinActive()) {
    if (!pendingActive) { pendingActive = true; pendingSince = millis(); }
    else if (millis() - pendingSince >= cfg.estopStableMs) {
      pendingActive = false;
      estopLatch();
    }
  } else {
    pendingActive = false;
  }
}

static void emitTelemetry() {
  unsigned long up = (millis() - bootMs) / 1000UL;
  Serial.print("TEL {\"rpi_online\":true,\"source\":\"esp8266\",\"estop_active\":");
  Serial.print(estopActive ? "true" : "false");
  Serial.print(",\"steer_angle\":");
  Serial.print(steerAngle, 2);
  Serial.print(",\"center_angle\":");
  Serial.print(cfg.centerAngle, 2);
  Serial.print(",\"base_state\":\"");
  Serial.print(baseState);
  Serial.print("\",\"relay_on\":");
  Serial.print(relayOn ? "true" : "false");
  Serial.print(",\"power_pulsing\":");
  Serial.print(powerPulseActive ? "true" : "false");
  Serial.print(",\"pigpio_connected\":true,\"mqtt_connected\":true,\"uptime_s\":");
  Serial.print(up);
  Serial.println("}");
}

static bool jsonNumber(const String& s, const char* key, float& out) {
  String pat = String("\"") + key + "\":";
  int i = s.indexOf(pat);
  if (i < 0) return false;
  i += pat.length();
  int j = i;
  while (j < (int)s.length() && (isdigit(s[j]) || s[j]=='-' || s[j]=='+' || s[j]=='.')) j++;
  if (j == i) return false;
  out = s.substring(i, j).toFloat();
  return true;
}

static bool jsonBool(const String& s, const char* key, bool& out) {
  String pat = String("\"") + key + "\":";
  int i = s.indexOf(pat);
  if (i < 0) return false;
  i += pat.length();
  out = s.startsWith("true", i);
  return true;
}

static void applyConfig(const String& json) {
  float f; bool b;
  if (jsonNumber(json, "servo_pin", f))     cfg.servoPin = (int)f;
  if (jsonNumber(json, "min_pulse_us", f))  cfg.minPulseUs = (int)f;
  if (jsonNumber(json, "max_pulse_us", f))  cfg.maxPulseUs = (int)f;
  if (jsonNumber(json, "center_angle", f))  cfg.centerAngle = f;
  if (jsonNumber(json, "max_angle_deg", f)) cfg.maxAngleDeg = f;
  if (jsonNumber(json, "deadband_deg", f))  cfg.deadbandDeg = f;
  if (jsonNumber(json, "out1", f))          cfg.out1 = (int)f;
  if (jsonNumber(json, "out2", f))          cfg.out2 = (int)f;
  if (jsonNumber(json, "out3", f))          cfg.out3 = (int)f;
  if (jsonNumber(json, "relay_pin", f))     cfg.relayPin = (int)f;
  if (jsonNumber(json, "power_relay_pin", f))   cfg.powerRelayPin = (int)f;
  if (jsonNumber(json, "power_on_pulse_ms", f)) cfg.powerOnPulseMs = (unsigned long)f;
  if (jsonNumber(json, "power_off_pulse_ms", f))cfg.powerOffPulseMs = (unsigned long)f;
  if (jsonNumber(json, "estop_pin", f))     cfg.estopPin = (int)f;
  if (jsonBool(json,   "estop_active_low", b)) cfg.estopActiveLow = b;
  if (jsonNumber(json, "estop_stable_ms", f))  cfg.estopStableMs = (unsigned long)f;
  if (jsonNumber(json, "blink_relay_ms", f))   cfg.blinkRelayMs = (unsigned long)f;
  if (jsonNumber(json, "telemetry_ms", f))     cfg.telemetryMs = (unsigned long)f;

  pinMode(cfg.out1, OUTPUT);
  pinMode(cfg.out2, OUTPUT);
  pinMode(cfg.out3, OUTPUT);
  pinMode(cfg.relayPin, OUTPUT);
  pinMode(cfg.powerRelayPin, OUTPUT);
  digitalWrite(cfg.powerRelayPin, LOW);
  pinMode(cfg.estopPin, cfg.estopActiveLow ? INPUT_PULLUP : INPUT);
  servoSetup();
  baseStop();
  relaySet(false);
  servoWriteAngle(cfg.centerAngle);
  cfgReceived = true;
}

static void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "WHO") { Serial.println("CARCALIB-ESP8266 v1"); return; }
  if (line == "PING") { Serial.println("PONG"); return; }
  if (line.startsWith("CFG ")) {
    applyConfig(line.substring(4));
    Serial.println("CFGOK");
    return;
  }
  if (line.startsWith("SERVO ")) {
    if (estopActive) { Serial.println("LOG servo blocked - estop"); return; }
    if (!servoAttached) servoSetup();
    float a = line.substring(6).toFloat();
    if (fabs(a - lastWrittenAngle) >= cfg.deadbandDeg) servoWriteAngle(a);
    return;
  }
  if (line.startsWith("BASE ")) {
    String cmd = line.substring(5); cmd.trim(); cmd.toUpperCase();
    if (!baseCommand(cmd)) Serial.println("LOG base rejected");
    return;
  }
  if (line.startsWith("RELAY ")) {
    String cmd = line.substring(6); cmd.trim(); cmd.toUpperCase();
    if (cmd == "ON") relaySet(true);
    else if (cmd == "OFF") relaySet(false);
    return;
  }
  if (line.startsWith("POWER ")) {
    if (estopActive) { Serial.println("LOG power blocked - estop"); return; }
    String cmd = line.substring(6); cmd.trim(); cmd.toUpperCase();
    if (cmd == "ON") powerPulse(true);
    else if (cmd == "OFF") powerPulse(false);
    else Serial.println("LOG power bad arg");
    return;
  }
  if (line == "ESTOP_RESET") { estopTryReset(); return; }

  Serial.print("LOG unknown cmd: ");
  Serial.println(line);
}

static void pumpSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') { handleLine(rxBuf); rxBuf = ""; }
    else if (c != '\r') {
      rxBuf += c;
      if (rxBuf.length() > 512) rxBuf = "";
    }
    ESP.wdtFeed();
  }
}

void setup() {
  Serial.begin(115200);
  bootMs = millis();
  pinMode(cfg.out1, OUTPUT);
  pinMode(cfg.out2, OUTPUT);
  pinMode(cfg.out3, OUTPUT);
  pinMode(cfg.relayPin, OUTPUT);
  pinMode(cfg.powerRelayPin, OUTPUT);
  digitalWrite(cfg.powerRelayPin, LOW);
  pinMode(cfg.estopPin, cfg.estopActiveLow ? INPUT_PULLUP : INPUT);
  servoSetup();
  baseStop();
  relaySet(false);
  delay(50);
  Serial.println("CARCALIB-ESP8266 v1");
}

void loop() {
  ESP.wdtFeed();
  pumpSerial();
  estopPoll();
  powerPoll();
  unsigned long now = millis();
  if (now - lastTelemetryMs >= cfg.telemetryMs) {
    lastTelemetryMs = now;
    emitTelemetry();
  }
}
