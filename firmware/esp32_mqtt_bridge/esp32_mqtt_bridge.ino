/*
 * car-calib ESP32 serial actuator bridge
 * ----------------------------------------
 * The ESP32 is a dumb-ish actuator driven by the MiniPC over USB-serial.
 * No WiFi, no MQTT. The MiniPC (runtime/esp32_serial_bridge.py) scans
 * serial ports, handshakes with WHO, pushes config with CFG, then streams
 * SERVO/BASE/RELAY/ESTOP_RESET commands. The ESP32 streams TEL telemetry
 * and ESTOP transition events back.
 *
 * Line protocol (newline-terminated, ASCII):
 *   MiniPC -> ESP32:
 *     WHO                         -> reply: CARCALIB-ESP32 v1
 *     CFG {json}                  -> reply: CFGOK   (sets pins/pulse/limits/estop)
 *     SERVO <angle_deg>           -> set servo angle (signed)
 *     BASE <STATE>                -> FORWARD|BACKWARD|STOP|LOCK|UNLOCK|TURN_LEFT|TURN_RIGHT
 *     RELAY ON|OFF
 *     ESTOP_RESET                 -> clear latch if button released
 *     PING                        -> reply: PONG
 *   ESP32 -> MiniPC:
 *     CARCALIB-ESP32 v1           (WHO reply / boot banner)
 *     CFGOK
 *     PONG
 *     TEL {json}                  (periodic telemetry)
 *     ESTOP {json}                (on latch/clear transition)
 *     LOG <msg>                   (diagnostics)
 *
 * Steering convention matches the rest of the project:
 *   more negative angle = LEFT, more positive = RIGHT.
 * Pulse mapping is absolute [-90,90] deg -> [min_pulse_us, max_pulse_us],
 * the input angle is first clamped to [left_limit, right_limit].
 */

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Config (defaults; overridden by CFG from MiniPC)
// ---------------------------------------------------------------------------
struct Config {
  // servo
  int   servoPin       = 12;
  int   minPulseUs     = 500;
  int   maxPulseUs     = 2500;
  float centerAngle    = -8.0f;
  float maxAngleDeg    = 45.0f;   // limits = center +/- this
  float deadbandDeg    = 1.0f;
  // base motor 3-pin
  int   out1           = 17;
  int   out2           = 27;
  int   out3           = 22;
  // relay
  int   relayPin       = 5;
  // estop
  int   estopPin       = 6;
  bool  estopActiveLow = true;
  unsigned long estopStableMs = 20;
  unsigned long blinkRelayMs  = 5000;
  // telemetry
  unsigned long telemetryMs = 1000;
};

Config cfg;
bool   cfgReceived = false;

// ---------------------------------------------------------------------------
// Servo PWM via LEDC
// ---------------------------------------------------------------------------
static const int SERVO_LEDC_CH   = 0;
static const int SERVO_LEDC_FREQ = 50;       // 50 Hz
static const int SERVO_LEDC_BITS = 16;       // 16-bit duty
static const float SERVO_ABS_MIN_DEG = -90.0f;
static const float SERVO_ABS_MAX_DEG =  90.0f;
static const unsigned long SERVO_PERIOD_US = 20000UL; // 50 Hz

float steerAngle = 0.0f;
float lastWrittenAngle = 1e9f;
bool  servoAttached = false;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
String baseState  = "STOP";
bool   relayOn    = false;
bool   estopActive = false;
unsigned long estopLatchedAtMs = 0;
unsigned long bootMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long blinkUntilMs = 0;
unsigned long lastBlinkToggleMs = 0;
bool          blinkRelayState = false;

String rxBuf;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static float leftLimit()  { return cfg.centerAngle - cfg.maxAngleDeg; }
static float rightLimit() { return cfg.centerAngle + cfg.maxAngleDeg; }

static int angleToPulseUs(float angle) {
  float clamped = clampf(angle, leftLimit(), rightLimit());
  float span = SERVO_ABS_MAX_DEG - SERVO_ABS_MIN_DEG;
  float ratio = (clamped - SERVO_ABS_MIN_DEG) / span;
  ratio = clampf(ratio, 0.0f, 1.0f);
  return (int)(cfg.minPulseUs + ratio * (cfg.maxPulseUs - cfg.minPulseUs));
}

static void servoSetup() {
  ledcSetup(SERVO_LEDC_CH, SERVO_LEDC_FREQ, SERVO_LEDC_BITS);
  ledcAttachPin(cfg.servoPin, SERVO_LEDC_CH);
  servoAttached = true;
}

static void servoWriteAngle(float angle) {
  if (!servoAttached) return;
  int pulseUs = angleToPulseUs(angle);
  uint32_t duty = (uint32_t)((uint64_t)pulseUs * 65535UL / SERVO_PERIOD_US);
  ledcWrite(SERVO_LEDC_CH, duty);
  steerAngle = angle;
  lastWrittenAngle = angle;
}

static void servoRelease() {
  if (!servoAttached) return;
  ledcWrite(SERVO_LEDC_CH, 0);  // 0 duty -> no pulse, servo relaxes
}

// base motor: write the 3-pin pattern
static void baseWrite(int b1, int b2, int b3, const char* label) {
  digitalWrite(cfg.out1, b1 ? HIGH : LOW);
  digitalWrite(cfg.out2, b2 ? HIGH : LOW);
  digitalWrite(cfg.out3, b3 ? HIGH : LOW);
  baseState = label;
}

static void baseStop() { baseWrite(0, 0, 0, "STOP"); }

// returns true if command applied, false if blocked/unknown
static bool baseCommand(const String& cmd) {
  // E-stop gate: only STOP passes while latched
  if (estopActive && cmd != "STOP") return false;
  if (cmd == "FORWARD")     baseWrite(0, 1, 0, "FORWARD");
  else if (cmd == "BACKWARD")  baseWrite(0, 0, 1, "BACKWARD");
  else if (cmd == "STOP")      baseWrite(0, 0, 0, "STOP");
  else if (cmd == "LOCK")      baseWrite(1, 0, 1, "LOCK");
  else if (cmd == "UNLOCK")    baseWrite(1, 1, 0, "UNLOCK");
  else if (cmd == "TURN_LEFT") baseWrite(1, 0, 0, "TURN_LEFT");
  else if (cmd == "TURN_RIGHT")baseWrite(0, 1, 1, "TURN_RIGHT");
  else return false;
  return true;
}

static void relaySet(bool on) {
  relayOn = on;
  digitalWrite(cfg.relayPin, on ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
// E-stop
// ---------------------------------------------------------------------------
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
  emitEstop(false);
  Serial.println("LOG estop cleared");
}

static void estopPoll() {
  static bool pendingActive = false;
  static unsigned long pendingSince = 0;
  if (estopActive) {
    // relay blink window for visual warning
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
  // debounce a fresh activation over estopStableMs
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

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------
static void emitTelemetry() {
  unsigned long up = (millis() - bootMs) / 1000UL;
  Serial.print("TEL {\"rpi_online\":true,\"source\":\"esp32\",\"estop_active\":");
  Serial.print(estopActive ? "true" : "false");
  Serial.print(",\"steer_angle\":");
  Serial.print(steerAngle, 2);
  Serial.print(",\"center_angle\":");
  Serial.print(cfg.centerAngle, 2);
  Serial.print(",\"base_state\":\"");
  Serial.print(baseState);
  Serial.print("\",\"relay_on\":");
  Serial.print(relayOn ? "true" : "false");
  Serial.print(",\"pigpio_connected\":true,\"mqtt_connected\":true,\"uptime_s\":");
  Serial.print(up);
  Serial.println("}");
}

// ---------------------------------------------------------------------------
// Config parse (tiny hand-rolled JSON number/bool extractor)
// ---------------------------------------------------------------------------
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
  if (jsonNumber(json, "estop_pin", f))     cfg.estopPin = (int)f;
  if (jsonBool(json,   "estop_active_low", b)) cfg.estopActiveLow = b;
  if (jsonNumber(json, "estop_stable_ms", f))  cfg.estopStableMs = (unsigned long)f;
  if (jsonNumber(json, "blink_relay_ms", f))   cfg.blinkRelayMs = (unsigned long)f;
  if (jsonNumber(json, "telemetry_ms", f))     cfg.telemetryMs = (unsigned long)f;

  // re-init hardware with the new pins
  pinMode(cfg.out1, OUTPUT);
  pinMode(cfg.out2, OUTPUT);
  pinMode(cfg.out3, OUTPUT);
  pinMode(cfg.relayPin, OUTPUT);
  pinMode(cfg.estopPin, cfg.estopActiveLow ? INPUT_PULLUP : INPUT_PULLDOWN);
  servoSetup();
  baseStop();
  relaySet(false);
  servoWriteAngle(cfg.centerAngle);
  cfgReceived = true;
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------
static void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "WHO") {
    Serial.println("CARCALIB-ESP32 v1");
    return;
  }
  if (line == "PING") { Serial.println("PONG"); return; }
  if (line.startsWith("CFG ")) {
    applyConfig(line.substring(4));
    Serial.println("CFGOK");
    return;
  }
  if (line.startsWith("SERVO ")) {
    if (estopActive) { Serial.println("LOG servo blocked - estop"); return; }
    float a = line.substring(6).toFloat();
    // deadband against last written, same source
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
      if (rxBuf.length() > 512) rxBuf = "";  // overflow guard
    }
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  bootMs = millis();
  // apply defaults so the board is safe even before CFG arrives
  pinMode(cfg.out1, OUTPUT);
  pinMode(cfg.out2, OUTPUT);
  pinMode(cfg.out3, OUTPUT);
  pinMode(cfg.relayPin, OUTPUT);
  pinMode(cfg.estopPin, cfg.estopActiveLow ? INPUT_PULLUP : INPUT_PULLDOWN);
  servoSetup();
  baseStop();
  relaySet(false);
  delay(50);
  Serial.println("CARCALIB-ESP32 v1");
}

void loop() {
  pumpSerial();
  estopPoll();
  unsigned long now = millis();
  if (now - lastTelemetryMs >= cfg.telemetryMs) {
    lastTelemetryMs = now;
    emitTelemetry();
  }
}
