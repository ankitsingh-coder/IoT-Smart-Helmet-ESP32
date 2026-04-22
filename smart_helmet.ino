#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// -------------------- WiFi Config --------------------
const char* WIFI_SSID = "AAA";
const char* WIFI_PASS = "Acube@123";

// Optional fallback AP if WiFi fails:s
const char* AP_SSID = "SmartHelmet-ESP32";
const char* AP_PASS = "12345678"; // min 8 chars

// -------------------- Pin Config --------------------
static const int PIN_IR_HELMET = 27;   // IR sensor output (ACTIVE LOW)
static const int PIN_MQ3_ADC   = 34;   // MQ3 analog output to ADC
static const int PIN_RELAY     = 26;   // Relay IN (ACTIVE LOW)
static const int PIN_KEYSW     = 25;   // Key switch input

// -------------------- Polarity Config --------------------
static const bool IR_ACTIVE_LOW     = true;  // given
static const bool RELAY_ACTIVE_LOW  = true;  // given
static const bool KEY_ACTIVE_LOW    = true;  // common wiring with INPUT_PULLUP

// -------------------- MQ3 Config --------------------
// NOTE: MQ sensors need warm-up and calibration.
// Start with a conservative threshold and tune with real readings.
static const int   MQ3_THRESHOLD_RAW = 1900;   // 0..4095 (ESP32 ADC). Tune this.
static const float MQ3_SMOOTH_ALPHA  = 0.15f;  // smoothing (0..1)

// -------------------- Accident Detection Config --------------------
static const float IMPACT_G_THRESHOLD   = 3.0f;   // impact threshold in G (tune 2.5..4.0)
static const uint32_t IMPACT_CONFIRM_MS = 120;    // must exceed threshold this long

static const float TILT_DEG_THRESHOLD   = 70.0f;  // severe tilt/fall angle
static const uint32_t TILT_CONFIRM_MS   = 800;    // must stay tilted this long

// Latch accident until reset condition:
static const uint32_t AUTO_RESET_KEY_OFF_MS = 5000; // reset accident latch if key is OFF for 5s

// -------------------- Globals --------------------
WebServer server(80);
Adafruit_MPU6050 mpu;

bool helmetWorn = false;
bool keyOn = false;
bool alcoholDetected = false;

bool accidentLatched = false;
uint32_t accidentTimeMs = 0;

bool motorEnabled = false; // desired motor enable state (relay)
uint32_t lastKeyOffMs = 0;

// MPU readings
float ax=0, ay=0, az=0; // m/s^2
float gx=0, gy=0, gz=0; // rad/s
float tempC=0;

float accelG = 1.0f;
float pitchDeg = 0.0f, rollDeg = 0.0f;

uint32_t impactAboveSince = 0;
uint32_t tiltAboveSince = 0;

// MQ3
float mq3Smooth = 0;

// -------------------- Helpers --------------------
static inline bool readActive(bool activeLow, int pin) {
  int v = digitalRead(pin);
  if (activeLow) return (v == LOW);
  return (v == HIGH);
}

static inline void writeRelay(bool activeLow, int pin, bool on) {
  // on=true means motor enabled
  if (activeLow) digitalWrite(pin, on ? LOW : HIGH);
  else           digitalWrite(pin, on ? HIGH : LOW);
}

String ipToString(IPAddress ip) {
  return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
}

// -------------------- Web UI --------------------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width,initial-scale=1"/>
  <title>Smart Helmet Dashboard</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 16px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    .card { border: 1px solid #ddd; border-radius: 10px; padding: 12px; }
    .ok { color: #0a7; font-weight: 700; }
    .bad { color: #c22; font-weight: 700; }
    .muted { color: #666; }
    button { padding: 10px 12px; border-radius: 10px; border: 1px solid #ccc; background: #f6f6f6; }
  </style>
</head>
<body>
  <h2>Smart Helmet Dashboard</h2>
  <div class="muted">Auto-refresh every 5 seconds</div>
  <div style="height:10px"></div>

  <div class="grid">
    <div class="card">
      <div><b>Helmet Worn:</b> <span id="helmet">-</span></div>
      <div><b>Alcohol Detected:</b> <span id="alcohol">-</span></div>
      <div><b>Key Switch:</b> <span id="key">-</span></div>
      <div><b>Motor Enabled:</b> <span id="motor">-</span></div>
      <div><b>Accident Latched:</b> <span id="accident">-</span></div>
    </div>

    <div class="card">
      <div><b>MQ3 Raw (smoothed):</b> <span id="mq3">-</span></div>
      <div><b>Accel (G):</b> <span id="g">-</span></div>
      <div><b>Pitch/Roll:</b> <span id="pr">-</span></div>
      <div><b>Temp:</b> <span id="temp">-</span></div>
      <div class="muted" id="ip">IP: -</div>
    </div>
  </div>

  <div style="height:12px"></div>
  <button onclick="resetAccident()">Reset Accident (API)</button>

<script>
async function refresh() {
  try {
    const r = await fetch('/api/status');
    const j = await r.json();

    const okbad = (v) => v ? '<span class="ok">YES</span>' : '<span class="bad">NO</span>';
    document.getElementById('helmet').innerHTML = okbad(j.helmetWorn);
    document.getElementById('alcohol').innerHTML = j.alcoholDetected ? '<span class="bad">YES</span>' : '<span class="ok">NO</span>';
    document.getElementById('key').innerHTML = okbad(j.keyOn);
    document.getElementById('motor').innerHTML = j.motorEnabled ? '<span class="ok">ON</span>' : '<span class="bad">OFF</span>';
    document.getElementById('accident').innerHTML = j.accidentLatched ? '<span class="bad">YES</span>' : '<span class="ok">NO</span>';

    document.getElementById('mq3').textContent = j.mq3Smooth.toFixed(0) + ' (thr ' + j.mq3Threshold + ')';
    document.getElementById('g').textContent = j.accelG.toFixed(2);
    document.getElementById('pr').textContent = j.pitchDeg.toFixed(1) + ' / ' + j.rollDeg.toFixed(1);
    document.getElementById('temp').textContent = j.tempC.toFixed(1) + ' °C';
    document.getElementById('ip').textContent = 'IP: ' + j.ip;
  } catch (e) {
    // ignore transient errors
  }
}
async function resetAccident(){
  try { await fetch('/api/reset', {method:'POST'}); } catch(e){}
  refresh();
}
refresh();
setInterval(refresh, 5000);
</script>
</body>
</html>
)HTML";

// -------------------- Logic --------------------
void updateSensors() {
  // Helmet worn (IR active low)
  helmetWorn = readActive(IR_ACTIVE_LOW, PIN_IR_HELMET);

  // Key switch
  keyOn = readActive(KEY_ACTIVE_LOW, PIN_KEYSW);

  // MQ3 read & smooth
  int mqRaw = analogRead(PIN_MQ3_ADC); // 0..4095
  if (mq3Smooth <= 1) mq3Smooth = mqRaw;
  mq3Smooth = (MQ3_SMOOTH_ALPHA * mqRaw) + ((1.0f - MQ3_SMOOTH_ALPHA) * mq3Smooth);
  alcoholDetected = (mq3Smooth >= MQ3_THRESHOLD_RAW);

  // MPU6050
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;

  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;

  tempC = t.temperature;

  // Accel magnitude in G
  float mag = sqrtf(ax*ax + ay*ay + az*az);
  accelG = mag / 9.80665f;

  // Pitch/Roll from accel (simple)
  pitchDeg = atan2f(ax, sqrtf(ay*ay + az*az)) * 57.2958f;
  rollDeg  = atan2f(ay, sqrtf(ax*ax + az*az)) * 57.2958f;
}

void updateAccidentDetection() {
  uint32_t now = millis();

  // Impact detection
  if (accelG >= IMPACT_G_THRESHOLD) {
    if (impactAboveSince == 0) impactAboveSince = now;
    if (!accidentLatched && (now - impactAboveSince >= IMPACT_CONFIRM_MS)) {
      accidentLatched = true;
      accidentTimeMs = now;
    }
  } else {
    impactAboveSince = 0;
  }

  // Tilt / fall detection
  bool tilted = (fabs(pitchDeg) >= TILT_DEG_THRESHOLD) || (fabs(rollDeg) >= TILT_DEG_THRESHOLD);
  if (tilted) {
    if (tiltAboveSince == 0) tiltAboveSince = now;
    if (!accidentLatched && (now - tiltAboveSince >= TILT_CONFIRM_MS)) {
      accidentLatched = true;
      accidentTimeMs = now;
    }
  } else {
    tiltAboveSince = 0;
  }

  // Auto reset latch if key is OFF for a while
  if (!keyOn) {
    if (lastKeyOffMs == 0) lastKeyOffMs = now;
    if (accidentLatched && (now - lastKeyOffMs >= AUTO_RESET_KEY_OFF_MS)) {
      accidentLatched = false;
    }
  } else {
    lastKeyOffMs = 0;
  }
}

void updateMotorControl() {
  // Rule:
  // - If alcohol detected OR helmet not worn => motor should NOT enable even if key ON
  // - If accident detected => stop motor immediately
  bool safeToRun = helmetWorn && !alcoholDetected && !accidentLatched;
  motorEnabled = keyOn && safeToRun;

  writeRelay(RELAY_ACTIVE_LOW, PIN_RELAY, motorEnabled);
}

// -------------------- API Handlers --------------------
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  String ip = (WiFi.status() == WL_CONNECTED) ? ipToString(WiFi.localIP()) : ipToString(WiFi.softAPIP());

  // Manual JSON to avoid extra dependencies
  String json = "{";
  json += "\"helmetWorn\":" + String(helmetWorn ? "true" : "false") + ",";
  json += "\"alcoholDetected\":" + String(alcoholDetected ? "true" : "false") + ",";
  json += "\"keyOn\":" + String(keyOn ? "true" : "false") + ",";
  json += "\"motorEnabled\":" + String(motorEnabled ? "true" : "false") + ",";
  json += "\"accidentLatched\":" + String(accidentLatched ? "true" : "false") + ",";
  json += "\"mq3Smooth\":" + String(mq3Smooth, 1) + ",";
  json += "\"mq3Threshold\":" + String(MQ3_THRESHOLD_RAW) + ",";
  json += "\"accelG\":" + String(accelG, 3) + ",";
  json += "\"pitchDeg\":" + String(pitchDeg, 2) + ",";
  json += "\"rollDeg\":" + String(rollDeg, 2) + ",";
  json += "\"tempC\":" + String(tempC, 2) + ",";
  json += "\"uptimeMs\":" + String(millis()) + ",";
  json += "\"ip\":\"" + ip + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void handleReset() {
  // Only reset if key is OFF (safer), but you can relax this if you want
  if (!keyOn) {
    accidentLatched = false;
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.send(403, "application/json", "{\"ok\":false,\"msg\":\"Turn key OFF to reset\"}");
  }
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 9000) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    // connected
    return;
  }

  // Fallback AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
}

// -------------------- Setup / Loop --------------------
void setup() {
  Serial.begin(115200);

  // Pins
  pinMode(PIN_IR_HELMET, INPUT_PULLUP); // active low sensor
  pinMode(PIN_KEYSW, INPUT_PULLUP);     // active low switch to GND
  pinMode(PIN_RELAY, OUTPUT);

  // Ensure motor OFF at boot
  writeRelay(RELAY_ACTIVE_LOW, PIN_RELAY, false);

  // ADC setup (ESP32 default is okay; read range depends on attenuation)
  analogReadResolution(12); // 0..4095
  // Optionally: analogSetAttenuation(ADC_11db); // wider voltage range (0..~3.3V), depends on board/core

  // MPU6050 init
  Wire.begin(21, 22);
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found. Motor will be kept OFF for safety.");
    accidentLatched = true; // fail-safe: block motor if MPU missing
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // WiFi
  setupWiFi();

  // Web server routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/reset", HTTP_POST, handleReset);

  // Nice-to-have: allow reset via GET too (optional)
  // server.on("/api/reset", HTTP_GET, handleReset);

  server.begin();
}

void loop() {
  server.handleClient();

  // Sensor + logic loop (fast enough, but not insane)
  static uint32_t lastTick = 0;
  uint32_t now = millis();
  if (now - lastTick >= 50) { // 20 Hz
    lastTick = now;

    updateSensors();
    updateAccidentDetection();
    updateMotorControl();
  }
}