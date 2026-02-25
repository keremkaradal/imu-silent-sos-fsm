/*
  IMU Silent SOS 
  ---------------------------
  Gesture: UP -> DOWN -> UP  => SOS

  Summary:
    - Calibrates baseline pitch and gyro bias
    - Learns dominant gyro axis and UP direction during calibration
    - Uses angle + gyro gating to detect UP/DOWN events
    - Verifies UDU sequence with a finite state machine (FSM)
    - Applies timeout and cooldown for robustness

  Hardware:
    - MPU6050 via I2C (Wire)

  Serial:
    '+' / '-' : adjust angle threshold
    'h' / 'g' : adjust gyro threshold
    'p'       : print parameters
    'c'       : recalibrate
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

/* ===================== Parameters ===================== */

struct Config {
  float angEnterDeg = 20.0f;     // angle threshold (deg)
  float angExitDeg  = 8.0f;      // hysteresis (reserved)
  float gyGateDps   = 35.0f;     // gyro gate (deg/s) - updated in calibration
  uint16_t cooldownMs = 2500;    // block retrigger after SOS
  uint16_t timeoutMs  = 2000;    // max time between FSM steps
  uint16_t printMs    = 100;     // debug print period
} cfg;

/* ===================== FSM ===================== */

enum State : uint8_t { IDLE, GOT_UP, GOT_DOWN, SOS_DETECTED, COOLDOWN };
static State state = IDLE;

/* ===================== Globals ===================== */

static Adafruit_MPU6050 mpu;

static float baselinePitchDeg = 0.0f;
static float gyroBiasDps[3]   = {0.0f, 0.0f, 0.0f};

static uint32_t lastEventMs   = 0;
static uint32_t cooldownStart = 0;

static const uint8_t kManualButtonPin = 4;

// Dominant gyro axis selected during calibration (0:X, 1:Y, 2:Z)
static uint8_t activeGyroAxis = 0;

// Direction correction: UP gyro becomes positive after multiplying by upDirSign
static float upDirSign = 1.0f;

// Simple moving average gyro filter
static const uint8_t kFiltSize = 3;
static float gyroFilt[kFiltSize] = {0};
static uint8_t filtIdx = 0;

/* ===================== Helpers ===================== */

static inline float rad2deg(float r) { return r * 57.2957795f; }

static float computePitchDeg(const sensors_event_t &acc) {
  return rad2deg(atan2f(-acc.acceleration.x,
                        sqrtf(acc.acceleration.y * acc.acceleration.y +
                              acc.acceleration.z * acc.acceleration.z)));
}

static float gyroAxisDegPerSec(const sensors_event_t &g, uint8_t axis) {
  float rawRad = (axis == 0) ? g.gyro.x : (axis == 1) ? g.gyro.y : g.gyro.z;
  return rad2deg(rawRad) - gyroBiasDps[axis];
}

static float gyroMovingAverage(float v) {
  gyroFilt[filtIdx] = v;
  filtIdx = (filtIdx + 1) % kFiltSize;

  float sum = 0.0f;
  for (uint8_t i = 0; i < kFiltSize; i++) sum += gyroFilt[i];
  return sum / kFiltSize;
}

static void printParams() {
  Serial.println();
  Serial.println(F("=== Parameters ==="));
  Serial.print(F("angEnterDeg: ")); Serial.println(cfg.angEnterDeg, 1);
  Serial.print(F("gyGateDps:   ")); Serial.println(cfg.gyGateDps, 1);
  Serial.print(F("timeoutMs:   ")); Serial.println(cfg.timeoutMs);
  Serial.print(F("cooldownMs:  ")); Serial.println(cfg.cooldownMs);
  Serial.print(F("axis:        "));
  Serial.println(activeGyroAxis == 0 ? F("X") : (activeGyroAxis == 1 ? F("Y") : F("Z")));
  Serial.print(F("upDirSign:   ")); Serial.println(upDirSign, 1);
  Serial.println(F("=================="));
}

/* ===================== Calibration ===================== */

static void calibrateSystem() {
  Serial.println();
  Serial.println(F("== Calibration =="));

  // (1) Baseline pitch + gyro bias
  Serial.println(F("Hold still (2s)..."));
  delay(2000);

  float sumPitch = 0.0f;
  float sumG[3]  = {0.0f, 0.0f, 0.0f};

  for (int i = 0; i < 100; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    sumPitch += computePitchDeg(a);
    sumG[0]  += rad2deg(g.gyro.x);
    sumG[1]  += rad2deg(g.gyro.y);
    sumG[2]  += rad2deg(g.gyro.z);

    delay(10);
  }

  baselinePitchDeg = sumPitch / 100.0f;
  gyroBiasDps[0]   = sumG[0] / 100.0f;
  gyroBiasDps[1]   = sumG[1] / 100.0f;
  gyroBiasDps[2]   = sumG[2] / 100.0f;

  Serial.print(F("Baseline pitch: "));
  Serial.print(baselinePitchDeg, 2);
  Serial.println(F(" deg"));

  // (2) Learn UP motion: dominant axis + direction + max gyro magnitude
  Serial.println(F("Prepare UP (3s)..."));
  delay(3000);
  Serial.println(F("Move UP now."));

  float maxAbsGy = 0.0f;
  int signVotes[3] = {0, 0, 0};

  for (int i = 0; i < 60; i++) { // ~600 ms
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    float gx = gyroAxisDegPerSec(g, 0);
    float gy = gyroAxisDegPerSec(g, 1);
    float gz = gyroAxisDegPerSec(g, 2);

    if (fabsf(gx) > maxAbsGy) { maxAbsGy = fabsf(gx); activeGyroAxis = 0; }
    if (fabsf(gy) > maxAbsGy) { maxAbsGy = fabsf(gy); activeGyroAxis = 1; }
    if (fabsf(gz) > maxAbsGy) { maxAbsGy = fabsf(gz); activeGyroAxis = 2; }

    if (fabsf(gx) > 20.0f) signVotes[0] += (gx > 0 ? 1 : -1);
    if (fabsf(gy) > 20.0f) signVotes[1] += (gy > 0 ? 1 : -1);
    if (fabsf(gz) > 20.0f) signVotes[2] += (gz > 0 ? 1 : -1);

    delay(10);
  }

  upDirSign = (signVotes[activeGyroAxis] >= 0) ? 1.0f : -1.0f;

  // Optional DOWN (kept as UX step, not used for axis selection)
  Serial.println(F("Prepare DOWN (3s)..."));
  delay(3000);
  Serial.println(F("Move DOWN now."));
  delay(600);

  // Auto gate (60% of observed peak)
  cfg.gyGateDps = maxAbsGy * 0.60f;

  Serial.println(F("Calibration complete."));
  printParams();

  // Reset FSM timing
  state = IDLE;
  lastEventMs = millis();
}

/* ===================== Serial ===================== */

static void handleSerial() {
  if (!Serial.available()) return;

  const char c = (char)Serial.read();
  switch (c) {
    case '+': cfg.angEnterDeg += 3.0f; break;
    case '-': cfg.angEnterDeg = max(10.0f, cfg.angEnterDeg - 3.0f); break;
    case 'h': cfg.gyGateDps   += 5.0f; break;
    case 'g': cfg.gyGateDps   = max(20.0f, cfg.gyGateDps - 5.0f); break;
    case 'p': printParams(); break;
    case 'c': calibrateSystem(); break;
    default: break;
  }
}

/* ===================== Setup / Loop ===================== */

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  pinMode(kManualButtonPin, INPUT_PULLUP);

  if (!mpu.begin()) {
    Serial.println(F("MPU6050 not found."));
    while (1) delay(100);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println(F("MPU6050 ready."));
  calibrateSystem();

  Serial.println(F("System ready: UP -> DOWN -> UP = SOS"));
  Serial.println(F("Commands: + - h g p c"));
}

void loop() {
  const uint32_t now = millis();

  // Manual trigger
  if (digitalRead(kManualButtonPin) == LOW) {
    Serial.println(F("MANUAL SOS"));
    state = COOLDOWN;
    cooldownStart = now;
    delay(500);
  }

  // Read IMU
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  const float pitchDeg = computePitchDeg(a);
  const float dp = pitchDeg - baselinePitchDeg;

  const float rawGy = gyroAxisDegPerSec(g, activeGyroAxis);
  const float gyDps = gyroMovingAverage(rawGy) * upDirSign;

  // Event detection (angle + gyro gate)
  const bool upEvent   = (dp >  cfg.angEnterDeg) && (gyDps >  cfg.gyGateDps);
  const bool downEvent = (dp < -cfg.angEnterDeg) && (gyDps < -cfg.gyGateDps);

  // Timeout between steps
  if ((now - lastEventMs > cfg.timeoutMs) && state != IDLE && state != COOLDOWN) {
    Serial.println(F("TIMEOUT -> IDLE"));
    state = IDLE;
  }

  // FSM
  switch (state) {
    case IDLE:
      if (upEvent) {
        Serial.println(F("UP"));
        state = GOT_UP;
        lastEventMs = now;
      }
      break;

    case GOT_UP:
      if (downEvent) {
        Serial.println(F("DOWN"));
        state = GOT_DOWN;
        lastEventMs = now;
      }
      break;

    case GOT_DOWN:
      if (upEvent) {
        Serial.println(F("SOS DETECTED"));
        state = SOS_DETECTED;
      }
      break;

    case SOS_DETECTED:
      cooldownStart = now;
      state = COOLDOWN;
      break;

    case COOLDOWN:
      if (now - cooldownStart > cfg.cooldownMs) {
        state = IDLE;
        Serial.println(F("COOLDOWN END"));
      }
      break;
  }

  // Debug print
  static uint32_t lastPrint = 0;
  if (now - lastPrint > cfg.printMs) {
    lastPrint = now;

    Serial.print(F("dp:")); Serial.print(dp, 1);
    Serial.print(F(" | gy:")); Serial.print(gyDps, 1);
    Serial.print(F(" | state:"));

    switch (state) {
      case IDLE: Serial.print(F("IDLE")); break;
      case GOT_UP: Serial.print(F("GOT_UP")); break;
      case GOT_DOWN: Serial.print(F("GOT_DOWN")); break;
      case COOLDOWN: Serial.print(F("COOLDOWN")); break;
      default: Serial.print(F("?")); break;
    }
    Serial.println();
  }

  handleSerial();
  delay(10);
}