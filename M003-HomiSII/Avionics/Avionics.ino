/* ============================================================================
   MODEL ROCKET AVIONICS  —  ESP32 flight computer
   ----------------------------------------------------------------------------
   Functions:
     1. Log altitude + acceleration (and more) to SD card at ~20 Hz.
     2. Compute vertical velocity three ways: barometric derivative,
        accelerometer integral, and a complementary-filter fusion (the
        "most accurate" estimate — accel gives low-noise short term, baro
        anchors long term to kill integration drift).
     3. Detect apogee from the (fused) velocity + a sustained altitude drop.
     4. At apogee: swing the servo 0deg -> 90deg (deploy) and sound the buzzer.
        On landing: continuous buzzer beacon.

   SAFETY MODEL  (servo is the SOLE recovery deployment — no motor ejection):
     - Hardware arm switch cuts the servo's 5V. Until it's flipped, the servo
       is physically incapable of moving. Everything else stays powered and
       logging on the pad. Arm it LAST, through the access cutout.
     - BOOST LOCKOUT: apogee detection is structurally impossible until the
       state machine has seen BOOST -> COAST *and* a minimum time has elapsed.
       This is the single most important safeguard against a high-speed
       deployment during powered flight or the burnout transient.
     - BACKUP TIMER: if no barometric apogee is detected, a timed deploy fires
       shortly after predicted apogee so a baro failure can't cause a lawn dart.
       ****  TUNE BACKUP_DEPLOY_MS to just AFTER your simulated apogee time.  ****
     - Defensive init: if a sensor or the SD card fails to start, the board
       beeps an error but KEEPS RUNNING so deployment can still happen.

   LIBRARIES (install via Arduino Library Manager):
     - Adafruit BMP085 Library   (drives the BMP180 — register-compatible)
     - ESP32Servo
     - Board: "ESP32 Dev Module" (espressif/arduino-esp32 core)
     SD.h, SPI.h, Wire.h ship with the core. The MPU-6500 is driven directly
     over I2C below (no library) for reliability.

   MOUNTING: mount the board so the chosen UP axis (default +Z) points at the
   nose cone. Verify with the serial monitor: at rest the up-axis should read
   ~ +1.00 g.
   ============================================================================ */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP085.h>
#include <ESP32Servo.h>
#include <math.h>

// ---------------------------------------------------------------------------
// PIN MAP
// ---------------------------------------------------------------------------
#define PIN_SDA        21
#define PIN_SCL        22
#define PIN_SD_CS       5
#define PIN_SERVO      13
#define PIN_BUZZER     25
#define PIN_ARM_SENSE  14   // DPDT pole 2: LOW = armed (to GND). Pole 1 gates servo 5V in hardware.
#define PIN_LED         2   // onboard LED, status

// ---------------------------------------------------------------------------
// FLIGHT-CRITICAL TUNABLES  — review every one of these before flying
// ---------------------------------------------------------------------------
#define LOOP_MS               50      // main loop period -> 20 Hz logging/detection
#define G_MS2               9.80665f

// Servo travel
#define SERVO_REST_ANGLE       0      // holding (pin retained) position
#define SERVO_DEPLOY_ANGLE    90      // released position

// Launch detection (enter BOOST)
#define LAUNCH_G             2.0f     // up-axis specific force threshold, g
#define LAUNCH_SAMPLES         3      // consecutive samples above threshold

// Burnout detection (BOOST -> COAST)
#define BURNOUT_G            0.9f     // up-axis drops below this after thrust ends
#define BURNOUT_SAMPLES        3
#define MIN_BOOST_MS         300      // ignore burnout before this (avoids ignition noise)

// Apogee detection (only evaluated in COAST, only after lockout)
#define LOCKOUT_MS          1500      // min time since launch before apogee can trigger
#define APOGEE_V             1.0f     // fused velocity at/below this counts as "stopped climbing", m/s
#define APOGEE_SAMPLES         5      // consecutive samples of falling altitude to confirm

// Backup / safety-net deploy  —  MUST be tuned to fire just AFTER expected apogee
#define BACKUP_DEPLOY_MS    7000      // TUNE: predicted apogee time + margin (e.g. OpenRocket)

// Landing detection (DESCENT -> LANDED)
#define LAND_ALT             10.0f    // near ground, m AGL
#define LAND_V               1.5f     // near-zero vertical speed, m/s
#define LAND_SAMPLES          60      // ~3 s of stillness at 20 Hz

// Filters
#define ALT_EMA              0.35f    // altitude low-pass (0..1, higher = less smoothing)
#define FUSE_ALPHA           0.95f    // complementary filter weight on accel integration

// Sensor orientation: which accel axis points at the nose, and its sign.
#define UP_AXIS                2      // 0=X, 1=Y, 2=Z
#define UP_SIGN              1.0f     // +1 if that axis reads +1g at rest nose-up, else -1

// MPU-6500 scaling (configured for +/-16 g, +/-2000 dps below)
#define ACC_LSB_PER_G      2048.0f
#define GYR_LSB_PER_DPS      16.4f
#define MPU_ADDR             0x68

// ---------------------------------------------------------------------------
// GLOBALS
// ---------------------------------------------------------------------------
Adafruit_BMP085 bmp;
Servo deployServo;
File logFile;

bool haveBMP = false, haveMPU = false, haveSD = false;

float groundPressure = 101325.0f;   // Pa, captured on pad -> altitude datum
float accelBiasG      = 0.0f;       // up-axis zero-g bias correction

// derived flight state
float altRaw = 0, altFilt = 0, altFiltPrev = 0;
float vBaro = 0, vAccel = 0, vFused = 0;
float aVert = 0;                     // vertical accel, m/s^2 (gravity removed)
float accG[3] = {0,0,0}, gyrDps[3] = {0,0,0};
float _lastPa = 101325.0f;              // cached pressure so log + altitude share one read
float bmp_lastPa() { return _lastPa; }

enum FlightState { PAD, BOOST, COAST, DESCENT, LANDED };
FlightState state = PAD;

uint32_t tLaunch = 0, tDeploy = 0, tPrev = 0;
int launchCount = 0, burnoutCount = 0, apogeeCount = 0, landCount = 0;
bool deployed = false;
const char* lastEvent = "";

// buzzer non-blocking timing
uint32_t buzzTimer = 0;
bool buzzOn = false;

// ---------------------------------------------------------------------------
// MPU-6500 low-level (raw I2C)
// ---------------------------------------------------------------------------
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}
uint8_t mpuRead(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}
bool mpuBegin() {
  uint8_t who = mpuRead(0x75);            // WHO_AM_I -> 0x70 for MPU-6500
  if (who != 0x70 && who != 0x71) return false;  // 0x71 = MPU-9250 (compatible)
  mpuWrite(0x6B, 0x80); delay(100);       // reset
  mpuWrite(0x6B, 0x01); delay(10);        // wake, clock = PLL
  mpuWrite(0x1A, 0x03);                   // DLPF ~41 Hz (cuts airframe vibration)
  mpuWrite(0x1B, 0x18);                   // gyro +/-2000 dps
  mpuWrite(0x1C, 0x18);                   // accel +/-16 g
  delay(10);
  return true;
}
void mpuReadAll() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();               // temperature, ignored
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();
  accG[0] = ax / ACC_LSB_PER_G;
  accG[1] = ay / ACC_LSB_PER_G;
  accG[2] = az / ACC_LSB_PER_G;
  gyrDps[0] = gx / GYR_LSB_PER_DPS;
  gyrDps[1] = gy / GYR_LSB_PER_DPS;
  gyrDps[2] = gz / GYR_LSB_PER_DPS;
}
inline float upAxisG() { return UP_SIGN * accG[UP_AXIS]; }

// ---------------------------------------------------------------------------
// Buzzer helpers  (active buzzer: HIGH = on; flip if yours is active-low)
// ---------------------------------------------------------------------------
void buzz(bool on) { digitalWrite(PIN_BUZZER, on ? HIGH : LOW); }
void buzzBurst(int n, int onMs, int offMs) {
  for (int i = 0; i < n; i++) { buzz(true); delay(onMs); buzz(false); delay(offMs); }
}

// ---------------------------------------------------------------------------
// SD logging
// ---------------------------------------------------------------------------
void openLog() {
  if (!haveSD) return;
  char name[16];
  for (int i = 0; i < 1000; i++) {
    snprintf(name, sizeof(name), "/LOG%03d.csv", i);
    if (!SD.exists(name)) break;
  }
  logFile = SD.open(name, FILE_WRITE);
  if (logFile) {
    logFile.println("t_ms,state,event,pressPa,altRaw,altFilt,"
                    "aUpG,aVert,vBaro,vAccel,vFused,gx,gy,gz");
    logFile.flush();
    Serial.print("Logging to "); Serial.println(name);
  } else {
    haveSD = false;
  }
}
void writeLog(uint32_t t) {
  if (!haveSD || !logFile) return;
  logFile.printf("%lu,%d,%s,%.1f,%.2f,%.2f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f\n",
                 (unsigned long)t, (int)state, lastEvent, bmp_lastPa(),
                 altRaw, altFilt, upAxisG(), aVert, vBaro, vAccel, vFused,
                 gyrDps[0], gyrDps[1], gyrDps[2]);
  lastEvent = "";
}

// ---------------------------------------------------------------------------
// Deployment
// ---------------------------------------------------------------------------
void deploy(const char* why) {
  if (deployed) return;
  deployServo.write(SERVO_DEPLOY_ANGLE);   // no-ops mechanically if disarmed (servo unpowered)
  deployed = true;
  tDeploy = millis();
  lastEvent = why;
  state = DESCENT;
  buzzBurst(3, 90, 90);                     // distinct apogee/deploy signal
  Serial.print("DEPLOY: "); Serial.println(why);
  if (haveSD && logFile) logFile.flush();
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(PIN_BUZZER, OUTPUT); buzz(false);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_ARM_SENSE, INPUT_PULLUP);

  // Servo to rest FIRST, before anything can go wrong, so a powered servo holds the pin.
  deployServo.setPeriodHertz(50);
  deployServo.attach(PIN_SERVO, 500, 2400);
  deployServo.write(SERVO_REST_ANGLE);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  haveMPU = mpuBegin();
  haveBMP = bmp.begin();                    // default oversampling (ultra-high res)
  haveSD  = SD.begin(PIN_SD_CS);

  Serial.printf("MPU:%d  BMP:%d  SD:%d\n", haveMPU, haveBMP, haveSD);

  // Error tones, but we ALWAYS continue so recovery can still fire.
  if (!haveMPU) buzzBurst(2, 400, 200);     // 2 long = IMU fault
  if (!haveBMP) buzzBurst(3, 400, 200);     // 3 long = baro fault (rely on backup timer)
  if (!haveSD)  buzzBurst(4, 200, 200);     // 4 short = no SD (flight still armed)

  // --- Pad calibration: capture ground pressure + accelerometer bias ---
  float pSum = 0, aSum = 0; int n = 0;
  for (int i = 0; i < 60; i++) {            // ~1.5 s of averaging on the pad
    if (haveBMP) pSum += bmp.readPressure();
    if (haveMPU) { mpuReadAll(); aSum += upAxisG(); }
    n++; delay(25);
  }
  if (haveBMP) groundPressure = pSum / n;
  if (haveMPU) accelBiasG   = (aSum / n) - 1.0f;   // up-axis should read +1g at rest
  Serial.printf("P0=%.1f Pa  accelBias=%.3f g\n", groundPressure, accelBiasG);

  altFilt = altFiltPrev = 0;
  openLog();

  buzzBurst(1, 120, 0);                      // "ready" chirp
  tPrev = millis();
}

// ---------------------------------------------------------------------------
// Sensor read + derived quantities
// ---------------------------------------------------------------------------
void readSensors(float dt) {
  if (haveMPU) mpuReadAll();
  if (haveBMP) {
    _lastPa = bmp.readPressure();
    altRaw = 44330.0f * (1.0f - powf(_lastPa / groundPressure, 0.1903f));
  }
  altFiltPrev = altFilt;
  altFilt += ALT_EMA * (altRaw - altFilt);

  // vertical acceleration with gravity + bias removed (assumes up-axis ~ vertical)
  float upG = upAxisG() - accelBiasG;
  aVert = (upG - 1.0f) * G_MS2;

  // three velocity estimates
  if (dt > 0) vBaro = (altFilt - altFiltPrev) / dt;
  vAccel += aVert * dt;                                   // drifts (shown for comparison)
  vFused = FUSE_ALPHA * (vFused + aVert * dt) + (1.0f - FUSE_ALPHA) * vBaro;
}

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
void updateState(uint32_t now) {
  float upG = upAxisG() - accelBiasG;

  switch (state) {
    case PAD:
      launchCount = (upG > LAUNCH_G) ? launchCount + 1 : 0;
      if (launchCount >= LAUNCH_SAMPLES) {
        state = BOOST; tLaunch = now;
        vAccel = 0; vFused = 0;                 // start integrators clean at liftoff
        lastEvent = "LAUNCH";
        Serial.println("LAUNCH");
      }
      break;

    case BOOST:
      if (now - tLaunch > MIN_BOOST_MS) {
        burnoutCount = (upG < BURNOUT_G) ? burnoutCount + 1 : 0;
        if (burnoutCount >= BURNOUT_SAMPLES) {
          state = COAST; lastEvent = "BURNOUT";
          Serial.println("BURNOUT -> COAST");
        }
      }
      break;

    case COAST:
      // BOOST LOCKOUT: only past this point can apogee ever fire.
      if (now - tLaunch > LOCKOUT_MS) {
        bool falling = (altFilt < altFiltPrev);
        apogeeCount = (vFused < APOGEE_V && falling) ? apogeeCount + 1 : 0;
        if (apogeeCount >= APOGEE_SAMPLES) deploy("APOGEE_BARO");
      }
      break;

    case DESCENT:
      if (fabsf(altFilt) < LAND_ALT && fabsf(vFused) < LAND_V) landCount++;
      else landCount = 0;
      if (landCount >= LAND_SAMPLES) {
        state = LANDED; lastEvent = "LANDED";
        Serial.println("LANDED");
        if (haveSD && logFile) { logFile.flush(); logFile.close(); }
      }
      break;

    case LANDED:
      break;
  }

  // Backup timer deploy — catches a baro failure. Fires only if not already out.
  if (!deployed && (state == BOOST || state == COAST)
      && (now - tLaunch > BACKUP_DEPLOY_MS)) {
    deploy("BACKUP_TIMER");
  }
}

// ---------------------------------------------------------------------------
// Buzzer behaviour by state (non-blocking)
// ---------------------------------------------------------------------------
void updateBuzzer(uint32_t now) {
  bool armed = (digitalRead(PIN_ARM_SENSE) == LOW);
  digitalWrite(PIN_LED, armed);

  if (state == LANDED) {                       // loud locate beacon, forever
    if (now - buzzTimer > 400) { buzzOn = !buzzOn; buzz(buzzOn); buzzTimer = now; }
    return;
  }
  if (state == PAD) {                          // audible arm status through the cutout
    uint32_t period = armed ? 2000 : 3000;
    if (now - buzzTimer > period) { buzzTimer = now;
      if (armed) buzzBurst(1, 40, 0);          // single chirp = armed
      else       buzzBurst(2, 40, 60);         // double chirp = disarmed
    }
    return;
  }
  buzz(false);                                 // silent during flight until deploy/landing
}

// ---------------------------------------------------------------------------
// MAIN LOOP  — paced to LOOP_MS
// ---------------------------------------------------------------------------
void loop() {
  uint32_t t0 = millis();
  float dt = (t0 - tPrev) / 1000.0f;
  tPrev = t0;

  readSensors(dt);
  updateState(t0);
  updateBuzzer(t0);
  writeLog(t0);

  // periodic flush (~1 s) so a crash-on-landing loses at most ~1 s of data
  static uint8_t flushCtr = 0;
  if (haveSD && logFile && state != LANDED && ++flushCtr >= 20) { logFile.flush(); flushCtr = 0; }

  uint32_t elapsed = millis() - t0;
  if (elapsed < LOOP_MS) delay(LOOP_MS - elapsed);
}