/*
  HOMIS-II flight computer - ESP32-CAM (AI-Thinker) + GY-87 + MG90S
  ------------------------------------------------------------------
  Built on Logger/Logger.ino (same wiring, same SD/camera setup). Adds:
    - 100 Hz IMU (MPU6050) + 50 Hz barometer (BMP180), non-blocking
    - Kalman-filtered altitude and vertical velocity (FlightLogic.h)
    - Apogee detection -> MG90S turns lock -> release (parachute out)
    - Wi-Fi access point with a pre-launch page: sensor checks, servo test, ARM
    - Survives a brown-out reset mid-flight (state kept in RTC memory)

  Files on the SD card, one folder per power-up:
    /run_NNN/flight.csv  every sample from ARM to landing (10 Hz when SAFE)
    /run_NNN/events.csv  arm, launch, burnout, deploy, landing, errors
    /run_NNN/photos.csv  + IMG_nnnnnn.jpg, one per second, as before

  Wi-Fi: join AP_SSID / AP_PASS, open http://192.168.4.1
  Wi-Fi switches off at launch and back on after landing.

  Wiring: unchanged from Logger (see ../esp32cam_gy87_project_1.md).
    GY-87 SDA -> GPIO13, SCL -> GPIO12, MG90S signal -> GPIO4.
    Cover the flash LED: it shares GPIO4 and glows with the servo pulses.

  Arduino IDE: board "AI Thinker ESP32-CAM", CPU 240 MHz, "Huge APP (3MB No OTA)",
  PSRAM enabled. No extra libraries.
*/

#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "driver/ledc.h"
#include "freertos/stream_buffer.h"
#include "esp_system.h"
#include "FlightLogic.h"
#include "WebPage.h"

// ------------------------------------------------------------ user settings
const char *AP_SSID = "HOMIS2-AVIONICS";
const char *AP_PASS = "isra-m003";          // min 8 characters; change before the field
const uint32_t PHOTO_PERIOD_MS = 1000;
const uint32_t LOOP_MS = 10;                // 100 Hz flight loop
float SEA_LEVEL_PA = 101325.0f;             // only for the absolute altitude on the page
const framesize_t FRAME_SIZE = FRAMESIZE_SVGA;
const int JPEG_QUALITY = 12;

// Servo: MG90S, 50 Hz. 0 deg = SERVO_MIN_US, 180 deg = SERVO_MAX_US.
// Lock/release angles default to 0 -> 90 and can be changed from the Wi-Fi page.
const float SERVO_MIN_US = 500, SERVO_MAX_US = 2400;
const float DEFAULT_LOCK_DEG = 0, DEFAULT_RELEASE_DEG = 90;

// ------------------------------------------------------------ pins
#define I2C_SDA    13
#define I2C_SCL    12
#define SERVO_PIN   4

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const uint8_t MPU_ADDR = 0x68;
const uint8_t BMP_ADDR = 0x77;

// ------------------------------------------------------------ shared state
// The flight task owns the Flight object, the sensors and the servo. Other
// tasks talk to it through cmdQueue and read it through the Status snapshot.
enum CmdType : uint8_t { CMD_ARM, CMD_DISARM, CMD_SERVO_LOCK, CMD_SERVO_RELEASE,
                         CMD_SET_BACKUP, CMD_SET_LOCK_DEG, CMD_SET_RELEASE_DEG };
struct Cmd { CmdType type; float value; };

struct Status {
  uint8_t state;
  bool mpuOk, mpuLive, bmpOk, bmpLive, sdOk, camOk, servoReleased, restored, wifiOn;
  float pressPa, tempC, hBaro, alt, vel, aVert, gForce, gyroDps, tilt, maxAlt, t;
  float backupS, lockDeg, releaseDeg;
  uint32_t photos, logDrops, mpuErrs, bmpErrs, uptime;
  char msg[80];
  char deployReason[12];
};

Status st = {};
portMUX_TYPE stMux = portMUX_INITIALIZER_UNLOCKED;
QueueHandle_t cmdQueue, eventQueue;
StreamBufferHandle_t logStream;
SemaphoreHandle_t sdMutex;
Preferences prefs;
WebServer server(80);
fl::Flight flight;

struct LogEvent { uint32_t ms; char text[92]; };

char runDir[16], flightLog[40], eventLog[40], photoLog[40];
bool sdOk = false, mpuOk = false, bmpOk = false;
volatile bool wifiWanted = true;
float lockDeg = DEFAULT_LOCK_DEG, releaseDeg = DEFAULT_RELEASE_DEG;
bool servoReleased = false;

// Brown-out recovery. RTC_NOINIT memory survives a reset but not a power cycle.
const uint32_t RTC_MAGIC = 0x4D303033;   // "M003"
struct RtcState { uint32_t magic; fl::Saved saved; bool servoReleased; };
RTC_NOINIT_ATTR RtcState rtc;
bool restored = false;

// ============================================================ helpers
void logEvent(const char *fmt, ...) {
  LogEvent e;
  e.ms = millis();
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(e.text, sizeof e.text, fmt, ap);
  va_end(ap);
  Serial.printf("[%lu] %s\n", (unsigned long)e.ms, e.text);
  if (eventQueue) xQueueSend(eventQueue, &e, 0);
}

bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint16_t)addr, len, true) != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// ============================================================ MPU6050
// +/-16 g (a solid motor can pass 8 g), +/-2000 deg/s, DLPF 44 Hz
const float ACC_LSB = 9.80665f / 2048.0f;               // -> m/s^2
const float GYR_LSB = (1.0f / 16.4f) * (PI / 180.0f);   // -> rad/s

bool mpuInit() {
  uint8_t who = 0;
  if (!i2cRead(MPU_ADDR, 0x75, &who, 1)) return false;
  Serial.printf("MPU6050 WHO_AM_I = 0x%02X%s\n", who, who == 0x68 ? "" : " (clone chip?)");
  i2cWrite8(MPU_ADDR, 0x6B, 0x80);   // reset
  delay(100);
  i2cWrite8(MPU_ADDR, 0x6B, 0x01);   // wake, clock = gyro X PLL
  i2cWrite8(MPU_ADDR, 0x1A, 0x03);   // DLPF ~44 Hz
  i2cWrite8(MPU_ADDR, 0x1B, 0x18);   // gyro +/-2000 deg/s
  i2cWrite8(MPU_ADDR, 0x1C, 0x18);   // accel +/-16 g
  return true;
}

// raw[] is kept by the caller to spot a frozen sensor
bool mpuRead(float f[3], float w[3], bool &sat, uint8_t raw[14]) {
  if (!i2cRead(MPU_ADDR, 0x3B, raw, 14)) return false;
  sat = false;
  for (int i = 0; i < 3; i++) {
    int16_t a = (int16_t)((raw[2 * i] << 8) | raw[2 * i + 1]);
    int16_t g = (int16_t)((raw[8 + 2 * i] << 8) | raw[9 + 2 * i]);
    if (a >= 32700 || a <= -32700) sat = true;
    f[i] = a * ACC_LSB;
    w[i] = g * GYR_LSB;
  }
  return true;
}

// ============================================================ BMP180 (datasheet algorithm)
struct {
  int16_t  AC1, AC2, AC3;
  uint16_t AC4, AC5, AC6;
  int16_t  B1, B2, MB, MC, MD;
} cal;
const uint8_t OSS = 2;                 // 13.5 ms conversion -> fits 50 Hz
const uint32_t BMP_T_US = 5000, BMP_P_US = 14000;

bool bmpInit() {
  uint8_t id = 0;
  if (!i2cRead(BMP_ADDR, 0xD0, &id, 1) || id != 0x55) return false;
  uint8_t b[22];
  if (!i2cRead(BMP_ADDR, 0xAA, b, 22)) return false;
  auto s16 = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
  auto u16 = [&](int i) { return (uint16_t)((b[i] << 8) | b[i + 1]); };
  cal.AC1 = s16(0);  cal.AC2 = s16(2);  cal.AC3 = s16(4);
  cal.AC4 = u16(6);  cal.AC5 = u16(8);  cal.AC6 = u16(10);
  cal.B1  = s16(12); cal.B2  = s16(14); cal.MB  = s16(16);
  cal.MC  = s16(18); cal.MD  = s16(20);
  return true;
}

bool bmpStartTemp()  { return i2cWrite8(BMP_ADDR, 0xF4, 0x2E); }
bool bmpStartPress() { return i2cWrite8(BMP_ADDR, 0xF4, 0x34 + (OSS << 6)); }

bool bmpReadTemp(int32_t &B5, float &tempC) {
  uint8_t b[2];
  if (!i2cRead(BMP_ADDR, 0xF6, b, 2)) return false;
  int32_t UT = ((int32_t)b[0] << 8) | b[1];
  int32_t X1 = ((UT - (int32_t)cal.AC6) * (int32_t)cal.AC5) >> 15;
  int32_t X2 = ((int32_t)cal.MC * 2048) / (X1 + cal.MD);
  B5 = X1 + X2;
  tempC = ((B5 + 8) >> 4) / 10.0f;
  return true;
}

bool bmpReadPress(int32_t B5, int32_t &pressPa, int32_t &UP) {
  uint8_t b[3];
  if (!i2cRead(BMP_ADDR, 0xF6, b, 3)) return false;
  UP = (((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | b[2]) >> (8 - OSS);
  int32_t B6 = B5 - 4000;
  int32_t X1 = ((int32_t)cal.B2 * ((B6 * B6) >> 12)) >> 11;
  int32_t X2 = ((int32_t)cal.AC2 * B6) >> 11;
  int32_t X3 = X1 + X2;
  int32_t B3 = ((((int32_t)cal.AC1 * 4 + X3) * (1 << OSS)) + 2) / 4;
  X1 = ((int32_t)cal.AC3 * B6) >> 13;
  X2 = ((int32_t)cal.B1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  uint32_t B4 = ((uint32_t)cal.AC4 * (uint32_t)(X3 + 32768)) >> 15;
  uint32_t B7 = ((uint32_t)UP - (uint32_t)B3) * (uint32_t)(50000UL >> OSS);
  int32_t p = (B7 < 0x80000000UL) ? (int32_t)((B7 * 2) / B4) : (int32_t)((B7 / B4) * 2);
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;
  pressPa = p + ((X1 + X2 + 3791) >> 4);
  return true;
}

// blocking read, setup only
bool bmpReadBlocking(float &tempC, int32_t &pa) {
  int32_t B5, UP;
  if (!bmpStartTemp()) return false;
  delayMicroseconds(BMP_T_US);
  if (!bmpReadTemp(B5, tempC)) return false;
  if (!bmpStartPress()) return false;
  delayMicroseconds(BMP_P_US);
  return bmpReadPress(B5, pa, UP);
}

// ============================================================ servo (LEDC timer 1)
// Driven through the ESP-IDF LEDC driver directly: the camera clock owns LEDC
// timer 0 / channel 0, so the servo takes timer 1 / channel 2.
const ledc_mode_t SERVO_MODE = LEDC_LOW_SPEED_MODE;
const ledc_channel_t SERVO_CH = LEDC_CHANNEL_2;

uint32_t servoDuty(float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  float us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0f;
  return (uint32_t)(us / 20000.0f * 16384.0f);   // 14-bit, 20 ms period
}

void servoInit(float deg) {
  ledc_timer_config_t t = {};
  t.speed_mode = SERVO_MODE;
  t.duty_resolution = LEDC_TIMER_14_BIT;
  t.timer_num = LEDC_TIMER_1;
  t.freq_hz = 50;
  t.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&t);
  ledc_channel_config_t c = {};
  c.gpio_num = SERVO_PIN;
  c.speed_mode = SERVO_MODE;
  c.channel = SERVO_CH;
  c.timer_sel = LEDC_TIMER_1;
  c.duty = servoDuty(deg);
  c.hpoint = 0;
  ledc_channel_config(&c);
}

void servoWrite(float deg) {
  ledc_set_duty(SERVO_MODE, SERVO_CH, servoDuty(deg));
  ledc_update_duty(SERVO_MODE, SERVO_CH);
}

void servoLock()    { servoReleased = false; servoWrite(lockDeg); }
void servoRelease() { servoReleased = true;  servoWrite(releaseDeg); }

// ============================================================ camera
bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk  = XCLK_GPIO_NUM;
  c.pin_pclk  = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href  = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.sccb_i2c_port = 1;              // GY-87 is on port 0
  c.pin_pwdn  = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  if (psramFound()) {
    c.frame_size   = FRAME_SIZE;
    c.jpeg_quality = JPEG_QUALITY;
    c.fb_count     = 2;
    c.fb_location  = CAMERA_FB_IN_PSRAM;
    c.grab_mode    = CAMERA_GRAB_LATEST;
  } else {
    c.frame_size   = FRAMESIZE_VGA;
    c.jpeg_quality = 14;
    c.fb_count     = 1;
    c.fb_location  = CAMERA_FB_IN_DRAM;
    c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  }
  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  for (int i = 0; i < 4; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(100);
  }
  return true;
}

// ============================================================ SD card
bool initSD() {
  if (!SD_MMC.begin("/sdcard", true)) return false;   // 1-bit mode
  if (SD_MMC.cardType() == CARD_NONE) return false;
  for (int n = 1; n < 1000; n++) {
    snprintf(runDir, sizeof runDir, "/run_%03d", n);
    if (!SD_MMC.exists(runDir)) break;
  }
  SD_MMC.mkdir(runDir);
  snprintf(flightLog, sizeof flightLog, "%s/flight.csv", runDir);
  snprintf(eventLog,  sizeof eventLog,  "%s/events.csv", runDir);
  snprintf(photoLog,  sizeof photoLog,  "%s/photos.csv", runDir);
  File f = SD_MMC.open(flightLog, FILE_WRITE);
  if (!f) return false;
  f.println("ms,state,p_Pa,temp_C,h_baro_m,h_m,v_ms,a_vert_ms2,acc_bias,"
            "ax,ay,az,gx,gy,gz,tilt_deg,sat");
  f.close();
  f = SD_MMC.open(eventLog, FILE_WRITE);
  if (f) { f.println("ms,event"); f.close(); }
  f = SD_MMC.open(photoLog, FILE_WRITE);
  if (f) { f.println("ms,file,bytes"); f.close(); }
  Serial.printf("Logging to %s\n", runDir);
  return true;
}

// ============================================================ flight task (core 1)
// Hardware checks that FlightLogic cannot see. Returns an error or nullptr.
const char *armBlocker(bool mpuLive, bool bmpLive) {
  if (!mpuOk || !mpuLive) return "Arm refused: MPU6050 not reading";
  if (!bmpOk || !bmpLive) return "Arm refused: BMP180 not reading";
  if (!sdOk)              return "Arm refused: no SD card (flight would not be logged)";
  if (flight.backupS <= 0) return "Arm refused: set the backup timer first";
  if (servoReleased)      return "Arm refused: servo is in RELEASE - lock it first";
  return nullptr;
}

void handleCmd(const Cmd &c, bool mpuLive, bool bmpLive) {
  bool safe = flight.state == fl::SAFE;
  switch (c.type) {
    case CMD_ARM: {
      const char *why = safe ? armBlocker(mpuLive, bmpLive) : "Arm refused: not in SAFE";
      if (why) { snprintf(flight.msg, sizeof flight.msg, "%s", why); logEvent("%s", why); }
      else if (flight.arm()) logEvent("arm requested, calibrating");
      break;
    }
    case CMD_DISARM:
      if (flight.disarm()) logEvent("disarmed");
      break;
    case CMD_SERVO_LOCK:
      if (safe) { servoLock(); logEvent("servo test: lock (%.0f deg)", lockDeg); }
      break;
    case CMD_SERVO_RELEASE:
      if (safe) { servoRelease(); logEvent("servo test: release (%.0f deg)", releaseDeg); }
      break;
    case CMD_SET_BACKUP:
      if (safe) { flight.backupS = c.value; logEvent("backup timer set to %.2f s", c.value); }
      break;
    case CMD_SET_LOCK_DEG:
      if (safe) { lockDeg = c.value; if (!servoReleased) servoWrite(lockDeg); }
      break;
    case CMD_SET_RELEASE_DEG:
      if (safe) { releaseDeg = c.value; if (servoReleased) servoWrite(releaseDeg); }
      break;
  }
}

void flightTask(void *) {
  TickType_t last = xTaskGetTickCount();
  uint32_t lastUs = micros();

  // sensor health
  uint8_t raw[14], prevRaw[14] = {0};
  int mpuFail = 0, mpuSame = 0, bmpFail = 0, bmpSame = 0;
  uint32_t mpuErrs = 0, bmpErrs = 0, logDrops = 0;
  // baro scheduler: temperature once a second, pressure the rest of the time
  enum { B_TEMP, B_PRESS } bph = B_TEMP;
  uint32_t bStart = micros();
  int32_t B5 = 0, pa = 0, UP = 0, prevUP = -1;
  float tempC = NAN;
  int pressCount = 0;
  if (bmpOk) bmpStartTemp();

  uint32_t n = 0;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(LOOP_MS));
    uint32_t nowUs = micros();
    fl::Frame fr = {};
    fr.dt = (nowUs - lastUs) * 1e-6f;
    lastUs = nowUs;

    // ---- IMU
    if (mpuOk) {
      fr.imuOk = mpuRead(fr.f, fr.w, fr.saturated, raw);
      if (fr.imuOk) {
        mpuFail = 0;
        mpuSame = memcmp(raw, prevRaw, 14) == 0 ? mpuSame + 1 : 0;
        memcpy(prevRaw, raw, 14);
        if (mpuSame > 50) fr.imuOk = false;          // frozen: same bytes for 0.5 s
      } else { mpuFail++; mpuErrs++; }
    }

    // ---- barometer
    if (bmpOk) {
      uint32_t el = nowUs - bStart;
      bool ok = true;
      if (bph == B_TEMP && el >= BMP_T_US) {
        ok = bmpReadTemp(B5, tempC) && bmpStartPress();
        bph = B_PRESS; bStart = micros();
      } else if (bph == B_PRESS && el >= BMP_P_US) {
        ok = bmpReadPress(B5, pa, UP);
        if (ok) {
          fr.baroNew = pa > 30000 && pa < 110000;
          fr.pressPa = pa;
          bmpSame = UP == prevUP ? bmpSame + 1 : 0;
          prevUP = UP;
          if (bmpSame > 25) fr.baroNew = false;      // frozen
          pressCount++;
        }
        if (pressCount % 50 == 0) { ok = bmpStartTemp() && ok; bph = B_TEMP; }
        else                      { ok = bmpStartPress() && ok; }
        bStart = micros();
      }
      if (ok) bmpFail = 0;
      else {
        bmpFail++; bmpErrs++;
        bmpStartTemp(); bph = B_TEMP; bStart = micros();   // restart the cycle
      }
    }
    bool mpuLive = mpuOk && mpuFail < 10 && mpuSame <= 50;
    bool bmpLive = bmpOk && bmpFail < 10 && bmpSame <= 25;

    // ---- commands from the web page
    Cmd c;
    while (xQueueReceive(cmdQueue, &c, 0) == pdTRUE) handleCmd(c, mpuLive, bmpLive);

    // ---- estimate + state machine
    uint8_t before = flight.state;
    uint16_t ev = flight.step(fr);

    if (ev & fl::EV_DEPLOY) servoRelease();          // first, before any logging
    if (ev & fl::EV_ARMED)       logEvent("%s", flight.msg);
    if (ev & fl::EV_CAL_FAILED)  logEvent("%s", flight.msg);
    if (ev & fl::EV_LAUNCH)      { logEvent("%s", flight.msg); wifiWanted = false; }
    if (ev & fl::EV_BURNOUT)     logEvent("%s, v=%.1f m/s, h=%.1f m", flight.msg, flight.kf.x[1], flight.kf.x[0]);
    if (ev & fl::EV_DEPLOY)      logEvent("%s", flight.msg);
    if (ev & fl::EV_FALSE_LAUNCH){ logEvent("%s", flight.msg); wifiWanted = true; }
    if (ev & fl::EV_LANDED)      { logEvent("landed, max altitude %.1f m", flight.maxAlt); wifiWanted = true; }

    // ---- brown-out memory
    if (flight.state >= fl::ARMED) {
      flight.save(rtc.saved);
      rtc.servoReleased = servoReleased;
      rtc.magic = RTC_MAGIC;
    } else {
      rtc.magic = 0;
    }

    // ---- log: every sample while armed/flying, 10 Hz otherwise
    bool full = flight.state >= fl::ARMED && flight.state <= fl::DESCENT;
    if (full || n % 10 == 0 || before != flight.state) {
      char line[200];
      int len = snprintf(line, sizeof line,
        "%lu,%s,%ld,%.1f,%.2f,%.2f,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.1f,%d\n",
        (unsigned long)millis(), fl::STATE_NAME[flight.state], (long)pa, tempC,
        flight.hBaro, flight.kf.x[0], flight.kf.x[1], flight.aVert, flight.kf.x[2],
        fr.f[0], fr.f[1], fr.f[2], fr.w[0], fr.w[1], fr.w[2], flight.tilt, fr.saturated ? 1 : 0);
      if (xStreamBufferSend(logStream, line, len, 0) != (size_t)len) logDrops++;
    }

    // ---- snapshot for the web page and serial
    float fmag = sqrtf(fr.f[0] * fr.f[0] + fr.f[1] * fr.f[1] + fr.f[2] * fr.f[2]);
    float gw[3] = {fr.w[0] - flight.gyroBias[0], fr.w[1] - flight.gyroBias[1], fr.w[2] - flight.gyroBias[2]};
    portENTER_CRITICAL(&stMux);
    st.state = flight.state;
    st.mpuOk = mpuOk; st.mpuLive = mpuLive; st.bmpOk = bmpOk; st.bmpLive = bmpLive;
    st.sdOk = sdOk; st.servoReleased = servoReleased; st.restored = restored;
    st.pressPa = pa; st.tempC = tempC; st.hBaro = flight.hBaro;
    st.alt = flight.kf.x[0]; st.vel = flight.kf.x[1]; st.aVert = flight.aVert;
    st.gForce = fmag / fl::G0;
    st.gyroDps = sqrtf(gw[0] * gw[0] + gw[1] * gw[1] + gw[2] * gw[2]) * fl::RAD2DEG;
    st.tilt = flight.tilt; st.maxAlt = flight.maxAlt; st.t = flight.t;
    st.backupS = flight.backupS; st.lockDeg = lockDeg; st.releaseDeg = releaseDeg;
    st.logDrops = logDrops; st.mpuErrs = mpuErrs; st.bmpErrs = bmpErrs;
    st.uptime = millis() / 1000;
    memcpy(st.msg, flight.msg, sizeof st.msg);
    strncpy(st.deployReason, flight.deployReason, sizeof st.deployReason - 1);
    portEXIT_CRITICAL(&stMux);

    if (n % 50 == 0)
      Serial.printf("%-11s h=%7.2f v=%7.2f a=%6.2f |f|=%.2fg p=%ld mpu=%d bmp=%d\n",
                    fl::STATE_NAME[flight.state], flight.kf.x[0], flight.kf.x[1],
                    flight.aVert, fmag / fl::G0, (long)pa, mpuLive, bmpLive);
    n++;
  }
}

// ============================================================ logger task (core 0)
void loggerTask(void *) {
  static char buf[8192];
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(200));
    size_t len = xStreamBufferReceive(logStream, buf, sizeof buf, 0);
    LogEvent e;
    bool haveEvent = uxQueueMessagesWaiting(eventQueue) > 0;
    if (!sdOk) {                        // keep draining so nothing backs up
      while (xQueueReceive(eventQueue, &e, 0) == pdTRUE) {}
      continue;
    }
    if (!len && !haveEvent) continue;
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    if (len) {
      File f = SD_MMC.open(flightLog, FILE_APPEND);
      if (f) { f.write((uint8_t *)buf, len); f.close(); }
    }
    if (haveEvent) {
      File f = SD_MMC.open(eventLog, FILE_APPEND);
      while (xQueueReceive(eventQueue, &e, 0) == pdTRUE)
        if (f) f.printf("%lu,\"%s\"\n", (unsigned long)e.ms, e.text);
      if (f) f.close();
    }
    xSemaphoreGive(sdMutex);
  }
}

// ============================================================ camera task (core 0)
void cameraTask(void *) {
  TickType_t last = xTaskGetTickCount();
  uint32_t n = 0;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(PHOTO_PERIOD_MS));
    uint32_t t = millis();
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      portENTER_CRITICAL(&stMux); st.camOk = false; portEXIT_CRITICAL(&stMux);
      continue;
    }
    n++;
    bool ok = true;
    if (sdOk) {
      char path[40];
      snprintf(path, sizeof path, "%s/IMG_%06lu.jpg", runDir, (unsigned long)n);
      xSemaphoreTake(sdMutex, portMAX_DELAY);
      File f = SD_MMC.open(path, FILE_WRITE);
      ok = f && f.write(fb->buf, fb->len) == fb->len;
      if (f) f.close();
      File pl = SD_MMC.open(photoLog, FILE_APPEND);
      if (pl) { pl.printf("%lu,%s,%u\n", (unsigned long)t, path, (unsigned)fb->len); pl.close(); }
      xSemaphoreGive(sdMutex);
    }
    esp_camera_fb_return(fb);
    portENTER_CRITICAL(&stMux); st.camOk = ok; st.photos = n; portEXIT_CRITICAL(&stMux);
  }
}

// ============================================================ Wi-Fi + web (core 0)
void sendCmd(CmdType t, float v = 0) {
  Cmd c = {t, v};
  xQueueSend(cmdQueue, &c, 0);
}

void handleStatus() {
  Status s;
  portENTER_CRITICAL(&stMux);
  s = st;
  portEXIT_CRITICAL(&stMux);
  char j[900];
  snprintf(j, sizeof j,
    "{\"state\":\"%s\",\"msg\":\"%s\",\"restored\":%d,"
    "\"mpuOk\":%d,\"mpuLive\":%d,\"bmpOk\":%d,\"bmpLive\":%d,\"sdOk\":%d,\"camOk\":%d,"
    "\"servoReleased\":%d,\"pressPa\":%.0f,\"tempC\":%.1f,\"absAlt\":%.1f,\"hBaro\":%.2f,"
    "\"alt\":%.2f,\"vel\":%.2f,\"aVert\":%.2f,\"g\":%.3f,\"gyroDps\":%.2f,\"tilt\":%.1f,"
    "\"maxAlt\":%.1f,\"t\":%.2f,\"deployReason\":\"%s\",\"backupS\":%.2f,"
    "\"lockDeg\":%.0f,\"releaseDeg\":%.0f,\"photos\":%lu,\"logDrops\":%lu,"
    "\"mpuErrs\":%lu,\"bmpErrs\":%lu,\"uptime\":%lu,\"run\":\"%s\"}",
    fl::STATE_NAME[s.state], s.msg, s.restored,
    s.mpuOk, s.mpuLive, s.bmpOk, s.bmpLive, s.sdOk, s.camOk,
    s.servoReleased, s.pressPa, s.tempC,
    44330.0f * (1.0f - powf(s.pressPa / SEA_LEVEL_PA, 0.190295f)), s.hBaro,
    s.alt, s.vel, isnan(s.aVert) ? 0.0f : s.aVert, s.gForce, s.gyroDps, s.tilt,
    s.maxAlt, s.t, s.deployReason, s.backupS,
    s.lockDeg, s.releaseDeg, (unsigned long)s.photos, (unsigned long)s.logDrops,
    (unsigned long)s.mpuErrs, (unsigned long)s.bmpErrs, (unsigned long)s.uptime,
    sdOk ? runDir : "none");
  server.send(200, "application/json", j);
}

void handleCmdRequest() {
  String c = server.arg("c");
  if      (c == "arm")     sendCmd(CMD_ARM);
  else if (c == "disarm")  sendCmd(CMD_DISARM);
  else if (c == "lock")    sendCmd(CMD_SERVO_LOCK);
  else if (c == "release") sendCmd(CMD_SERVO_RELEASE);
  else { server.send(400, "text/plain", "unknown command"); return; }
  server.send(200, "text/plain", "ok");
}

// settings are only accepted in SAFE, and saved to flash
void handleConfig() {
  uint8_t state;
  portENTER_CRITICAL(&stMux); state = st.state; portEXIT_CRITICAL(&stMux);
  if (state != fl::SAFE) { server.send(409, "text/plain", "Disarm first"); return; }
  if (server.hasArg("backup")) {
    float v = server.arg("backup").toFloat();
    if (v < 1 || v > 120) { server.send(400, "text/plain", "Backup timer must be 1-120 s"); return; }
    prefs.putFloat("backup", v);
    sendCmd(CMD_SET_BACKUP, v);
  }
  if (server.hasArg("lock")) {
    float v = server.arg("lock").toFloat();
    if (v < 0 || v > 180) { server.send(400, "text/plain", "Angle must be 0-180"); return; }
    prefs.putFloat("lock", v);
    sendCmd(CMD_SET_LOCK_DEG, v);
  }
  if (server.hasArg("release")) {
    float v = server.arg("release").toFloat();
    if (v < 0 || v > 180) { server.send(400, "text/plain", "Angle must be 0-180"); return; }
    prefs.putFloat("release", v);
    sendCmd(CMD_SET_RELEASE_DEG, v);
  }
  server.send(200, "text/plain", "saved");
}

void wifiStart() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);   // enough for the pad; smaller current spikes
  server.begin();
  Serial.printf("Wi-Fi AP \"%s\" up, http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

void wifiStop() {
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("Wi-Fi off");
}

void webTask(void *) {
  server.on("/", HTTP_GET, [] { server.send_P(200, "text/html", WEB_PAGE); });
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/cmd", HTTP_POST, handleCmdRequest);
  server.on("/config", HTTP_POST, handleConfig);
  bool on = false;
  for (;;) {
    if (wifiWanted && !on)  { wifiStart(); on = true; }
    if (!wifiWanted && on)  { wifiStop();  on = false; }
    portENTER_CRITICAL(&stMux); st.wifiOn = on; portEXIT_CRITICAL(&stMux);
    if (on) server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ============================================================ setup
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\nHOMIS-II flight computer");

  // 0) was this a reset in the middle of a flight?
  esp_reset_reason_t why = esp_reset_reason();
  restored = rtc.magic == RTC_MAGIC && why != ESP_RST_POWERON;
  if (!restored) rtc.magic = 0;

  prefs.begin("m003", false);
  lockDeg    = prefs.getFloat("lock", DEFAULT_LOCK_DEG);
  releaseDeg = prefs.getFloat("release", DEFAULT_RELEASE_DEG);

  // 1) SD first, as in the logger (LEDC before the SD host caused 0x107).
  //    Not fatal: the parachute matters more than the log. One try after a reset.
  for (int attempt = 1; attempt <= (restored ? 1 : 3) && !sdOk; attempt++) {
    sdOk = initSD();
    if (!sdOk) { SD_MMC.end(); if (!restored) delay(500); }
  }
  if (!sdOk) Serial.println("SD card FAILED - arming will be refused");

  // 2) servo, straight to where it should be
  servoReleased = restored && rtc.servoReleased;
  servoInit(servoReleased ? releaseDeg : lockDeg);

  // 3) GY-87
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  Wire.setTimeOut(10);
  mpuOk = mpuInit();
  bmpOk = bmpInit();
  Serial.printf("MPU6050: %s   BMP180: %s\n", mpuOk ? "OK" : "NOT FOUND", bmpOk ? "OK" : "NOT FOUND");

  // 4) flight state
  float groundPa = 101325;
  if (bmpOk) {
    float sum = 0; int cnt = 0;
    for (int i = 0; i < 10; i++) {
      float tc; int32_t p;
      if (bmpReadBlocking(tc, p)) { sum += p; cnt++; }
    }
    if (cnt) groundPa = sum / cnt;
  }
  flight.begin(groundPa);
  flight.backupS = prefs.getFloat("backup", 0);
  if (restored) flight.restore(rtc.saved);

  cmdQueue = xQueueCreate(8, sizeof(Cmd));
  eventQueue = xQueueCreate(24, sizeof(LogEvent));
  logStream = xStreamBufferCreate(24 * 1024, 1);
  sdMutex = xSemaphoreCreateMutex();

  if (restored) logEvent("RESET in %s (reason %d) - resumed", fl::STATE_NAME[flight.state], (int)why);
  logEvent("boot: mpu=%d bmp=%d sd=%d backup=%.2f s lock=%.0f release=%.0f",
           mpuOk, bmpOk, sdOk, flight.backupS, lockDeg, releaseDeg);

  // 5) flight loop gets core 1 to itself; everything else on core 0
  xTaskCreatePinnedToCore(flightTask, "flight", 8192, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(loggerTask, "logger", 6144, NULL, 3, NULL, 0);

  // 6) camera, then Wi-Fi (Wi-Fi stays off if we woke up mid-flight)
  bool camOk = initCamera();
  st.camOk = camOk;
  logEvent("camera: %s", camOk ? "ok" : "FAILED");
  if (camOk) xTaskCreatePinnedToCore(cameraTask, "camera", 8192, NULL, 2, NULL, 0);

  wifiWanted = !(flight.state >= fl::BOOST && flight.state <= fl::DESCENT);
  xTaskCreatePinnedToCore(webTask, "web", 6144, NULL, 1, NULL, 0);
}

void loop() {
  vTaskDelete(NULL);
}
