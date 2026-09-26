/*
  ESP32-CAM (AI-Thinker) + GY-87 logger
  -------------------------------------
  - Photo every 1.0 s                    -> /run_NNN/IMG_000001.jpg ...
  - Altitude + acceleration every 0.5 s  -> /run_NNN/sensors.csv
  - Photo timestamps                     -> /run_NNN/photos.csv

  Wiring (see diagram):
    GY-87 3.3V -> ESP32-CAM 3V3      GY-87 SDA   -> GPIO13
    GY-87 GND  -> ESP32-CAM GND      GY-87 SCL   -> GPIO12
    GY-87 FSYNC -> GND               VCC_IN, INTA, DRDY -> not connected

  Pin plan (no conflicts):
    Camera : 0, 5, 18, 19, 21, 22, 23, 25, 26, 27, 32, 34, 35, 36, 39 (internal)
    PSRAM  : 16 (internal - never use)
    microSD: 2, 14, 15 (1-bit mode)
    I2C    : 13 (SDA), 12 (SCL)
    Flash LED: 4 (held LOW)
    Serial : 1, 3

  !! GPIO12 is the flash-voltage strapping pin. The GY-87's pull-up on SCL
  !! will stop the board booting unless you first burn the flash-voltage
  !! eFuse to 3.3 V (one-time, see instructions in chat).

  Arduino IDE settings:
    Board: "AI Thinker ESP32-CAM"   Partition: "Huge APP (3MB No OTA)"
    PSRAM: Enabled (if shown)       Upload speed: 115200 (through an Uno)

  No external libraries needed (sensor drivers are below). The Adafruit
  sensor libraries are avoided on purpose: Adafruit_Sensor.h defines a
  sensor_t type that clashes with esp_camera.h.
*/

#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include <Wire.h>
#include <math.h>

// ------------------------------------------------------------ user settings
const uint32_t PHOTO_PERIOD_MS  = 1000;
const uint32_t SENSOR_PERIOD_MS = 500;
float SEA_LEVEL_PA = 101325.0f;        // set to local sea-level pressure for true altitude
const framesize_t FRAME_SIZE = FRAMESIZE_SVGA;   // 800x600; VGA is faster, XGA/UXGA slower
const int JPEG_QUALITY = 12;           // 10 = better/larger, 20 = worse/smaller

// ------------------------------------------------------------ pins
#define I2C_SDA    13
#define I2C_SCL    12
#define FLASH_LED   4

// AI-Thinker camera pins
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

// ------------------------------------------------------------ I2C addresses
const uint8_t MPU_ADDR = 0x68;   // MPU6050 (AD0 low on GY-87)
const uint8_t BMP_ADDR = 0x77;   // BMP180 / BMP085

// ------------------------------------------------------------ globals
SemaphoreHandle_t sdMutex;
char runDir[16];
char sensorLog[40];
char photoLog[40];
bool mpuOk = false, bmpOk = false;
float baseAltitude = NAN;

// ============================================================ I2C helpers
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
bool mpuInit() {
  uint8_t who = 0;
  if (!i2cRead(MPU_ADDR, 0x75, &who, 1)) return false;
  Serial.printf("MPU6050 WHO_AM_I = 0x%02X%s\n", who, who == 0x68 ? "" : " (clone chip?)");
  i2cWrite8(MPU_ADDR, 0x6B, 0x80);   // device reset
  delay(100);
  i2cWrite8(MPU_ADDR, 0x6B, 0x01);   // wake, clock = gyro X PLL
  i2cWrite8(MPU_ADDR, 0x1A, 0x03);   // DLPF ~44 Hz
  i2cWrite8(MPU_ADDR, 0x1C, 0x10);   // accel range +/-8 g (4096 LSB/g)
  return true;
}

bool mpuReadAccel(float &ax, float &ay, float &az) {
  uint8_t b[6];
  if (!i2cRead(MPU_ADDR, 0x3B, b, 6)) return false;
  const float k = 9.80665f / 4096.0f;              // -> m/s^2
  ax = (int16_t)((b[0] << 8) | b[1]) * k;
  ay = (int16_t)((b[2] << 8) | b[3]) * k;
  az = (int16_t)((b[4] << 8) | b[5]) * k;
  return true;
}

// ============================================================ BMP180 (datasheet algorithm)
struct {
  int16_t  AC1, AC2, AC3;
  uint16_t AC4, AC5, AC6;
  int16_t  B1, B2, MB, MC, MD;
} cal;
const uint8_t OSS = 3;   // ultra-high-resolution oversampling

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

bool bmpRead(float &tempC, int32_t &pressPa) {
  uint8_t b[3];

  // raw temperature
  if (!i2cWrite8(BMP_ADDR, 0xF4, 0x2E)) return false;
  delay(5);
  if (!i2cRead(BMP_ADDR, 0xF6, b, 2)) return false;
  int32_t UT = ((int32_t)b[0] << 8) | b[1];

  // raw pressure
  if (!i2cWrite8(BMP_ADDR, 0xF4, 0x34 + (OSS << 6))) return false;
  delay(26);
  if (!i2cRead(BMP_ADDR, 0xF6, b, 3)) return false;
  int32_t UP = (((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | b[2]) >> (8 - OSS);

  // temperature
  int32_t X1 = ((UT - (int32_t)cal.AC6) * (int32_t)cal.AC5) >> 15;
  int32_t X2 = ((int32_t)cal.MC * 2048) / (X1 + cal.MD);
  int32_t B5 = X1 + X2;
  tempC = ((B5 + 8) >> 4) / 10.0f;

  // pressure
  int32_t B6 = B5 - 4000;
  X1 = ((int32_t)cal.B2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((int32_t)cal.AC2 * B6) >> 11;
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

float pressureToAltitude(float pa) {
  return 44330.0f * (1.0f - powf(pa / SEA_LEVEL_PA, 0.190295f));
}

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
  c.pin_sccb_sda = SIOD_GPIO_NUM;   // camera has its own I2C (SCCB) bus on 26/27
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.sccb_i2c_port = 1;              // Wire (GY-87) uses port 0 -> no sharing
  c.pin_pwdn  = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    c.frame_size   = FRAME_SIZE;
    c.jpeg_quality = JPEG_QUALITY;
    c.fb_count     = 2;
    c.fb_location  = CAMERA_FB_IN_PSRAM;
    c.grab_mode    = CAMERA_GRAB_LATEST;   // always get a fresh frame
  } else {
    Serial.println("No PSRAM - falling back to VGA");
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
  // let auto-exposure settle
  for (int i = 0; i < 4; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(100);
  }
  return true;
}

// ============================================================ SD card
bool initSD() {
  if (!SD_MMC.begin("/sdcard", true)) {   // true = 1-bit mode -> frees GPIO 4, 12, 13
    Serial.println("SD card mount failed");
    return false;
  }
  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("No SD card inserted");
    return false;
  }
  // new folder per power-up so runs don't overwrite each other
  for (int n = 1; n < 1000; n++) {
    snprintf(runDir, sizeof runDir, "/run_%03d", n);
    if (!SD_MMC.exists(runDir)) break;
  }
  SD_MMC.mkdir(runDir);
  snprintf(sensorLog, sizeof sensorLog, "%s/sensors.csv", runDir);
  snprintf(photoLog,  sizeof photoLog,  "%s/photos.csv",  runDir);

  File f = SD_MMC.open(sensorLog, FILE_WRITE);
  if (f) { f.println("ms,temp_C,pressure_Pa,altitude_m,rel_altitude_m,ax_ms2,ay_ms2,az_ms2"); f.close(); }
  f = SD_MMC.open(photoLog, FILE_WRITE);
  if (f) { f.println("ms,file,bytes"); f.close(); }

  Serial.printf("Logging to %s\n", runDir);
  return true;
}

// ============================================================ tasks
void sensorTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    uint32_t t = millis();

    float ax = NAN, ay = NAN, az = NAN;
    if (mpuOk) mpuReadAccel(ax, ay, az);

    float tempC = NAN, alt = NAN, rel = NAN;
    int32_t pa = 0;
    if (bmpOk && bmpRead(tempC, pa)) {
      alt = pressureToAltitude(pa);
      rel = alt - baseAltitude;
    }

    char line[160];
    snprintf(line, sizeof line, "%lu,%.1f,%ld,%.2f,%.2f,%.3f,%.3f,%.3f\n",
             (unsigned long)t, tempC, (long)pa, alt, rel, ax, ay, az);

    // readings are already timestamped; waiting for the SD card here
    // does not shift the sampling schedule
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    File f = SD_MMC.open(sensorLog, FILE_APPEND);
    if (f) { f.print(line); f.close(); }
    xSemaphoreGive(sdMutex);

    Serial.print(line);
  }
}

void cameraTask(void *) {
  TickType_t last = xTaskGetTickCount();
  uint32_t n = 0;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(PHOTO_PERIOD_MS));
    uint32_t t = millis();

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { Serial.println("Capture failed"); continue; }
    n++;

    char path[40];
    snprintf(path, sizeof path, "%s/IMG_%06lu.jpg", runDir, (unsigned long)n);

    xSemaphoreTake(sdMutex, portMAX_DELAY);
    File f = SD_MMC.open(path, FILE_WRITE);
    bool ok = f && f.write(fb->buf, fb->len) == fb->len;
    if (f) f.close();
    File pl = SD_MMC.open(photoLog, FILE_APPEND);
    if (pl) { pl.printf("%lu,%s,%u\n", (unsigned long)t, path, (unsigned)fb->len); pl.close(); }
    xSemaphoreGive(sdMutex);

    size_t len = fb->len;
    esp_camera_fb_return(fb);

    uint32_t took = millis() - t;
    Serial.printf("# photo %s %u bytes %s (%lu ms)\n", path, (unsigned)len,
                  ok ? "saved" : "WRITE FAILED", (unsigned long)took);
    if (took > PHOTO_PERIOD_MS)
      Serial.println("# warning: photo took longer than 1 s - lower FRAME_SIZE or use a faster card");
  }
}

// ============================================================ setup / loop
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nESP32-CAM + GY-87 logger");

  // 1) SD card FIRST, in 1-bit mode (GPIO 2, 14, 15).
  //    The camera drives its clock with the LEDC peripheral; starting LEDC
  //    before the SD host makes sdmmc_host_reset time out (error 0x107).
  bool sdOk = false;
  for (int attempt = 1; attempt <= 3 && !sdOk; attempt++) {
    sdOk = initSD();
    if (!sdOk) { SD_MMC.end(); delay(500); }
  }
  if (!sdOk) { Serial.println("Halting: check card is FAT32, <=32 GB, fully inserted."); while (true) delay(1000); }

  // 2) camera second (its own I2C port for SCCB on GPIO 26/27)
  if (!initCamera()) { Serial.println("Halting."); while (true) delay(1000); }

  // 3) keep the flash LED off (GPIO 4 is free in 1-bit mode)
  pinMode(FLASH_LED, OUTPUT);
  digitalWrite(FLASH_LED, LOW);

  // 4) GY-87 on GPIO 13 / 12
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  mpuOk = mpuInit();
  bmpOk = bmpInit();
  Serial.printf("MPU6050: %s   BMP180: %s\n", mpuOk ? "OK" : "NOT FOUND", bmpOk ? "OK" : "NOT FOUND");

  // reference altitude = average of first readings (for rel_altitude_m)
  if (bmpOk) {
    float sum = 0; int cnt = 0;
    for (int i = 0; i < 10; i++) {
      float tc; int32_t pa;
      if (bmpRead(tc, pa)) { sum += pressureToAltitude(pa); cnt++; }
    }
    if (cnt) baseAltitude = sum / cnt;
    Serial.printf("Base altitude: %.2f m\n", baseAltitude);
  }

  sdMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(sensorTask, "sensors", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(cameraTask, "camera",  8192, NULL, 2, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);   // all work happens in the two tasks
}