/*
  FlightLogic.h - state estimation and deployment logic, no hardware.

  Plain C++ (no Arduino calls) so the same code runs on the ESP32 and in the
  desktop simulator in ../FlightSim/.

  Estimation
    1. Attitude: on the pad the gravity vector gives the orientation. After
       launch the gyro is integrated (quaternion), so the accelerometer can be
       projected onto the true vertical even when the rocket tilts or weathercocks.
    2. Vertical acceleration = (specific force rotated to world frame).z - g,
       where g is the accelerometer's own reading at rest (cancels scale error).
    3. A 3-state Kalman filter [altitude, velocity, accel bias] uses that
       acceleration as the process input and the barometer as the measurement.
       Accel gives fast, smooth velocity; baro stops the integration drifting;
       the bias state soaks up the leftover accel/attitude error.

  Deployment (first one wins, all only after launch was detected)
    - velocity : in COAST, T+ >= apogeeLockoutS, KF velocity < 0 for apogeeSamples
    - baro     : baro altitude has fallen baroDescentM below its maximum
    - timer    : T+ >= backupS (set from the OpenRocket sim apogee time)
*/
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

namespace fl {

const float G0 = 9.80665f;
const float RAD2DEG = 57.2957795f;

// Defaults, not sourced from a flight test. Check them against the OpenRocket
// sim for the rocket actually flying (thrust-to-weight, burn time, apogee time).
struct Tuning {
  float launchAccelG     = 2.5f;   // |specific force| above this many g ...
  int   launchSamples    = 5;      // ... for this many consecutive samples (50 ms)
  float launchBaroM      = 20.0f;  // backup launch detect: baro altitude above pad
  int   burnoutSamples   = 10;     // net vertical accel < 0 for this long -> COAST
  float apogeeLockoutS   = 1.5f;   // no velocity-based deploy before T+ this
  int   apogeeSamples    = 10;     // KF velocity < 0 for this long (100 ms)
  float baroDescentM     = 5.0f;   // backup: baro this far below its max
  float minFlightAltM    = 5.0f;   // a "launch" that never gets this high is a bump
  float falseLaunchS     = 3.0f;   // ... and is cancelled after this long
  float landedSpeed      = 1.0f;   // |v| below this ...
  float landedS          = 5.0f;   // ... for this long -> LANDED
  int   calSamples       = 200;    // pad calibration, 2 s at 100 Hz
  float calMaxAccelStd   = 0.3f;   // m/s^2, per axis
  float calMaxGyroStd    = 0.02f;  // rad/s (~1.1 deg/s), per axis
  // Kalman noise (1 sigma)
  float sigmaAccelPad    = 0.3f;   // m/s^2
  float sigmaAccelFlight = 2.0f;   // vibration + attitude error
  float sigmaAccelSat    = 30.0f;  // accelerometer clipped at 16 g
  float sigmaAccelBlind  = 10.0f;  // no usable accel: assume -g, trust baro
  float sigmaBias        = 0.05f;  // bias random walk, m/s^2 per sqrt(s)
  float sigmaBaroPad     = 0.5f;   // m
  float sigmaBaroFlight  = 1.5f;   // m
};

enum State : uint8_t { SAFE, CALIBRATING, ARMED, BOOST, COAST, DESCENT, LANDED };
static const char *const STATE_NAME[] = {
  "SAFE", "CALIBRATING", "ARMED", "BOOST", "COAST", "DESCENT", "LANDED" };

enum Event : uint16_t {
  EV_ARMED = 1, EV_CAL_FAILED = 2, EV_LAUNCH = 4, EV_BURNOUT = 8,
  EV_DEPLOY = 16, EV_LANDED = 32, EV_FALSE_LAUNCH = 64 };

// One sensor sample, as the hardware layer hands it over.
struct Frame {
  float dt;          // s since previous frame
  bool  imuOk;       // accel + gyro read succeeded
  float f[3];        // specific force, body frame, m/s^2 (reads +g upward at rest)
  float w[3];        // angular rate, body frame, rad/s, raw (bias not removed)
  bool  saturated;   // any accel axis at full scale
  bool  baroNew;     // pressPa is a fresh reading
  float pressPa;
};

// What survives a brown-out reset (kept in RTC memory by the sketch).
struct Saved {
  uint8_t state;
  float P0, gRef, gyroBias[3], upBody[3], t, maxAlt, maxBaro, backupS;
};

// ------------------------------------------------------------------ quaternion
struct Quat {
  float w = 1, x = 0, y = 0, z = 0;

  void normalize() {
    float n = sqrtf(w * w + x * x + y * y + z * z);
    if (n > 0) { w /= n; x /= n; y /= n; z /= n; }
  }
  // body -> world rotation that takes unit body vector u onto world +z
  static Quat fromUp(const float u[3]) {
    Quat q;
    if (u[2] < -0.9999f) { q.w = 0; q.x = 1; return q; }   // upside down
    q.w = 1 + u[2]; q.x = u[1]; q.y = -u[0]; q.z = 0;
    q.normalize();
    return q;
  }
  // q_dot = 0.5 * q (x) (0, w_body)
  void integrate(const float r[3], float dt) {
    float h = 0.5f * dt;
    float dw = -x * r[0] - y * r[1] - z * r[2];
    float dx =  w * r[0] + y * r[2] - z * r[1];
    float dy =  w * r[1] - x * r[2] + z * r[0];
    float dz =  w * r[2] + x * r[1] - y * r[0];
    w += h * dw; x += h * dx; y += h * dy; z += h * dz;
    normalize();
  }
  // world = q * body * q^-1
  void rotate(const float v[3], float out[3]) const {
    float tx = 2 * (y * v[2] - z * v[1]);
    float ty = 2 * (z * v[0] - x * v[2]);
    float tz = 2 * (x * v[1] - y * v[0]);
    out[0] = v[0] + w * tx + (y * tz - z * ty);
    out[1] = v[1] + w * ty + (z * tx - x * tz);
    out[2] = v[2] + w * tz + (x * ty - y * tx);
  }
};

// ------------------------------------------------------------------ Kalman
// x = [altitude m, velocity m/s, accel bias m/s^2]; input u = measured vertical accel
struct KF {
  float x[3] = {0, 0, 0};
  float P[3][3] = {};

  void reset(float h, float varV = 0.01f) {
    x[0] = h; x[1] = 0; x[2] = 0;
    memset(P, 0, sizeof P);
    P[0][0] = 1; P[1][1] = varV; P[2][2] = 1;
  }

  // freezeBias: no usable accel, so the input is a guess. Hold the bias at
  // zero and uncorrelated, or it would soak up the guess error and poison
  // velocity for the rest of the flight.
  void predict(float u, float dt, float sa, float sb, bool freezeBias = false) {
    if (freezeBias) {
      x[2] = 0;
      for (int i = 0; i < 3; i++) P[2][i] = P[i][2] = 0;
    }
    float dt2 = dt * dt;
    float a = u - x[2];
    x[0] += x[1] * dt + 0.5f * a * dt2;
    x[1] += a * dt;

    const float F[3][3] = {{1, dt, -0.5f * dt2}, {0, 1, -dt}, {0, 0, 1}};
    float FP[3][3], N[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        FP[i][j] = 0;
        for (int k = 0; k < 3; k++) FP[i][j] += F[i][k] * P[k][j];
      }
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        N[i][j] = 0;
        for (int k = 0; k < 3; k++) N[i][j] += FP[i][k] * F[j][k];
      }
    const float G[3] = {0.5f * dt2, dt, 0};
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) P[i][j] = N[i][j] + G[i] * G[j] * sa * sa;
    if (!freezeBias) P[2][2] += sb * sb * dt;
  }

  void update(float z, float sr) {
    float S = P[0][0] + sr * sr;
    float K[3] = {P[0][0] / S, P[1][0] / S, P[2][0] / S};
    float y = z - x[0];
    float row0[3] = {P[0][0], P[0][1], P[0][2]};
    for (int i = 0; i < 3; i++) {
      x[i] += K[i] * y;
      for (int j = 0; j < 3; j++) P[i][j] -= K[i] * row0[j];
    }
  }
};

// ------------------------------------------------------------------ flight
class Flight {
public:
  Tuning tun;
  State state = SAFE;
  float backupS = 0;           // backup deploy time, T+ s; 0 = not set (arming refused)

  // estimates, readable by the sketch
  KF    kf;
  Quat  q;
  float gRef = G0, P0 = 101325, gyroBias[3] = {0, 0, 0}, upBody[3] = {0, 0, 1};
  float hBaro = 0, hBaroSm = 0, aVert = 0, tilt = 0;
  float t = 0, maxAlt = 0, maxBaro = 0;
  bool  attitudeValid = true;  // false after an in-flight reset (orientation lost)
  const char *deployReason = "";
  char  msg[80] = "";

  void begin(float groundPa) {
    P0 = groundPa; state = SAFE; kf.reset(0); hBaroSm = 0;
  }

  bool arm() {
    if (state != SAFE) return false;
    calN = calPN = 0;
    memset(cs, 0, sizeof cs); memset(cs2, 0, sizeof cs2);
    memset(gs, 0, sizeof gs); memset(gs2, 0, sizeof gs2);
    ps = 0;
    state = CALIBRATING;
    snprintf(msg, sizeof msg, "Calibrating - keep the rocket still");
    return true;
  }

  bool disarm() {
    if (state == CALIBRATING || state == ARMED || state == LANDED) {
      state = SAFE;
      snprintf(msg, sizeof msg, "Disarmed");
      return true;
    }
    return false;
  }

  void save(Saved &s) const {
    s.state = state; s.P0 = P0; s.gRef = gRef; s.t = t;
    s.maxAlt = maxAlt; s.maxBaro = maxBaro; s.backupS = backupS;
    memcpy(s.gyroBias, gyroBias, sizeof gyroBias);
    memcpy(s.upBody, upBody, sizeof upBody);
  }

  // Resume after a reset. In flight the attitude is gone, so the filter
  // falls back to barometer-only velocity.
  void restore(const Saved &s) {
    P0 = s.P0; gRef = s.gRef; t = s.t; maxAlt = s.maxAlt; maxBaro = s.maxBaro;
    backupS = s.backupS;
    memcpy(gyroBias, s.gyroBias, sizeof gyroBias);
    memcpy(upBody, s.upBody, sizeof upBody);
    state = (State)s.state;
    if (state == BOOST) state = COAST;
    attitudeValid = state <= ARMED;
    if (state == CALIBRATING) state = SAFE;
    q = Quat::fromUp(upBody);
    needInit = true;
    snprintf(msg, sizeof msg, "Resumed %s after reset", STATE_NAME[state]);
  }

  uint16_t step(const Frame &fr) {
    uint16_t ev = 0;
    float dt = fr.dt < 0.001f ? 0.001f : (fr.dt > 0.05f ? 0.05f : fr.dt);
    bool pad = state <= ARMED;
    float fmag = sqrtf(fr.f[0] * fr.f[0] + fr.f[1] * fr.f[1] + fr.f[2] * fr.f[2]);

    // ---- attitude and vertical acceleration
    float u, sa;
    bool blind = false;
    if (fr.imuOk && attitudeValid) {
      float w[3] = {fr.w[0] - gyroBias[0], fr.w[1] - gyroBias[1], fr.w[2] - gyroBias[2]};
      if (pad) {
        if (fabsf(fmag - gRef) < 0.15f * gRef) {
          float up[3] = {fr.f[0] / fmag, fr.f[1] / fmag, fr.f[2] / fmag};
          q = Quat::fromUp(up);
        }
      } else {
        q.integrate(w, dt);
      }
      float fw[3], ax[3];
      q.rotate(fr.f, fw);
      q.rotate(upBody, ax);
      aVert = fw[2] - gRef;
      tilt = acosf(ax[2] > 1 ? 1 : (ax[2] < -1 ? -1 : ax[2])) * RAD2DEG;
      u = aVert;
      sa = fr.saturated ? tun.sigmaAccelSat : (pad ? tun.sigmaAccelPad : tun.sigmaAccelFlight);
    } else {
      aVert = NAN;
      u = pad ? 0 : -G0;
      sa = pad ? tun.sigmaAccelPad : tun.sigmaAccelBlind;
      blind = !pad;
    }

    // ---- Kalman filter
    if (needInit) {
      if (fr.baroNew) {
        hBaro = hBaroSm = altitude(fr.pressPa);
        kf.reset(hBaro, pad ? 0.01f : 400.0f);
        needInit = false;
      }
    } else {
      kf.predict(u, dt, sa, tun.sigmaBias, blind);
      if (fr.baroNew) {
        hBaro = altitude(fr.pressPa);
        hBaroSm += 0.2f * (hBaro - hBaroSm);
        kf.update(hBaro, pad ? tun.sigmaBaroPad : tun.sigmaBaroFlight);
      }
    }

    // ---- state machine
    switch (state) {
      case SAFE:
        break;

      case CALIBRATING:
        accumulate(fr);
        if (calN >= tun.calSamples) {
          if (finishCal()) { state = ARMED; ev |= EV_ARMED; }
          else             { state = SAFE;  ev |= EV_CAL_FAILED; }
        }
        break;

      case ARMED: {
        if (fr.imuOk && fmag > tun.launchAccelG * gRef) launchCnt++; else launchCnt = 0;
        bool byAccel = launchCnt >= tun.launchSamples;
        bool byBaro = fr.baroNew && hBaroSm > tun.launchBaroM;
        if (byAccel || byBaro) {
          state = BOOST;
          t = byAccel ? (tun.launchSamples - 1) * dt : 0;
          maxAlt = kf.x[0]; maxBaro = hBaroSm;
          burnCnt = apoCnt = 0;
          ev |= EV_LAUNCH;
          snprintf(msg, sizeof msg, "Launch (%s)", byAccel ? "accel" : "baro");
        }
        break;
      }

      case BOOST:
      case COAST: {
        t += dt;
        if (kf.x[0] > maxAlt) maxAlt = kf.x[0];
        if (fr.baroNew && hBaroSm > maxBaro) maxBaro = hBaroSm;

        if (state == BOOST) {
          float net = (fr.imuOk && attitudeValid) ? u - kf.x[2] : -1;
          burnCnt = net < 0 ? burnCnt + 1 : 0;
          if (burnCnt >= tun.burnoutSamples) {
            state = COAST; ev |= EV_BURNOUT;
            snprintf(msg, sizeof msg, "Burnout at T+%.2f s", t);
          }
        }

        bool high = maxAlt >= tun.minFlightAltM || maxBaro >= tun.minFlightAltM;
        if (!high && t > tun.falseLaunchS) {
          state = ARMED; launchCnt = 0;
          ev |= EV_FALSE_LAUNCH;
          snprintf(msg, sizeof msg, "False launch cancelled - back to ARMED");
          break;
        }

        const char *why = nullptr;
        if (state == COAST && high && t >= tun.apogeeLockoutS) {
          apoCnt = kf.x[1] < 0 ? apoCnt + 1 : 0;
          if (apoCnt >= tun.apogeeSamples) why = "velocity";
        }
        if (!why && high && maxBaro - hBaroSm >= tun.baroDescentM) why = "baro";
        if (!why && backupS > 0 && t >= backupS) why = "timer";
        if (why) {
          state = DESCENT; deployReason = why; landedT = 0;
          ev |= EV_DEPLOY;
          snprintf(msg, sizeof msg, "Deployed (%s) at T+%.2f s, %.1f m", why, t, kf.x[0]);
        }
        break;
      }

      case DESCENT:
        t += dt;
        landedT = fabsf(kf.x[1]) < tun.landedSpeed ? landedT + dt : 0;
        if (landedT >= tun.landedS) { state = LANDED; ev |= EV_LANDED; }
        break;

      case LANDED:
        t += dt;
        break;
    }
    return ev;
  }

  float altitude(float pa) const { return 44330.0f * (1.0f - powf(pa / P0, 0.190295f)); }

private:
  int calN = 0, calPN = 0, launchCnt = 0, burnCnt = 0, apoCnt = 0;
  double cs[3], cs2[3], gs[3], gs2[3], ps;
  float landedT = 0;
  bool needInit = false;

  void accumulate(const Frame &fr) {
    if (!fr.imuOk) return;
    for (int i = 0; i < 3; i++) {
      cs[i] += fr.f[i]; cs2[i] += (double)fr.f[i] * fr.f[i];
      gs[i] += fr.w[i]; gs2[i] += (double)fr.w[i] * fr.w[i];
    }
    calN++;
    if (fr.baroNew) { ps += fr.pressPa; calPN++; }
  }

  bool finishCal() {
    double a[3], g[3], aStd = 0, gStd = 0;
    for (int i = 0; i < 3; i++) {
      a[i] = cs[i] / calN; g[i] = gs[i] / calN;
      double va = cs2[i] / calN - a[i] * a[i], vg = gs2[i] / calN - g[i] * g[i];
      aStd = fmax(aStd, sqrt(fmax(va, 0))); gStd = fmax(gStd, sqrt(fmax(vg, 0)));
    }
    float amag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (aStd > tun.calMaxAccelStd) {
      snprintf(msg, sizeof msg, "Arm failed: rocket moving (accel sd %.2f m/s2)", aStd); return false; }
    if (gStd > tun.calMaxGyroStd) {
      snprintf(msg, sizeof msg, "Arm failed: rocket moving (gyro sd %.2f deg/s)", gStd * RAD2DEG); return false; }
    if (fabsf(amag - G0) > 0.1f * G0) {
      snprintf(msg, sizeof msg, "Arm failed: accel reads %.2f m/s2 at rest, expected 9.8", amag); return false; }
    if (calPN < 20) {
      snprintf(msg, sizeof msg, "Arm failed: only %d baro readings", calPN); return false; }

    gRef = amag;
    for (int i = 0; i < 3; i++) { upBody[i] = a[i] / amag; gyroBias[i] = g[i]; }
    P0 = ps / calPN;
    q = Quat::fromUp(upBody);
    kf.reset(0);
    hBaro = hBaroSm = 0; maxAlt = maxBaro = 0; t = 0;
    launchCnt = 0; attitudeValid = true; deployReason = "";
    snprintf(msg, sizeof msg, "ARMED. g=%.3f m/s2, ground %.0f Pa", gRef, P0);
    return true;
  }
};

}  // namespace fl
