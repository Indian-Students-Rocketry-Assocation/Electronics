/*
  Desktop test for ../FlightComputer/FlightLogic.h
  Build:  c++ -std=c++17 -O2 -o flight_sim flight_sim.cpp && ./flight_sim

  Simulates a 2-D flight (vertical plane) with a tilted rail, weathercocking,
  a thrust curve that clips the 16 g accelerometer, drag, and sensor noise
  similar to an MPU6050 + BMP180. Then runs the same Flight class the ESP32
  runs and reports when it deployed versus the true apogee.

  The rocket numbers below are placeholders, not HOMIS-II. Replace them with
  the OpenRocket values to test the real flight.
*/
#include "../FlightComputer/FlightLogic.h"
#include <random>
#include <vector>
#include <cstdio>

struct Rocket {
  float mass = 0.45f;        // kg, placeholder
  float cdA = 0.0035f;       // Cd * area, m^2, placeholder
  float peakThrust = 120.0f; // N (early spike, clips 16 g), placeholder
  float avgThrust = 25.0f;   // N, placeholder
  float burn = 1.6f;         // s, placeholder
  float railTiltDeg = 5.0f;
};

float thrust(const Rocket &r, float t) {
  if (t < 0 || t > r.burn) return 0;
  if (t < 0.1f) return r.peakThrust * t / 0.1f;
  if (t < 0.2f) return r.peakThrust + (r.avgThrust - r.peakThrust) * (t - 0.1f) / 0.1f;
  return r.avgThrust;
}

struct Opts {
  bool imuDead = false;      // accelerometer/gyro stops at launch
  bool baroDead = false;     // barometer stops at launch
  bool bumpOnly = false;     // someone knocks the rocket on the pad, no launch
  float resetAt = -1;        // brown-out reset at this T+, 1.5 s reboot, restore from RTC
  float backupS = 0;         // 0 = true apogee + 2 s
  unsigned seed = 1;
};

struct Result { float trueApogeeT, trueApogeeH, deployT, deployH, deployV; const char *why; bool launched; };

Result run(const Rocket &r, const Opts &o, bool verbose = false) {
  std::mt19937 rng(o.seed);
  std::normal_distribution<float> n01(0, 1);

  const float dt = 0.01f, rho = 1.2f, P0 = 100800;
  // sensor mounting: arbitrary body frame, rocket axis along body -Y (as if the
  // board is mounted sideways) to prove orientation does not matter
  auto toBody = [](float along, float across, float out[3]) {
    out[0] = across; out[1] = -along; out[2] = 0;
  };
  float accBias[3] = {0.15f * n01(rng), 0.15f * n01(rng), 0.15f * n01(rng)};
  float accScale = 1.0f + 0.02f * n01(rng);
  float gyroBias[3] = {0.02f * n01(rng), 0.02f * n01(rng), 0.02f * n01(rng)};

  fl::Flight f;
  f.begin(P0);
  f.backupS = 1;  // placeholder so arm() is allowed; set properly after the dry run
  f.arm();

  // world state: x horizontal, z up
  float pz = 0, vx = 0, vz = 0;
  float theta = (90.0f - r.railTiltDeg) / fl::RAD2DEG;  // pitch from horizontal
  float railLen = 1.0f, padWait = 3.0f, tLaunch = padWait;
  bool launched = false, onRail = true;
  float trueApoT = 0, trueApoH = 0;
  Result res{0, 0, -1, 0, 0, "none", false};

  for (int i = 0; i < 12000; i++) {
    float t = i * dt;
    float tl = t - tLaunch;
    float T = o.bumpOnly ? 0 : thrust(r, tl);
    float v = sqrtf(vx * vx + vz * vz);
    float D = 0.5f * rho * r.cdA * v * v;

    float omega = 0;
    if (pz > railLen || !onRail) onRail = false;
    if (!onRail && v > 1) {
      // weathercock: pitch follows the flight path with a first-order lag
      float path = atan2f(vz, vx);
      omega = (path - theta) * 8.0f;
    }
    theta += omega * dt;

    // specific force along/across the body (what an accelerometer sees)
    float fAlong = 0, fAcross = 0;
    float ax = 0, az = 0;
    if (tl >= 0 && !(onRail && T / r.mass < fl::G0 && pz <= 0)) {
      if (onRail) {
        // rail carries the cross-axis load, so the accelerometer sees it
        fAlong = (T - D) / r.mass;
        fAcross = fl::G0 * cosf(theta);
        float aRail = fAlong - fl::G0 * sinf(theta);
        ax = aRail * cosf(theta); az = aRail * sinf(theta);
      } else {
        float ux = v > 0.1f ? vx / v : cosf(theta), uz = v > 0.1f ? vz / v : sinf(theta);
        ax = (T * cosf(theta) - D * ux) / r.mass;
        az = (T * sinf(theta) - D * uz) / r.mass - fl::G0;
        // body-frame specific force = rotate (a - g) into body
        float sx = ax, sz = az + fl::G0;
        fAlong = sx * cosf(theta) + sz * sinf(theta);
        fAcross = -sx * sinf(theta) + sz * cosf(theta);
      }
      vx += ax * dt; vz += az * dt; pz += vz * dt;
      if (pz < 0 && tl > 1) { pz = 0; vx = vz = 0; }
      if (!launched && pz > 0) launched = true;
    } else {
      // at rest on the pad: accelerometer reads +g along the world vertical
      fAlong = fl::G0 * sinf(theta);
      fAcross = fl::G0 * cosf(theta);
      if (o.bumpOnly && t > tLaunch && t < tLaunch + 0.04f) fAlong += 4 * fl::G0;  // 40 ms knock
    }
    if (pz > trueApoH) { trueApoH = pz; trueApoT = tl; }

    // ---- sensors
    fl::Frame fr{};
    fr.dt = dt;
    float fb[3]; toBody(fAlong, fAcross, fb);
    fr.imuOk = !(o.imuDead && tl > 0);
    fr.saturated = false;
    for (int k = 0; k < 3; k++) {
      float m = fb[k] * accScale + accBias[k] + 0.08f * n01(rng);  // + motor vibration
      if (tl > 0 && tl < r.burn) m += 1.5f * n01(rng);
      float lim = 16 * fl::G0;
      if (m > lim) { m = lim; fr.saturated = true; }
      if (m < -lim) { m = -lim; fr.saturated = true; }
      fr.f[k] = m;
    }
    float wb[3] = {0, 0, omega};  // pitch about body Z (plane of flight)
    for (int k = 0; k < 3; k++) fr.w[k] = wb[k] + gyroBias[k] + 0.003f * n01(rng);

    fr.baroNew = (i % 2 == 0) && !(o.baroDead && tl > 0);
    float Pa = P0 * powf(1 - pz / 44330.0f, 5.255f);
    fr.pressPa = Pa + 3.0f * n01(rng);   // ~0.25 m rms, BMP180 OSS2 class

    if (o.resetAt > 0 && tl >= o.resetAt && tl < o.resetAt + 1.5f) {
      if (tl < o.resetAt + dt / 2) {         // save on the way down, lose everything else
        fl::Saved sv; f.save(sv);
        f = fl::Flight(); f.restore(sv);
      }
      continue;                              // rebooting: no frames
    }
    uint16_t ev = f.step(fr);
    if (verbose && ev) printf("  t=%6.2f  ev=%3u  %s\n", tl, ev, f.msg);
    if (ev & fl::EV_ARMED) f.backupS = o.backupS;
    if (ev & fl::EV_DEPLOY) {
      res.deployT = tl; res.deployH = pz; res.deployV = vz; res.why = f.deployReason;
    }
    if (verbose && i % 20 == 0 && tl > -0.1f && tl < trueApoT + 3)
      printf("  t=%5.2f %-8s h=%7.2f kf_h=%7.2f v=%7.2f kf_v=%7.2f tilt=%5.1f\n", tl,
             fl::STATE_NAME[f.state], pz, f.kf.x[0], vz, f.kf.x[1], f.tilt);
    if (res.deployT >= 0 && tl > res.deployT + 1) break;
  }
  res.trueApogeeT = trueApoT; res.trueApogeeH = trueApoH; res.launched = f.state >= fl::BOOST;
  return res;
}

int main() {
  Rocket r;

  // dry run to find the true apogee time (stands in for the OpenRocket number)
  Opts dry; dry.backupS = 100;
  Result d = run(r, dry);
  float backup = d.trueApogeeT + 2.0f;
  printf("Placeholder rocket: apogee %.1f m at T+%.2f s -> backup timer %.2f s\n\n",
         d.trueApogeeH, d.trueApogeeT, backup);

  printf("Nominal flight, verbose:\n");
  Opts o; o.backupS = backup; o.seed = 7;
  Result v = run(r, o, true);
  printf("-> deployed by %s at T+%.2f s (true apogee T+%.2f), %.1f m of %.1f m, v=%.2f m/s\n\n",
         v.why, v.deployT, v.trueApogeeT, v.deployH, v.trueApogeeH, v.deployV);

  const int N = 500;
  struct Case { const char *name; bool imu, baro; float reset; };
  Case cases[] = {{"all sensors", false, false, -1}, {"IMU dies at launch", true, false, -1},
                  {"baro dies at launch", false, true, -1}, {"reset at T+2 s", false, false, 2.0f},
                  {"reset at T+5.5 s", false, false, 5.5f}};
  int fail = 0;
  for (auto &c : cases) {
    std::vector<float> err;
    int reasons[3] = {0, 0, 0};
    float worst = 0;
    for (int s = 0; s < N; s++) {
      Opts m; m.backupS = backup; m.seed = 100 + s; m.imuDead = c.imu; m.baroDead = c.baro; m.resetAt = c.reset;
      Result x = run(r, m);
      if (x.deployT < 0) { fail++; printf("  seed %d: NO DEPLOY\n", s); continue; }
      float e = x.deployT - x.trueApogeeT;
      err.push_back(e);
      if (fabsf(e) > fabsf(worst)) worst = e;
      if (!strcmp(x.why, "velocity")) reasons[0]++;
      else if (!strcmp(x.why, "baro")) reasons[1]++;
      else reasons[2]++;
    }
    float mean = 0; for (float e : err) mean += e; mean /= err.size();
    float sd = 0; for (float e : err) sd += (e - mean) * (e - mean); sd = sqrtf(sd / err.size());
    printf("%-22s deploy - apogee: mean %+.3f s, sd %.3f s, worst %+.3f s | velocity %d, baro %d, timer %d\n",
           c.name, mean, sd, worst, reasons[0], reasons[1], reasons[2]);
    if (fabsf(worst) > 2.5f) fail++;
  }

  // pad knock must not deploy
  int bumpDeploys = 0;
  for (int s = 0; s < N; s++) {
    Opts b; b.bumpOnly = true; b.backupS = backup; b.seed = 900 + s;
    if (run(r, b).deployT >= 0) bumpDeploys++;
  }
  printf("%-22s deployed in %d of %d runs\n", "40 ms 5 g pad knock", bumpDeploys, N);
  if (bumpDeploys) fail++;

  printf("\n%s\n", fail ? "FAIL" : "PASS");
  return fail ? 1 : 0;
}
