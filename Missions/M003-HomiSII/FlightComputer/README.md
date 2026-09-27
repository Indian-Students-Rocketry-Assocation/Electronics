# HOMIS-II flight computer

ESP32-CAM + GY-87 + MG90S. Estimates altitude and vertical velocity, detects
apogee and turns the servo from **lock** to **release** to eject the parachute.
Built on [`../Logger/Logger.ino`](../Logger/Logger.ino): same wiring, same
Arduino IDE settings, no extra libraries.

**Status (2026-09-26):** compiles (esp32 core 3.3.12) and passes the desktop
simulator. **Not yet run on the board.** Do the bench tests below before any flight.

## Files

| File | What |
|---|---|
| `FlightComputer.ino` | Sensors, servo, SD, camera, Wi-Fi, tasks |
| `FlightLogic.h` | Estimator + state machine. Plain C++, no hardware |
| `WebPage.h` | Pre-launch page |
| `../FlightSim/flight_sim.cpp` | Runs `FlightLogic.h` against simulated flights |

## How velocity is estimated

1. **Attitude.** On the pad, gravity gives the board's orientation, so mounting
   direction does not matter. After launch the gyro is integrated, so the accel
   is projected onto the true vertical even as the rocket tilts.
2. **Vertical acceleration** = accel rotated to the world frame, minus the 1 g
   the board measured at rest (this cancels accelerometer scale error).
3. **Kalman filter** with states altitude, velocity and accel bias. Accel drives
   the prediction at 100 Hz, and the baro (50 Hz) corrects it. When the accel
   clips at 16 g, the filter trusts the baro more. If the IMU stops responding,
   it runs on the baro alone.

## When it deploys

`SAFE → CALIBRATING → ARMED → BOOST → COAST → DESCENT → LANDED`

| Trigger | Condition |
|---|---|
| Launch | \|accel\| > 2.5 g for 50 ms, or baro > 20 m above the pad |
| Burnout | net vertical accel < 0 for 100 ms |
| **Deploy: velocity** (primary) | in COAST, T+ ≥ 1.5 s, filtered velocity < 0 for 100 ms |
| Deploy: baro (backup) | baro altitude 5 m below its maximum |
| Deploy: timer (backup) | T+ ≥ the backup timer set on the page |
| False launch | a "launch" that stays under 5 m for 3 s goes back to ARMED |
| Landed | \|v\| < 1 m/s for 5 s |

All deploys require launch to have been detected. Every threshold is a default
in `fl::Tuning` in `FlightLogic.h`. None of them comes from a flight test, so
check them against the OpenRocket sim for this rocket.

## Wi-Fi pre-launch page

Join **`HOMIS2-AVIONICS`**, password `isra-m003` (change `AP_PASS`), and open
**http://192.168.4.1**.

- **Checks:** MPU6050 and BMP180 detected *and* producing changing readings,
  accel reads 1 g at rest, SD card, camera, servo position, backup timer, log drops.
- **Live:** filtered and raw altitude, velocity, vertical accel, gyro rate, pressure.
- **Servo test** and **settings** (backup timer, lock angle, release angle).
  These work only in SAFE and are saved on the board.
- **ARM** is refused unless the IMU and baro are reading, the SD card works, the
  backup timer is set and the servo is locked. Arming then calibrates for 2 s.
  The rocket must be still, or arming fails and says why.

Wi-Fi turns off at launch and comes back on after landing.

## Logs (`/run_NNN/` on the SD card)

- `flight.csv`: every 10 ms from ARM to landing, 10 Hz otherwise. Columns: `ms,state,p_Pa,temp_C,h_baro_m,h_m,v_ms,a_vert_ms2,acc_bias,ax,ay,az,gx,gy,gz,tilt_deg,sat`
- `events.csv`: boot, arm, launch, burnout, deploy (with reason), landing, resets
- `photos.csv` + `IMG_nnnnnn.jpg`: one photo per second, as before

## Reset during flight

The flight state lives in RTC memory, which survives a reset but not a power
cycle. After a brown-out or crash reset while ARMED or flying, the board resumes
where it was: the servo stays released if it already fired, and velocity comes
from the baro alone because the attitude is lost. The reset button and power-on
count as a normal boot.

## Before flying

1. **Power.** The servo and ESP32 share 5 V. Put a 470–1000 µF capacitor across
   the servo supply, close to the servo, so a servo stall doesn't brown out the
   ESP32. The code survives a reset, but it is better not to have one.
2. **Flash LED.** It shares GPIO4 and glows with the servo pulses. Cover it.
3. **Servo angles.** On the page, set lock and release so that **Lock** holds
   the mechanism and **Release** ejects it. Pulse range: `SERVO_MIN_US` /
   `SERVO_MAX_US` (500–2400 µs for 0–180°). On power-up the servo goes to
   the lock angle, so set it before loading the parachute.
4. **Backup timer.** Set it to the OpenRocket apogee time plus about 2 s.
5. **Bench tests** (not yet done):
   - All checks OK on the page. Lock/Release move the mechanism the right way.
   - Arm and tap the board: no launch in `events.csv`.
   - Arm and swing the board hard in a circle (well over 2.5 g). It should
     detect launch, then log a false launch after 3 s and go back to ARMED.
   - Arm it inside a sealed jar, pump the pressure down, then let it back up.
     It should detect launch by baro, then deploy.
   - `logDrops` on the page stays 0 while photos are being saved.
6. **Simulate your rocket.** Put the OpenRocket mass, Cd·A and thrust in
   `Rocket` in `flight_sim.cpp` and run it (below).

## Simulator

```bash
cd ../FlightSim && c++ -std=c++17 -O2 -o flight_sim flight_sim.cpp && ./flight_sim
```

Results with the placeholder rocket (208 m apogee, 27 g spike that clips the
16 g accel, 5° rail tilt, board mounted sideways), 500 noisy runs per case:

| Case | Deploy − true apogee | Deployed by |
|---|---|---|
| All sensors | +0.11 s (sd 0.01, worst +0.14) | velocity |
| IMU dies at launch | +0.10 s (worst +0.12) | velocity (baro only) |
| Baro dies at launch | −0.42 s (worst −0.78) | velocity (accel only) |
| Reset at T+2 s | +0.11 s | velocity |
| Reset at T+5.5 s (1.5 s reboot) | +0.86 s | velocity, straight after reboot |
| 40 ms 5 g knock on the pad | never deploys | — |

The +0.1 s is the 100 ms confirmation window. The baro-dead case deploys early
because the 16 g clipping loses velocity during the spike.

## Known limits

- With the accelerometer saturated (> 16 g), velocity during that part comes
  from the baro, which is slower.
- The BMP180 sits inside the airframe. Vent holes and a bay sealed from the
  motor and parachute gases matter as much as the code.
- Single deployment only (no drogue/main split).
