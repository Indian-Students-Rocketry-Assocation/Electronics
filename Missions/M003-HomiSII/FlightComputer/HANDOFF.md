# Flight computer — handoff

Where the HOMIS-II flight computer work stopped on 2026-09-26, what is needed
to continue, and what to do with each piece once it arrives.

For an AI agent picking this up: read this file, then [README.md](README.md),
then `FlightLogic.h`. Follow `~/Documents/03_Work/ISRA/AGENTS.md`: nothing about
recovery or safety margins without a source, and commits are prefixed `[M003]`.

## Goal

The ESP32-CAM + GY-87 board works out altitude and vertical velocity, detects
apogee and turns the MG90S servo 90° to eject the parachute. Before launch, a
Wi-Fi page shows whether the sensors are working and is used to arm the board.
The servo and ESP32 share one 5 V supply.

## State

| Part | State |
|---|---|
| `FlightLogic.h`: Kalman filter (altitude, velocity, accel bias), attitude from gyro, launch/burnout/apogee state machine, 3 deploy triggers | Done. Passes `../FlightSim/` (500 runs per case, see README) |
| `FlightComputer.ino`: 100 Hz IMU, 50 Hz baro, servo on LEDC timer 1, SD logs, camera, recovery after a reset | Compiles (esp32 core 3.3.12, 36% flash). **First board run: sensors not found**, see below |
| `WebPage.h`: checks, live data, servo test, settings, ARM | Compiles. Not yet opened on a phone |
| Rocket numbers in the simulator | **Placeholders**, not HOMIS-II |
| Deploy thresholds (`fl::Tuning`) | Defaults I chose. Not from a test or a source |
| Nothing committed | `FlightComputer/`, `FlightSim/`, `Test/` are untracked |

### Open bug: first board run found no sensors

`../Test/events.csv` shows `boot: mpu=0 bmp=0 sd=1`. `../Test/flight.csv` has 50 s
of SAFE with every sensor column at zero. SD, camera and saved settings all work
(`backup=10.00 s` was read back from flash).

Both sensors failing at once points to the I²C bus, not one chip. Only one
start-up step differs from `Logger.ino`, which did find both sensors:

- **Logger:** SD → camera → `Wire.begin` (sensors start about 1.5 s after power-up)
- **FlightComputer:** SD → servo → `Wire.begin` → sensors → … → camera

Also: when a sensor is missing at boot, the firmware never looks for it again,
so the page stays at FAIL until the next reset.

## Info needed from you

Give these in any form (paste, screenshot, file path). Each one says what it unlocks.

### 1. Sensor bug (blocks everything else)
- [ ] **Serial monitor output** from power-up with `FlightComputer`, at 115200 baud.
      The `MPU6050 WHO_AM_I` / `MPU6050: … BMP180: …` lines matter most.
- [ ] **Does `Logger.ino` still find both sensors** on the same wiring, right now?
      Yes → firmware problem. No → a wire or the GY-87.

### 2. Rocket (for the simulator and the thresholds)
Taken from the HOMIS-II OpenRocket file (`ISRA/Hardware/Rocket Designs/`).
- [ ] Liftoff mass and burnout mass (kg)
- [ ] Motor: name, or its thrust curve (`.eng` file or a time/thrust table),
      plus burn time and peak thrust
- [ ] Max acceleration (g). This decides whether ±16 g clips.
- [ ] Apogee altitude (m) and **time to apogee (s)**
- [ ] Optimum ejection delay (s)
- [ ] Cd and reference diameter, or OpenRocket's drag numbers
- [ ] Launch rail length and angle
- [ ] Descent rate under the parachute (m/s)

### 3. Hardware
- [ ] Servo angles that actually lock and release the mechanism (test from the page)
- [ ] Is there a capacitor across the servo 5 V yet? Does the board reset
      when the servo moves (look for `RESET in` in `events.csv`)?
- [ ] Is the flash LED covered?
- [ ] What the 5 V source is (battery + regulator? capacity?) and how long the
      rocket may sit armed on the pad

### 4. Bench tests
One `run_NNN/` folder (`events.csv` + `flight.csv`) per test, copied into `../Test/`,
named for the test. The tests are listed in README → *Before flying*.

## What to do when each arrives

**Serial log / Logger result (1)**
1. If Logger works and FlightComputer doesn't: in `setup()`, move `Wire.begin` and
   the sensor start-up (`mpuInit`, `bmpInit`, the ground-pressure average) after
   `initCamera()`, as in Logger. Start the flight task after that.
2. In either case, have the flight task try `mpuInit()` / `bmpInit()` again
   about once a second while in SAFE, so a loose connector recovers without a reset.
3. Compile, then ask for a new `Test/` run. Pass: `boot: mpu=1 bmp=1`, pressure
   and accel columns not zero, page checks green.

**Rocket numbers (2)**
1. Put them into `struct Rocket` in `../FlightSim/flight_sim.cpp`. Replace the
   `thrust()` shape with the real curve (a table + interpolation).
2. Run it: `cd ../FlightSim && c++ -std=c++17 -O2 -o flight_sim flight_sim.cpp && ./flight_sim`
3. Check against OpenRocket: the simulated apogee and apogee time should roughly
   match. If they don't, the drag or mass is wrong.
4. Retune `fl::Tuning` only with a reason from the numbers. `launchAccelG` must stay
   well below the real liftoff acceleration. `apogeeLockoutS` must be longer than
   the burn. If the peak is over 16 g, note the clipping. The baro backup covers
   it, but check the baro-dead case.
5. Backup timer = apogee time + about 2 s. It is set on the page, not in the code.
6. Update the results table in README.md.

**Hardware answers (3)**
- Put the working servo angles in `DEFAULT_LOCK_DEG` / `DEFAULT_RELEASE_DEG`, so a
  new board starts right even before the page is used.
- If there are resets when the servo moves: add the capacitor first, then look again.
- Pad time × current draw (camera + Wi-Fi ≈ 300 mA peaks, not measured) vs battery
  capacity. Tell them if it's tight.

**Bench logs (4)**
Read `events.csv` first, then plot `flight.csv` (`h_m`, `h_baro_m`, `v_ms`,
`a_vert_ms2` against `ms`). Check: no gaps in `ms` (log drops), the velocity is
near 0 at rest, the right event fired and nothing extra did. Record each result
in README → *Before flying* and in the State table above.

## After that
- Commit: `[M003] Add flight computer: Kalman apogee detection, servo deploy, Wi-Fi arming`
  (only `Missions/M003-HomiSII/`, no `.DS_Store`).
- Update `~/Documents/03_Work/ISRA/ORG.md` → Electronics workstream once it
  passes bench tests.
- As items close, tick them here. When nothing is open, ask Dhairya whether to archive this file.
