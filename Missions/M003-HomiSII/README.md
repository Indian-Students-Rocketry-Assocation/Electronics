# Homi Shatam II - Electronics

Avionics for HOMIS-II: an ESP32-CAM with a GY-87 (MPU6050 accel/gyro + BMP180
barometer) and an MG90S servo. The board logs data and photos to SD, works out
altitude and vertical velocity, and turns the servo 90° at apogee to eject the
parachute. Before launch, a Wi-Fi page shows sensor health and is used to arm it.
The servo and ESP32 share one 5 V supply.

| Folder | What |
|---|---|
| [`FlightComputer/`](FlightComputer/) | Flight firmware, wiring and build settings. Start with [its README](FlightComputer/README.md) |
| [`FlightSim/`](FlightSim/) | Desktop simulator that runs the flight logic against simulated flights |
| [`Test/`](Test/) | Logs from board tests, one folder per test, plus `plot_flight.py` |

## Current state (2026-09-27)

**Not flight-ready.** The firmware runs on the board and has passed one indoor
test (below). It has not flown, had a pressure test, or been given the real rocket's numbers.

| Part | State | Evidence |
|---|---|---|
| Sensors, SD, camera, 100 Hz loop | ✅ Working on the board | Stair test: 200 s log, 100 Hz while armed, no gaps (max 11 ms between rows) |
| Kalman filter at rest | ✅ Working | Stair test, 5–118 s at rest: filtered velocity between −0.5 and +0.7 m/s |
| Kalman filter tracking a height change | ✅ Working | Stair test: one floor down and back up, filtered altitude reached −3.2 m and came back |
| Pad calibration / ARM | ✅ Working | Calibrated in 2 s. Vertical accel offset went from −0.29 to 0.00 m/s² |
| No false launch when carried | ✅ Passed | Stair test: 80 s armed while carried, up to 1.6 g and 104° tilt, no launch triggered |
| Apogee detection + deploy | 🧪 Simulator only | Deploys 0.11 s after apogee (500 runs). Placeholder rocket, not HOMIS-II |
| Servo actually ejecting the chute | ❓ Not tested | Lock/release angles are defaults (0° / 90°) |
| Recovery after a reset | 🧪 Simulator only | Never tested on the board |
| Deploy thresholds | ⚠️ Defaults, unsourced | `fl::Tuning` in `FlightLogic.h`: 2.5 g launch, 5 m baro drop, … |

The first board run (`boot: mpu=0 bmp=0`) had both sensors missing. The next run,
on the same code, found them. The cause was not recorded; if it happens again,
check the GY-87 wiring first.

## Tests

| Date | Test | Result |
|---|---|---|
| 2026-09-26 | [Stair test](Test/2026-09-26_Stair-Test/): armed indoors, carried from the 1st floor to the ground floor and back | Altitude tracked the floor change; no false launch. Only `flight.csv` was kept, no `events.csv` |

To plot a log: `python3 Test/plot_flight.py <flight.csv> <out.png>` (needs matplotlib).

## Known issues

1. **Baro-only launch detection can be set off by drift.** While armed,
   baro altitude > 20 m counts as launch, even with no speed. That takes about
   2.4 hPa of drift (weather, or sun heating the sensor over a long pad wait),
   and after that nothing cancels it, so it would deploy on the pad. Fix: also
   require filtered velocity > 5 m/s, then rerun the simulator.
2. **Reset recovery relies on RTC memory surviving a brown-out.** Not verified
   on this board.
3. **Sensors are only looked for at boot.** If a connector is loose at power-up,
   the board stays at "sensor FAIL" until it is reset.
4. Minor: the first log row has `p_Pa=0` (logged before the first baro reading),
   and `tilt_deg` means nothing before arming.

## Next steps

1. Fix issues 1 and 3.
2. **Servo:** find the lock and release angles with the real mechanism. Add a
   470–1000 µF capacitor on the servo 5 V, and cover the flash LED.
3. **Put HOMIS-II in the simulator** (mass, thrust curve, Cd, rail from
   OpenRocket). Set the thresholds and the backup timer from it.
4. **Bench tests** ([list](FlightComputer/README.md#before-flying)): swing test
   (false launch), sealed-jar pressure test (baro launch + deploy), reset test,
   a full servo eject with the chute packed. Keep both `events.csv` and `flight.csv`.
5. Full dress rehearsal: rocket assembled, armed on the rail for as long as a
   real launch day, powered from the flight battery.
6. Fly. Afterwards, copy `run_NNN/` into `Test/` and compare the logged apogee with OpenRocket.
