# Electronics

![License: MIT](https://img.shields.io/badge/License-MIT-orange.svg)
![Platform](https://img.shields.io/badge/platform-Arduino-blue.svg)
![Status](https://img.shields.io/badge/status-active-green.svg)

Flight computer firmware, sensor integration, telemetry code and circuit diagrams for our student-built rockets. Every file here was written by students and published openly so other teams don't have to start from scratch.

We're the Indian Students Rocketry Association, a student rocketry association from Anand, Gujarat, building towards 100 km — one mission at a time.

---

## Missions

| Mission | Rocket | Status | What's there |
|---|---|---|---|
| [M001](Missions/M001-SRM/) | SRM | Flew 2024-10-20. Motor exploded on ignition | Parts list only: Arduino Nano, ADXL345, BME280, nRF24L01+ telemetry |
| [M002](Missions/M002-HomiSI/) | Homi Shatam I | Flew 2025-03-11. Motor exploded within 1 s | Parts list only: Arduino Nano, MPU6050, BMP180, SD logging |
| [M003](Missions/M003-HomiSII/) | Homi Shatam II | In development, not yet flown | ESP32-CAM + GY-87 flight computer with Kalman-filtered apogee detection and servo parachute deploy, a desktop flight simulator, bench-test logs |
| [M004](Missions/M004-UnityKM/) | UnityKM | Not started | — |

No code or data survives from M001 and M002. Both boards were destroyed and nothing was committed at the time. Their READMEs were written afterwards from memory.

```
Missions/
├── M001-SRM/              # README only
├── M002-HomiSI/           # README only
├── M003-HomiSII/
│   ├── README.md          # State, test results, known issues — start here
│   ├── FlightComputer/    # Firmware, wiring, circuit diagram, build settings
│   ├── FlightSim/         # Runs the flight logic against simulated flights
│   └── Test/              # Logs from board tests, one folder per test (YYYY-MM-DD_Name)
└── M004-UnityKM/          # Placeholder
```

Each mission folder is self-contained, and its `README.md` says honestly what works and what hasn't been tested yet.

---

## Getting Started

1. Clone the repo
   ```bash
   git clone https://github.com/Indian-Students-Rocketry-Assocation/Electronics.git
   ```
2. Read the mission's `README.md`, then `FlightComputer/README.md` for wiring, board settings and libraries
3. Open the `FlightComputer/` folder in the Arduino IDE and upload

> **Note:** We use the Arduino IDE. If you're adapting this for PlatformIO, it should port cleanly — open a Discussion and let us know how it goes.

---

## Commit Convention

Every commit is prefixed with a mission number or `[General]`:

```
[M003] Add apogee detection logic v2
[M003] Fix SD card initialisation on cold boot
[General] Refactor continuity check function
```

---

## Contributing

Found a better approach to apogee detection? Noticed a bug? We're students — we want to know.

1. Open an issue describing the problem or idea
2. Fork the repo and make your changes
3. Submit a pull request with a clear description of what changed and why

Questions and open-ended discussion belong in [Discussions](../../discussions).

---

## License

MIT License — see [LICENSE](LICENSE). Use it, adapt it, build on it.

---

## About Us

We're a student rocketry association from Anand, Gujarat. This repository is part of our commitment to open-source rocketry education. Everything we learn, we share.

[🔗 All Repos](https://github.com/Indian-Students-Rocketry-Assocation) · [💬 Discussions](../../discussions)
