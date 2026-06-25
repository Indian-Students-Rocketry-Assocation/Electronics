# Electronics

![License: MIT](https://img.shields.io/badge/License-MIT-orange.svg)
![Platform](https://img.shields.io/badge/platform-Arduino-blue.svg)
![Status](https://img.shields.io/badge/status-active-green.svg)

Flight computer firmware, sensor integration, telemetry code and any other piece of code or circuit diagrams for our student-built rockets. Every file here was written by students, tested in the field, and published openly so other teams don't have to start from scratch.

We're a student rocketry association from Anand, Gujarat, building towards 100 km — one mission at a time.

---

## What's in here

```
avionics/
├── missions/
│   ├── M001-HomiSII/          # Firmware and configs as flown in Mission 001
│   ├── M002/
│   └── ...
└── resources/
    ├── wiring-diagrams/      # Annotated schematics for each hardware config
    ├── component-guides/     # Notes on specific sensors and modules we've used
```

Each mission folder is self-contained. You'll find the firmware exactly as it was at launch — not a cleaned-up version.

---

## Getting Started

1. Clone the repo
   ```bash
   git clone https://github.com/[org-name]/avionics.git
   ```
2. Open the relevant mission folder in Arduino IDE
3. Read the `README.md` inside the mission folder for pinout and library dependencies
4. Install required libraries via Arduino Library Manager — listed per mission

> **Note:** We use Arduino IDE. If you're adapting this for PlatformIO, it should port cleanly — open a Discussion and let us know how it goes.

---

## Commit Convention

Every commit is prefixed with a mission number or `[General]`:

```
[M001] Add apogee detection logic v2
[M001] Fix SD card initialisation on cold boot
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

[🔗 All Repos](https://github.com/[org-name]) · [💬 Discussions](../../discussions)
