# SRM - Electronics

ISRA's first rocket (Simplified Rocket Mission). Launched **20 October 2024** in
Anand, Gujarat. The motor exploded on ignition.

> Written on 2026-09-27 from the team's memory. No code, wiring or data from
> this mission survives, so this is the whole record.

## What flew

| Part | Role |
|---|---|
| Arduino Nano | Microcontroller |
| ADXL345 | Accelerometer |
| BME280 | Pressure / temperature / humidity (altitude) |
| nRF24L01+ | 2.4 GHz radio telemetry |

**Ignition** was ground-side: an igniter on a long wire, fired from a 12 V
battery. It was not connected to the flight board.

Not recorded: the firmware, the wiring, the ground receiver, the battery, and
whether the board did anything besides send data.

## Outcome

The motor exploded at ignition and destroyed the avionics. No telemetry was
received and no data survived.
