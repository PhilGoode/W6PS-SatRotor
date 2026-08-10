# W6PS SatRotor

Satellite antenna rotator controller for amateur radio satellite work —
current generation.

An Arduino GIGA R1 WiFi drives a Pelco-D PTZ head over RS-485 with
closed-loop position tracking, GPS timing, rotary-encoder jog control, and
runtime-tunable speed/tolerance calibration, all debugged against the real
hardware on the bench. An Arduino UNO Q is being brought up alongside it to
take on the tracking intelligence — pass prediction and orbital math on the
Linux side, with camera-based pointing verification planned next.

**Demo video:** https://youtu.be/dlWk-pP1Sz0

## What's here

- `firmware/sketches/giga_pelco_rotor/` — the GIGA controller sketch
- `firmware/sketches/ptz_azimuth_spoof/` — firmware for the custom spoof
  board that lives inside the PTZ head
- `experiments/skyfield_probe/` — Skyfield/SGP4 orbital-math groundwork for
  the UNO Q side
- Spoof board wiring and reference notes in the repo root

## Previous generation

The first-generation controller — STM32 Nucleo + Raspberry Pi touchscreen,
built into its own enclosure — has its own repo, including all its firmware,
docs, companion apps, and photos:
https://github.com/PhilGoode/Original-W6PS-SatRotor

73, W6PS
