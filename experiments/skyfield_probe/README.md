Skyfield Probe

This is an isolated, non-live experiment.

Purpose:
- test Skyfield pass/pointing math without touching the working Pi UI
- compare Skyfield az/el output against the current SatRotor bridge behavior
- keep all risk out of the live control path

Nothing in here is imported by the current Pi bridge or Nucleo/XIAO runtime.

Typical use:

1. install dependencies in a throwaway virtualenv:

   python3 -m venv .venv
   source .venv/bin/activate
   pip install skyfield sgp4

2. run the probe:

   python3 skyfield_probe.py --sat AO-91 --lat 35.1234 --lon -117.1234 --elev-ft 2500

3. optionally use a custom TLE file:

   python3 skyfield_probe.py --tle-file ./ao91.tle --lat 35.1234 --lon -117.1234

Notes:
- Default TLE fetch uses CelesTrak's GP endpoint by NORAD number.
- This script prints current alt/az/range and the next visible pass.
- It does not command hardware.
