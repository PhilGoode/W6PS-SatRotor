#!/usr/bin/env python3
"""
Safe standalone Skyfield experiment for SatRotor.

This file is intentionally isolated from the live bridge/UI path.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Iterable
from urllib.request import Request, urlopen


SAT_MAP = {
    "AO-91": 43017,
    "SO-50": 27607,
    "RS-44": 44909,
}

CELESTRAK_GP = "https://celestrak.org/NORAD/elements/gp.php?CATNR={norad}&FORMAT=TLE"


@dataclass
class TleLines:
    name: str
    line1: str
    line2: str


def die(msg: str, code: int = 1) -> None:
    print(msg, file=sys.stderr)
    raise SystemExit(code)


def load_skyfield():
    try:
        from skyfield.api import EarthSatellite, Loader, Topos  # type: ignore
    except Exception as exc:
        die(
            "Skyfield is not installed for this probe yet.\n"
            "Create a throwaway venv and install: pip install skyfield sgp4\n"
            f"Import error: {exc}"
        )
    return EarthSatellite, Loader, Topos


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Standalone Skyfield probe for W6PS SatRotor")
    p.add_argument("--sat", default="AO-91", help="Satellite name key (AO-91, SO-50, RS-44)")
    p.add_argument("--norad", type=int, help="Explicit NORAD catalog number")
    p.add_argument("--tle-file", type=Path, help="Optional 3-line TLE file")
    p.add_argument("--lat", type=float, required=True, help="Observer latitude in decimal degrees")
    p.add_argument("--lon", type=float, required=True, help="Observer longitude in decimal degrees")
    p.add_argument("--elev-ft", type=float, default=0.0, help="Observer elevation in feet")
    p.add_argument("--lookahead-hr", type=float, default=24.0, help="Hours to search for next pass")
    return p.parse_args()


def fetch_text(url: str) -> str:
    req = Request(url, headers={"User-Agent": "W6PS-SatRotor-Skyfield-Probe/1.0"})
    with urlopen(req, timeout=10) as resp:
        return resp.read().decode("utf-8", errors="replace")


def tle_from_lines(lines: Iterable[str]) -> TleLines:
    clean = [line.strip() for line in lines if line.strip()]
    if len(clean) < 3:
        die(f"Expected at least 3 non-empty TLE lines, got {len(clean)}")
    return TleLines(name=clean[0], line1=clean[1], line2=clean[2])


def load_tle(args: argparse.Namespace) -> TleLines:
    if args.tle_file:
        return tle_from_lines(args.tle_file.read_text().splitlines())

    norad = args.norad or SAT_MAP.get(args.sat.upper())
    if not norad:
        die(f"Unknown satellite '{args.sat}'. Use --norad or one of: {', '.join(sorted(SAT_MAP))}")
    text = fetch_text(CELESTRAK_GP.format(norad=norad))
    tle = tle_from_lines(text.splitlines())
    if not tle.name:
        tle.name = args.sat.upper()
    return tle


def fmt_dt(dt: datetime) -> str:
    return dt.astimezone().strftime("%Y-%m-%d %H:%M:%S %Z")


def main() -> int:
    args = parse_args()
    EarthSatellite, Loader, Topos = load_skyfield()

    tle = load_tle(args)
    load = Loader(str(Path.home() / ".skyfield"))
    ts = load.timescale()

    sat = EarthSatellite(tle.line1, tle.line2, tle.name, ts)
    observer = Topos(latitude_degrees=args.lat, longitude_degrees=args.lon, elevation_m=args.elev_ft * 0.3048)

    now = datetime.now(timezone.utc)
    t_now = ts.from_datetime(now)
    difference = sat - observer
    topocentric = difference.at(t_now)
    alt, az, distance = topocentric.altaz()

    print("Skyfield probe")
    print(f"satellite:   {tle.name}")
    print(f"observer:    lat={args.lat:.6f} lon={args.lon:.6f} elev_ft={args.elev_ft:.1f}")
    print(f"time:        {fmt_dt(now)}")
    print(f"altitude:    {alt.degrees:.2f} deg")
    print(f"azimuth:     {az.degrees:.2f} deg")
    print(f"range:       {distance.km:.1f} km / {distance.km * 0.621371:.1f} mi")
    print(f"in view:     {'yes' if alt.degrees > 0 else 'no'}")

    start = now
    end = now + timedelta(hours=args.lookahead_hr)
    t0 = ts.from_datetime(start)
    t1 = ts.from_datetime(end)
    times, events = sat.find_events(observer, t0, t1, altitude_degrees=0.0)

    pass_found = False
    print("")
    print("next pass search:")
    for ti, ev in zip(times, events):
        dt = ti.utc_datetime().replace(tzinfo=timezone.utc)
        label = {0: "rise", 1: "culminate", 2: "set"}.get(int(ev), f"event-{ev}")
        if not pass_found:
            pass_found = True
        print(f"  {label:10s} {fmt_dt(dt)}")
        if label == "set":
            break

    if not pass_found:
        print(f"  no rise/set events found in next {args.lookahead_hr:g} hours")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
