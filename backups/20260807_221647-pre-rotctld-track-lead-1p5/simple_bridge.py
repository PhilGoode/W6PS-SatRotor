#!/usr/bin/env python3
import base64
import fcntl
import json
import math
import os
import re
import stat
import socket
import socketserver
import subprocess
import threading
import time
import serial
from datetime import datetime, timedelta, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlencode, urlparse, unquote
from urllib.request import Request, urlopen


BASE_DIR = Path(__file__).resolve().parent
INDEX_PATH = BASE_DIR / "index-pi.html"
THREE_PATH = BASE_DIR / "three.min.js"
EARTH_TEXTURE_PATH = BASE_DIR / "earth-blue-marble-2048.png"
WORLD_MAP_PATH = BASE_DIR / "world-map.geojson"
ADMIN1_LINES_PATH = BASE_DIR / "admin1-lines.geojson"
ROTOR_ZERO_STATE_PATH = BASE_DIR / "rotor-zero-state.json"
STATION_STATE_PATH = BASE_DIR / "station-state.json"
ROTATOR_LINK_CONFIG_PATH = BASE_DIR / "rotator-link-config.json"
CIV_ROUTER_CONFIG_PATH = BASE_DIR / "civ-router-config.json"
ADIF_LOG_PATH = BASE_DIR / "qso_log.adi"
QSO_RECENT_PATH = BASE_DIR / "qso_recent.jsonl"
ROTOR_CALIBRATION_LOG_PATH = BASE_DIR / "rotor-calibration-log.jsonl"
CREDENTIALS_PATH = BASE_DIR / "upload_credentials.json"
EARTH_TEXTURE_DATA_URL = "data:image/png;base64," + base64.b64encode(EARTH_TEXTURE_PATH.read_bytes()).decode("ascii")

PORT = int(os.environ.get("PORT", "8000"))
SERIAL_BAUD = int(os.environ.get("SERIAL_BAUD", "115200"))
GPS_SERIAL_BAUD = int(os.environ.get("GPS_SERIAL_BAUD", "38400"))
ROTCTLD_ENABLE = str(os.environ.get("ROTCTLD_ENABLE", "1")).strip() != "0"
ROTCTLD_HOST = os.environ.get("ROTCTLD_HOST", "0.0.0.0")
ROTCTLD_PORT = int(os.environ.get("ROTCTLD_PORT", "4533"))
ROTOR_AZ_PHASE_OFFSET_DEG = float(os.environ.get("ROTOR_AZ_PHASE_OFFSET_DEG", "180.0"))
ROTOR_QUERY_AZ_WRAP_DEG = float(os.environ.get("ROTOR_QUERY_AZ_WRAP_DEG", "300.0"))
ROTOR_AZ_LIMIT_WARN_DEG = float(os.environ.get("ROTOR_AZ_LIMIT_WARN_DEG", "400.0"))
ROTOR_AZ_LIMIT_HARD_DEG = float(os.environ.get("ROTOR_AZ_LIMIT_HARD_DEG", "425.0"))
ROTOR_PHYSICAL_EL_MIN_DEG = float(os.environ.get("ROTOR_PHYSICAL_EL_MIN_DEG", "-30.0"))
ROTOR_PHYSICAL_EL_MAX_DEG = float(os.environ.get("ROTOR_PHYSICAL_EL_MAX_DEG", "180.0"))
ROTOR_HOME_AZ_DEADBAND_DEG = float(os.environ.get("ROTOR_HOME_AZ_DEADBAND_DEG", "0.5"))
ROTOR_HOME_EL_DEADBAND_DEG = float(os.environ.get("ROTOR_HOME_EL_DEADBAND_DEG", "0.5"))
ROTOR_HOME_FINAL_AZ_WINDOW_DEG = float(os.environ.get("ROTOR_HOME_FINAL_AZ_WINDOW_DEG", "1.5"))
ROTOR_HOME_FINAL_EL_WINDOW_DEG = float(os.environ.get("ROTOR_HOME_FINAL_EL_WINDOW_DEG", "1.5"))
ROTOR_HOME_PULSE_MS = float(os.environ.get("ROTOR_HOME_PULSE_MS", "120"))
ROTOR_PLANNED_MOVE_ENABLE = str(os.environ.get("ROTOR_PLANNED_MOVE_ENABLE", "1")).strip() != "0"
ROTOR_PLANNED_AZ_SPEED_DPS = float(os.environ.get("ROTOR_PLANNED_AZ_SPEED_DPS", "13.6"))
ROTOR_PLANNED_EL_SPEED_DPS = float(os.environ.get("ROTOR_PLANNED_EL_SPEED_DPS", "10.8"))
ROTOR_PLANNED_TIME_SAFETY_FACTOR = float(os.environ.get("ROTOR_PLANNED_TIME_SAFETY_FACTOR", "0.990"))
ROTOR_PLANNED_AZ_TOL_DEG = float(os.environ.get("ROTOR_PLANNED_AZ_TOL_DEG", "1.0"))
ROTOR_PLANNED_EL_TOL_DEG = float(os.environ.get("ROTOR_PLANNED_EL_TOL_DEG", "1.0"))
ROTOR_PLANNED_MAX_AXIS_SEC = float(os.environ.get("ROTOR_PLANNED_MAX_AXIS_SEC", "18.0"))
ROTOR_PLANNED_READBACK_DELAY_SEC = float(os.environ.get("ROTOR_PLANNED_READBACK_DELAY_SEC", "0.5"))
ROTOR_PLANNED_AZ_CORRECTION_ENABLE = str(os.environ.get("ROTOR_PLANNED_AZ_CORRECTION_ENABLE", "1")).strip() != "0"
ROTOR_PLANNED_AZ_CORRECTION_POINTS = os.environ.get(
    "ROTOR_PLANNED_AZ_CORRECTION_POINTS",
    "0:5.3,90:2.0,180:2.5,270:3.8,330:2.4,360:5.3",
)
ROTOR_READOUT_AZ_CORRECTION_ENABLE = str(os.environ.get("ROTOR_READOUT_AZ_CORRECTION_ENABLE", "1")).strip() != "0"
ROTOR_READOUT_AZ_CORRECTION_POINTS = os.environ.get(
    "ROTOR_READOUT_AZ_CORRECTION_POINTS",
    "0:0.0,2.2:-2.1,90:0.0,180:-0.6,270:-2.0,330:-3.0,360:0.0",
)
ROTOR_DISPLAY_AZ_CORRECTION_ENABLE = str(os.environ.get("ROTOR_DISPLAY_AZ_CORRECTION_ENABLE", "1")).strip() != "0"
ROTOR_DISPLAY_AZ_CORRECTION_POINTS = os.environ.get(
    "ROTOR_DISPLAY_AZ_CORRECTION_POINTS",
    "0:0.0,90:-3.0,180:-1.0,270:-4.0,330:0.0,360:0.0",
)
ROTOR_DISPLAY_HOME_SNAP_DEG = float(os.environ.get("ROTOR_DISPLAY_HOME_SNAP_DEG", "8.0"))
ROTOR_TRACK_LEAD_SECONDS = float(os.environ.get("ROTOR_TRACK_LEAD_SECONDS", "1.5"))
ROTOR_TRACK_LEAD_AZ_RATE_DEG_PER_SEC = float(os.environ.get("ROTOR_TRACK_LEAD_AZ_RATE_DEG_PER_SEC", "18.0"))
ROTOR_TRACK_LEAD_EL_RATE_DEG_PER_SEC = float(os.environ.get("ROTOR_TRACK_LEAD_EL_RATE_DEG_PER_SEC", "10.0"))
ROTOR_TRACK_MIN_CMD_AZ_DELTA_DEG = float(os.environ.get("ROTOR_TRACK_MIN_CMD_AZ_DELTA_DEG", "0.8"))
ROTOR_TRACK_MIN_CMD_EL_DELTA_DEG = float(os.environ.get("ROTOR_TRACK_MIN_CMD_EL_DELTA_DEG", "0.5"))
ROTOR_TRACK_ACQUIRE_AZ_WINDOW_DEG = float(os.environ.get("ROTOR_TRACK_ACQUIRE_AZ_WINDOW_DEG", "8.0"))
ROTOR_TRACK_ACQUIRE_EL_WINDOW_DEG = float(os.environ.get("ROTOR_TRACK_ACQUIRE_EL_WINDOW_DEG", "5.0"))
ROTOR_TRACK_ACQUIRE_MAX_HOLD_SEC = float(os.environ.get("ROTOR_TRACK_ACQUIRE_MAX_HOLD_SEC", "14.0"))
ROTOR_HOME_EL_TARGET_DEG = float(os.environ.get("ROTOR_HOME_EL_TARGET_DEG", "0.0"))
SAT_TRACK_MIN_EL_DEG = float(os.environ.get("SAT_TRACK_MIN_EL_DEG", "5.0"))
SAT_TRACK_END_EL_DEG = float(os.environ.get("SAT_TRACK_END_EL_DEG", str(SAT_TRACK_MIN_EL_DEG)))
SAT_TRACK_ZENITH_HOLD_EL_DEG = float(os.environ.get("SAT_TRACK_ZENITH_HOLD_EL_DEG", "70.0"))
TRACK_UPDATE_MS = float(os.environ.get("TRACK_UPDATE_MS", "150"))
ROTCTLD_TRACK_PROMOTE_WINDOW_SEC = float(os.environ.get("ROTCTLD_TRACK_PROMOTE_WINDOW_SEC", "3.0"))
ROTCTLD_TRACK_STALE_SEC = float(os.environ.get("ROTCTLD_TRACK_STALE_SEC", "3.5"))
ROTCTLD_TRACK_SEND_MS = float(os.environ.get("ROTCTLD_TRACK_SEND_MS", "300"))
SHTC3_I2C_BUS = os.environ.get("SHTC3_I2C_BUS", "/dev/i2c-1")
SHTC3_I2C_ADDR = int(os.environ.get("SHTC3_I2C_ADDR", "0x70"), 0)
EXTERNAL_ROTATOR_SERIAL_PATH = os.environ.get("EXTERNAL_ROTATOR_SERIAL_PATH", "").strip()
SATNOGS_API_BASE = os.environ.get("SATNOGS_API_BASE", "https://db.satnogs.org/api")
CELESTRAK_GP_URL = os.environ.get(
    "CELESTRAK_GP_URL",
    "https://celestrak.org/NORAD/elements/gp.php?CATNR={norad}&FORMAT=TLE",
)
CELESTRAK_AMATEUR_URL = os.environ.get(
    "CELESTRAK_AMATEUR_URL",
    "https://celestrak.org/NORAD/elements/gp.php?GROUP=amateur&FORMAT=tle",
)
AMSAT_GP_JSON_URL = os.environ.get(
    "AMSAT_GP_JSON_URL",
    "https://newark192.amsat.org/gpdata/current/daily-bulletin.json",
)
CELESTRAK_NOAA_URL = os.environ.get(
    "CELESTRAK_NOAA_URL",
    "https://celestrak.org/NORAD/elements/gp.php?GROUP=weather&FORMAT=tle",
)
SWPC_KP_URL = os.environ.get(
    "SWPC_KP_URL",
    "https://services.swpc.noaa.gov/products/noaa-planetary-k-index.json",
)
SWPC_XRAY_URL = os.environ.get(
    "SWPC_XRAY_URL",
    "https://services.swpc.noaa.gov/json/goes/primary/xrays-1-day.json",
)
SWPC_FLUX_URL = os.environ.get(
    "SWPC_FLUX_URL",
    "https://services.swpc.noaa.gov/products/summary/10cm-flux.json",
)
NWS_USER_AGENT = os.environ.get("NWS_USER_AGENT", "W6PS-SatRotor/1.0 (phil@w6ps.local)")
OPEN_METEO_FORECAST_URL = os.environ.get("OPEN_METEO_FORECAST_URL", "https://api.open-meteo.com/v1/forecast")
NOMINATIM_REVERSE_URL = os.environ.get("NOMINATIM_REVERSE_URL", "https://nominatim.openstreetmap.org/reverse")
NOAA_SAT_NAMES = ("NOAA 15", "NOAA 18", "NOAA 19", "METEOR-M2 3")
SATNOGS_NORAD_MAP = {
    "AO-91": 43017,
    "SO-50": 27607,
    "AO-73": 39444,
    "AO-7": 7530,
    "RS-44": 44909,
    "FO-29": 24278,
    "XW-2A": 40903,
    "XW-2B": 40911,
    "XW-2C": 40906,
    "XW-2D": 40907,
    "XW-2F": 40910,
}
SUPPORTED_SATS = tuple(SATNOGS_NORAD_MAP.keys())

ROTATOR_PROFILE_CATALOG = (
    {
        "id": "internal-head",
        "label": "SatRotor",
        "transport": "internal-ptz",
        "connector": "Internal control path",
        "baud": 115200,
        "dataBits": 8,
        "parity": "N",
        "stopBits": 1,
        "notes": "Use the internal SatRotor head and the live PTZ/query path.",
    },
    {
        "id": "antrunner-pro",
        "label": "AntRunner Pro",
        "transport": "rs232-db9",
        "connector": "Rear DB9 RS-232",
        "baud": 9600,
        "dataBits": 8,
        "parity": "N",
        "stopBits": 1,
        "notes": "External AntRunner Pro profile over the rear RS-232 port.",
    },
    {
        "id": "hamlib-rotctld",
        "label": "Hamlib rotctld",
        "transport": "tcp-rotctld",
        "connector": "Network TCP",
        "baud": None,
        "dataBits": None,
        "parity": None,
        "stopBits": None,
        "notes": "For Gpredict, MacDoppler, and other Hamlib TCP clients.",
    },
    {
        "id": "yaesu-gs232",
        "label": "Yaesu GS-232",
        "transport": "rs232-db9",
        "connector": "Rear DB9 RS-232",
        "baud": 9600,
        "dataBits": 8,
        "parity": "N",
        "stopBits": 1,
        "notes": "Primary external serial rotator profile for Yaesu/GS-232 style gear.",
    },
    {
        "id": "spid-rot1prog",
        "label": "SPID Rot1Prog",
        "transport": "rs232-db9",
        "connector": "Rear DB9 RS-232",
        "baud": 1200,
        "dataBits": 8,
        "parity": "N",
        "stopBits": 1,
        "notes": "Single-axis SPID serial profile.",
    },
    {
        "id": "spid-rot2prog",
        "label": "SPID Rot2Prog",
        "transport": "rs232-db9",
        "connector": "Rear DB9 RS-232",
        "baud": 600,
        "dataBits": 8,
        "parity": "N",
        "stopBits": 1,
        "notes": "Dual-axis SPID serial profile.",
    },
    {
        "id": "spid-md0x-yaesu",
        "label": "SPID MD-0x (Yaesu mode)",
        "transport": "rs232-db9",
        "connector": "Rear DB9 RS-232",
        "baud": 9600,
        "dataBits": 8,
        "parity": "N",
        "stopBits": 1,
        "notes": "Use when an MD-01/02/03 controller is configured to emulate Yaesu. Basic commands only.",
    },
)
ROTATOR_PROFILE_INDEX = {item["id"]: item for item in ROTATOR_PROFILE_CATALOG}


def candidate_serial_paths():
    env = os.environ.get("SERIAL_PATH", "").strip()
    if env:
      return [env]

    candidates = []
    for path in (
        "/dev/cu.usbmodem11343403",
        "/dev/ttyACM0",
        "/dev/ttyUSB0",
    ):
        if os.path.exists(path):
            candidates.append(path)

    try:
        usbmodems = sorted(Path("/dev").glob("cu.usbmodem*"), key=lambda p: p.stat().st_mtime, reverse=True)
        candidates.extend(str(p) for p in usbmodems)
    except Exception:
        pass

    seen = set()
    ordered = []
    for path in candidates:
        if path not in seen:
            seen.add(path)
            ordered.append(path)
    return ordered


SERIAL_CANDIDATES = candidate_serial_paths()
SERIAL_PATH = SERIAL_CANDIDATES[0] if SERIAL_CANDIDATES else "/dev/ttyACM0"


def resolve_gps_serial_path():
    env_path = os.environ.get("GPS_SERIAL_PATH", "").strip()
    if env_path.lower() in ("", "0", "off", "false", "none", "disabled"):
        return ""
    candidates = []
    if env_path:
        candidates.append(env_path)
    for path in candidates:
        if path and os.path.exists(path):
            return path
    return ""


GPS_SERIAL_PATH = resolve_gps_serial_path()

latest = {
    "az": 0.0,
    "azTotal": 0.0,
    "el": 0.0,
    "sats": 0,
    "gps": "NO FIX",
    "tx": 145.960,
    "rx": 435.250,
    "sat": "IDLE",
    "txMode": "FM",
    "rxMode": "FM",
    "status": "BOOT",
    "nucleoResetStatus": "--",
    "wifi": "",
    "gpsSatellites": [],
    "gpsUsed": [],
    "gpsLastNmea": "",
    "gpsLat": "--",
    "gpsLon": "--",
    "gpsAltFt": "--",
    "gpsLastFixTs": 0.0,
    "gpsAux": "--",
    "gpsPpsAux": "--",
    "gpsSignalAux": "--",
    "gpsDualBandCount": 0,
    "gpsBestL5Snr": "--",
    "gpsPpsCount": 0,
    "gpsPpsSeen": False,
    "gpsRst": "HI",
    "gpsEn": "HI",
    "gpsExtint": "HI",
    "gpsSafeboot": "HI",
    "gpsFixQuality": "--",
    "gpsPdop": "--",
    "gpsHdop": "--",
    "gpsVdop": "--",
    "gpsUtc": "--",
    "gpsDate": "--",
    "gpsSpeedKmh": "--",
    "gpsSpeedMph": "--",
    "gpsTrackDeg": "--",
    "rotorSource": "--",
    "menuTicks": 0,
    "menuPresses": 0,
}


def default_rotator_link_config():
    return {
        "selectedProfile": "internal-head",
        "serialModule": "DIUSTOU DSTR2TPA",
        "serialPortLabel": "Rear DB9 RS-232",
        "serialUart": "UART5",
        "serialTxPin": "PC12 / CN8-10 / D47",
        "serialRxPin": "PD2 / CN8-12 / D48",
        "serialPower": "3.3V logic side",
        "serialRtsCts": "Parked",
    }


def default_civ_router_config():
    return {
        "enabled": False,
        "routerSerialPath": os.environ.get("CIV_ROUTER_SERIAL_PATH", "").strip(),
        "routerLink": "ESP32 USB serial",
        "radioType": "standard-icom",
        "baud": 19200,
        "address": "A2",
        "vfoType": "main-up-sub-down",
        "updateIntervalMs": 500,
        "minFreqChangeHz": 25,
        "afterPassAction": "none",
        "afterPassValue": "",
        "afterPassMode": "FM",
        "afterPassPl": "",
        "port1Label": "IC-9700",
        "port2Label": "Icom Aux 1",
        "port3Label": "Icom Aux 2",
        "notes": "ESP32 3-port CI-V router. Pi sends CI-V frames; ESP32 owns bus routing.",
    }

WEBVIEW_HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no" />
  <title>External View</title>
  <style>
    html,body{margin:0;height:100%;background:#061224;color:#9ff;font-family:Arial,sans-serif;overflow:hidden}
    .root{height:100%;display:grid;grid-template-rows:60px 1fr}
    .bar{display:flex;align-items:center;justify-content:space-between;gap:10px;padding:8px 10px;background:rgba(4,10,22,.9);border-bottom:2px solid #1c5fff}
    .title{font-size:20px;font-weight:800;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
    .actions{display:flex;gap:8px;flex-shrink:0}
    .btn{min-height:42px;padding:8px 12px;border:2px solid #4ea3ff;border-radius:10px;background:#0b2244;color:#9ff;font:800 18px Arial;text-decoration:none;display:inline-flex;align-items:center;justify-content:center}
    iframe{width:100%;height:100%;border:0;background:#fff}
  </style>
</head>
<body>
  <div class="root">
    <div class="bar">
      <div class="title">{title}</div>
      <div class="actions">
        <a class="btn" href="/">BACK TO UI</a>
        <a class="btn" href="{target}" target="_blank" rel="noopener">OPEN DIRECT</a>
      </div>
    </div>
    <iframe src="{target}" title="{title}"></iframe>
  </div>
</body>
</html>
"""

rotor_live = {
    "az": 0.0,
    "azTotal": 0.0,
    "el": 0.0,
    "azUnwrapped": 0.0,
    "azPrevRaw": None,
    "queryAz": 0.0,
    "queryEl": 0.0,
    "queryPrevAz": None,
    "queryPrevTs": 0.0,
    "totalPrev": None,
    "fusedPrevRaw": None,
    "fusionAnchorTotalAz": None,
    "fusionAnchorAzUnwrapped": 0.0,
    "fusionUsingEstimate": False,
    "queryState": "BOOT",
    "queryValid": False,
    "queryPanOk": False,
    "queryTiltOk": False,
    "queryPanRaw": 0,
    "queryTiltRaw": 0,
}
rotor_zero = {
    "enabled": False,
    "az": 0.0,
    "el": 0.0,
    "azUnwrapped": 0.0,
    "totalAz": 0.0,
    "panRaw": None,
}
rotor_limit = {
    "travelAz": 0.0,
    "prevReferencedAz": None,
}
rotator_link_config = default_rotator_link_config()
civ_router_config = default_civ_router_config()
sat_lock = {
    "enabled": False,
    "sat": "AO-91",
    "lastGoodAz": None,
    "lastGoodEl": None,
    "lastGoodTs": 0.0,
    "commandAz": None,
    "commandEl": None,
    "commandTs": 0.0,
    "acquireActive": False,
    "acquireStartedTs": 0.0,
}

rotor_net = {
    "enabled": ROTCTLD_ENABLE,
    "targetAz": None,
    "targetEl": None,
    "active": False,
    "busy": False,
    "commandSeq": 0,
    "lastCmd": "PTZ STOP",
    "lastClient": "",
    "lastUpdateMs": 0.0,
    "lastSendMs": 0.0,
    "homePrevAzErr": None,
    "homePrevElErr": None,
    "homeWorseTicks": 0,
    "homeHoldTs": 0.0,
}
rotctld_client_streams = {}


def clear_sat_lock_last_good_locked():
    sat_lock["lastGoodAz"] = None
    sat_lock["lastGoodEl"] = None
    sat_lock["lastGoodTs"] = 0.0
    sat_lock["commandAz"] = None
    sat_lock["commandEl"] = None
    sat_lock["commandTs"] = 0.0
    sat_lock["acquireActive"] = False
    sat_lock["acquireStartedTs"] = 0.0

state_lock = threading.Lock()
serial_writer_lock = threading.Lock()
calibration_log_lock = threading.Lock()
serial_port = None
serial_file = None
last_serial_ts = 0.0
rotor_net_cancel_event = threading.Event()
external_rotator_lock = threading.Lock()
external_rotator_serial = None
external_rotator_profile_id = ""
gps_serial_port = None
gps_serial_file = None
last_gps_serial_ts = 0.0
gps_used_seen = {}
_skyfield_loader = None
_skyfield_ts = None
_skyfield_api = None
_tle_cache = {}
_tle_refresh_state = {"last_manual_refresh_ts": 0.0}
_amateur_catalog_cache = {"ts": 0.0, "items": []}
_noaa_catalog_cache = {"ts": 0.0, "items": []}
_visible_sats_cache = {"ts": 0.0, "observer": None, "rows": []}
_reverse_geocode_cache = {}
_reverse_geocode_lock = threading.Lock()
_last_reverse_geocode_ts = 0.0
_ducting_profile_cache = {}
_ducting_profile_lock = threading.Lock()
_satellite_objects = {}


def adif_escape(value):
    return "" if value is None else str(value)


def adif_length(value):
    return len(adif_escape(value).encode("utf-8"))


def append_qso_to_adif(qso):
    now = datetime.now(timezone.utc)
    fields = [
        f"<CALL:{adif_length(qso.get('callsign'))}>{adif_escape(qso.get('callsign'))}",
        f"<QSO_DATE:8>{now.strftime('%Y%m%d')}",
        f"<TIME_ON:6>{now.strftime('%H%M%S')}",
    ]

    if qso.get("freq_mhz"):
        fields.append(f"<FREQ:{adif_length(qso.get('freq_mhz'))}>{adif_escape(qso.get('freq_mhz'))}")
    if qso.get("mode"):
        fields.append(f"<MODE:{adif_length(qso.get('mode'))}>{adif_escape(qso.get('mode'))}")
    if qso.get("rst_sent"):
        fields.append(f"<RST_SENT:{adif_length(qso.get('rst_sent'))}>{adif_escape(qso.get('rst_sent'))}")
    if qso.get("rst_rcvd"):
        fields.append(f"<RST_RCVD:{adif_length(qso.get('rst_rcvd'))}>{adif_escape(qso.get('rst_rcvd'))}")
    if qso.get("sat_name"):
        fields.append(f"<SAT_NAME:{adif_length(qso.get('sat_name'))}>{adif_escape(qso.get('sat_name'))}")
    if qso.get("az") is not None and qso.get("el") is not None:
        comment = f"AZ={float(qso['az']):.1f} EL={float(qso['el']):.1f}"
        fields.append(f"<COMMENT:{adif_length(comment)}>{comment}")
    if qso.get("notes"):
        fields.append(f"<NOTES:{adif_length(qso.get('notes'))}>{adif_escape(qso.get('notes'))}")

    line = " ".join(fields) + " <EOR>\n"
    is_new_file = not ADIF_LOG_PATH.exists()
    with open(ADIF_LOG_PATH, "a", encoding="utf-8") as handle:
        if is_new_file:
            handle.write("ADIF Export from W6PS SatRotor\n")
            handle.write("<ADIF_VER:5>3.1.4\n")
            handle.write("<PROGRAMID:12>W6PS SatRotor\n")
            handle.write("<EOH>\n")
        handle.write(line)


def append_qso_recent_entry(qso):
    entry = {
        "ts": datetime.now(timezone.utc).isoformat(),
        "callsign": str(qso.get("callsign") or "").upper(),
        "freqMhz": str(qso.get("freq_mhz") or ""),
        "mode": str(qso.get("mode") or "").upper(),
        "rstSent": str(qso.get("rst_sent") or ""),
        "rstRcvd": str(qso.get("rst_rcvd") or ""),
        "az": None if qso.get("az") is None else round(float(qso.get("az")), 1),
        "el": None if qso.get("el") is None else round(float(qso.get("el")), 1),
        "satName": str(qso.get("sat_name") or ""),
        "notes": str(qso.get("notes") or ""),
    }
    with open(QSO_RECENT_PATH, "a", encoding="utf-8") as handle:
        handle.write(json.dumps(entry, ensure_ascii=False) + "\n")
    return entry


def load_recent_qsos(limit=20):
    if not QSO_RECENT_PATH.exists():
        return []
    rows = []
    with open(QSO_RECENT_PATH, "r", encoding="utf-8") as handle:
        for line in handle:
            text = line.strip()
            if not text:
                continue
            try:
                rows.append(json.loads(text))
            except Exception:
                continue
    return list(reversed(rows[-max(1, int(limit)):]))


def load_upload_credentials():
    if not CREDENTIALS_PATH.exists():
        return {}
    try:
        return json.loads(CREDENTIALS_PATH.read_text(encoding="utf-8"))
    except Exception:
        return {}


def save_upload_credentials(creds):
    CREDENTIALS_PATH.write_text(json.dumps(creds, ensure_ascii=False), encoding="utf-8")
    os.chmod(CREDENTIALS_PATH, stat.S_IRUSR | stat.S_IWUSR)


def credentials_status_summary():
    creds = load_upload_credentials()
    return {
        "qrz": bool(creds.get("qrz", {}).get("apiKey")),
        "clublog": bool(
            creds.get("clublog", {}).get("email")
            and creds.get("clublog", {}).get("password")
        ),
        "eqsl": bool(
            creds.get("eqsl", {}).get("username")
            and creds.get("eqsl", {}).get("password")
        ),
    }


def force_serial_reopen(reason):
    global serial_file, serial_port, last_serial_ts
    old_file = None
    with serial_writer_lock:
        old_file = serial_file
        serial_file = None
        serial_port = None
    last_serial_ts = 0.0
    if old_file is not None:
        try:
            old_file.close()
        except Exception:
            pass
    with state_lock:
        latest["status"] = f"SERIAL RECOVER: {reason}"


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def norm_az(value):
    az = float(value or 0.0) % 360.0
    if az < 0.0:
        az += 360.0
    if abs(az) < 0.5 or abs(az - 360.0) < 0.5:
        return 0.0
    return az


def normalize_angle_delta(value):
    delta = float(value or 0.0)
    while delta > 180.0:
        delta -= 360.0
    while delta < -180.0:
        delta += 360.0
    return delta


PAN_RAW_MODULO = 65536
PAN_RAW_DEG_SCALE = 100.0


def normalize_pan_raw(value):
    try:
        return int(round(float(value))) % PAN_RAW_MODULO
    except Exception:
        return 0


def pan_raw_to_display_az(current_raw, zero_raw):
    delta_counts = (normalize_pan_raw(current_raw) - normalize_pan_raw(zero_raw)) % PAN_RAW_MODULO
    return norm_az(delta_counts / PAN_RAW_DEG_SCALE)


def inferred_pan_raw_from_az(az_deg):
    return normalize_pan_raw(norm_az(float(az_deg or 0.0)) * PAN_RAW_DEG_SCALE)


def current_zero_pan_raw_locked():
    pan_raw = rotor_zero.get("panRaw")
    if pan_raw is None:
        return inferred_pan_raw_from_az(rotor_zero.get("az", 0.0))
    return normalize_pan_raw(pan_raw)


def raw_referenced_pan_az_locked():
    if not bool(rotor_zero.get("enabled")):
        return None
    if not bool(rotor_live.get("queryValid", False)) or not bool(rotor_live.get("queryPanOk", False)):
        return None
    return pan_raw_to_display_az(rotor_live.get("queryPanRaw", 0), current_zero_pan_raw_locked())


def referenced_pan_az_locked():
    raw_az = raw_referenced_pan_az_locked()
    if raw_az is None:
        return None
    return corrected_readout_az(raw_az)[0]


def parse_az_correction_points(spec):
    points = []
    for chunk in str(spec or "").split(","):
        part = chunk.strip()
        if not part or ":" not in part:
            continue
        az_text, corr_text = part.split(":", 1)
        try:
            az = float(az_text.strip())
            corr = float(corr_text.strip())
        except Exception:
            continue
        points.append((az, corr))
    if not points:
        points = [(0.0, 0.0), (360.0, 0.0)]
    points.sort(key=lambda row: row[0])
    if points[0][0] > 0.0:
        points.insert(0, (0.0, points[-1][1]))
    if points[-1][0] < 360.0:
        points.append((360.0, points[0][1]))
    return points


AZ_CORRECTION_POINTS = parse_az_correction_points(ROTOR_PLANNED_AZ_CORRECTION_POINTS)
READOUT_AZ_CORRECTION_POINTS = parse_az_correction_points(ROTOR_READOUT_AZ_CORRECTION_POINTS)
DISPLAY_AZ_CORRECTION_POINTS = parse_az_correction_points(ROTOR_DISPLAY_AZ_CORRECTION_POINTS)


def planned_az_correction(target_az):
    if not ROTOR_PLANNED_AZ_CORRECTION_ENABLE:
        return 0.0
    return interpolated_az_correction(target_az, AZ_CORRECTION_POINTS)


def interpolated_az_correction(target_az, points):
    az = norm_az(float(target_az))
    if az == 0.0 and float(target_az or 0.0) >= 359.5:
        az = 360.0
    prev_az, prev_corr = points[0]
    for next_az, next_corr in points[1:]:
        if az <= next_az:
            span = max(0.001, float(next_az) - float(prev_az))
            t = (az - float(prev_az)) / span
            return float(prev_corr) + (float(next_corr) - float(prev_corr)) * t
        prev_az, prev_corr = next_az, next_corr
    return float(points[-1][1])


def corrected_planned_target_az(target_az):
    correction = planned_az_correction(target_az)
    return norm_az(float(target_az) + correction), correction


def readout_az_correction(display_az):
    if not ROTOR_READOUT_AZ_CORRECTION_ENABLE:
        return 0.0
    return interpolated_az_correction(display_az, READOUT_AZ_CORRECTION_POINTS)


def corrected_readout_az(display_az):
    correction = readout_az_correction(display_az)
    return norm_az(float(display_az) + correction), correction


def display_az_correction(referenced_az):
    if not ROTOR_DISPLAY_AZ_CORRECTION_ENABLE:
        return 0.0
    return interpolated_az_correction(referenced_az, DISPLAY_AZ_CORRECTION_POINTS)


def corrected_display_az(referenced_az):
    correction = display_az_correction(referenced_az)
    return norm_az(float(referenced_az) + correction), correction


def advance_az_toward(current_az, target_az, max_delta_deg):
    delta = normalize_angle_delta(float(target_az) - float(current_az))
    if abs(delta) <= max_delta_deg:
        return norm_az(target_az)
    return norm_az(float(current_az) + (max_delta_deg if delta > 0.0 else -max_delta_deg))


def advance_linear_toward(current_value, target_value, max_delta):
    delta = float(target_value) - float(current_value)
    if abs(delta) <= max_delta:
        return float(target_value)
    return float(current_value) + (max_delta if delta > 0.0 else -max_delta)


def unwrap_angle_near_hint(raw_az, hint):
    raw_az = norm_az(raw_az)
    hint = float(hint or 0.0)
    base_turn = math.floor(hint / 360.0)
    candidates = [raw_az + 360.0 * (base_turn + offset) for offset in (-1, 0, 1)]
    return min(candidates, key=lambda candidate: abs(candidate - hint))


def reset_limit_tracker_locked(current_az=0.0):
    rotor_limit["travelAz"] = 0.0
    rotor_limit["prevReferencedAz"] = norm_az(float(current_az or 0.0))


def update_limit_tracker_locked(current_az):
    current_az = norm_az(float(current_az or 0.0))
    prev = rotor_limit.get("prevReferencedAz")
    if prev is None:
        rotor_limit["prevReferencedAz"] = current_az
        return
    delta = normalize_angle_delta(current_az - float(prev))
    rotor_limit["travelAz"] = float(rotor_limit.get("travelAz", 0.0)) + delta
    rotor_limit["prevReferencedAz"] = current_az


def referenced_rotor_locked():
    if rotor_zero["enabled"] and not bool(rotor_live.get("queryValid", False)):
        return 0.0, 0.0
    raw_query_az = norm_az(float(rotor_live.get("queryAz", rotor_live.get("az", latest.get("az", 0.0)))))
    raw_el = float(rotor_live.get("queryEl", rotor_live.get("el", latest.get("el", 0.0))))
    if rotor_zero["enabled"]:
        pan_az = referenced_pan_az_locked()
        if pan_az is not None:
            return pan_az, float(rotor_zero["el"]) - raw_el
        zero_az = norm_az(float(rotor_zero.get("az", 0.0)))
        return norm_az(raw_query_az - zero_az), float(rotor_zero["el"]) - raw_el
    if active_rotator_is_internal():
        return norm_az(raw_query_az - ROTOR_AZ_PHASE_OFFSET_DEG), raw_el
    return raw_query_az, raw_el


def reported_rotor_locked():
    reference_az, reference_el = referenced_rotor_locked()
    display_az, _ = corrected_display_az(reference_az)
    display_el = reference_el
    can_snap_home = rotor_zero["enabled"] and str(rotor_live.get("queryState", "")) == "FREEZE"
    if can_snap_home and abs(normalize_angle_delta(reference_az)) <= ROTOR_DISPLAY_HOME_SNAP_DEG:
        display_az = 0.0
    if can_snap_home and abs(display_el) <= 1.0:
        display_el = 0.0
    return display_az, display_el


def current_limit_relative_az_locked():
    return 0.0


def update_unwrapped_az_locked(raw_az):
    raw_az = norm_az(raw_az)
    prev_raw = rotor_live.get("fusedPrevRaw")
    if prev_raw is None:
        rotor_live["fusedPrevRaw"] = raw_az
        rotor_live["azUnwrapped"] = raw_az
        return
    delta = normalize_angle_delta(raw_az - float(prev_raw))
    rotor_live["azUnwrapped"] = float(rotor_live.get("azUnwrapped", float(prev_raw))) + delta
    rotor_live["fusedPrevRaw"] = raw_az


def refresh_fused_az_locked():
    query_az = norm_az(float(rotor_live.get("queryAz", 0.0)))
    query_prev = rotor_live.get("queryPrevAz")
    prev_unwrapped = rotor_live.get("azUnwrapped")

    query_delta = None
    if query_prev is not None:
        query_delta = normalize_angle_delta(query_az - float(query_prev))

    query_frozen = query_delta is not None and abs(query_delta) <= 0.2
    unwrap_hint = float(prev_unwrapped) if prev_unwrapped is not None else query_az
    fused_unwrapped = unwrap_angle_near_hint(query_az, unwrap_hint)
    rotor_live["azUnwrapped"] = fused_unwrapped
    rotor_live["fusedPrevRaw"] = query_az
    rotor_live["az"] = query_az
    rotor_live["fusionUsingEstimate"] = False
    rotor_live["queryState"] = "FREEZE" if query_frozen else "QUERY"

    rotor_live["queryPrevAz"] = query_az
    rotor_live["queryPrevTs"] = time.time()
    rotor_live["totalPrev"] = float(rotor_live.get("azTotal", 0.0))


def save_rotor_zero_locked():
    payload = {
        "enabled": bool(rotor_zero.get("enabled")),
        "az": float(rotor_zero.get("az", 0.0)),
        "el": float(rotor_zero.get("el", 0.0)),
        "azUnwrapped": float(rotor_zero.get("azUnwrapped", 0.0)),
        "totalAz": float(rotor_zero.get("totalAz", 0.0)),
        "panRaw": None if rotor_zero.get("panRaw") is None else int(normalize_pan_raw(rotor_zero.get("panRaw"))),
        "savedAt": datetime.now(timezone.utc).isoformat(),
    }
    tmp_path = ROTOR_ZERO_STATE_PATH.with_suffix(".json.tmp")
    tmp_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    tmp_path.replace(ROTOR_ZERO_STATE_PATH)


def load_rotor_zero_state():
    payload = None
    try:
        payload = json.loads(ROTOR_ZERO_STATE_PATH.read_text(encoding="utf-8"))
    except FileNotFoundError:
        payload = None
    except Exception:
        payload = None
    with state_lock:
        if isinstance(payload, dict) and bool(payload.get("enabled")):
            rotor_zero["enabled"] = True
            rotor_zero["az"] = norm_az(float(payload.get("az", 0.0)))
            rotor_zero["el"] = float(payload.get("el", 0.0))
            rotor_zero["azUnwrapped"] = rotor_zero["az"]
            rotor_zero["totalAz"] = float(payload.get("totalAz", 0.0))
            rotor_zero["panRaw"] = (
                normalize_pan_raw(payload.get("panRaw"))
                if payload.get("panRaw") is not None
                else inferred_pan_raw_from_az(rotor_zero["az"])
            )
            latest["az"] = 0.0
            latest["azTotal"] = 0.0
            latest["el"] = 0.0
            latest["rotorSource"] = "ZERO"
            latest["status"] = "ROTOR ZERO RESTORED"
        else:
            rotor_zero["enabled"] = False
            rotor_zero["az"] = 0.0
            rotor_zero["el"] = 0.0
            rotor_zero["azUnwrapped"] = 0.0
            rotor_zero["totalAz"] = 0.0
            rotor_zero["panRaw"] = None
            latest["az"] = 0.0
            latest["azTotal"] = 0.0
            latest["el"] = 0.0
            latest["rotorSource"] = "BOOT"
            latest["status"] = "ROTOR ZERO FRESH BOOT"
        rotor_limit["prevReferencedAz"] = None
        rotor_limit["travelAz"] = 0.0
    return False


def apply_encoder_overlay_locked():
    display_az, display_el = reported_rotor_locked()
    rotor_limit["prevReferencedAz"] = None
    rotor_limit["travelAz"] = 0.0
    latest["az"] = round(display_az, 1)
    latest["azTotal"] = round(display_az, 1)
    latest["el"] = round(display_el, 1)
    latest["rotorSource"] = "ZERO" if rotor_zero["enabled"] else "LIVE"


def home_az_error_from_reference(value):
    current = float(value or 0.0)
    choices = (current, current - 360.0, current + 360.0)
    return min(choices, key=lambda candidate: abs(candidate))


def parse_nmea_coord(raw, hemi, is_lat):
    hemi = str(hemi or "").strip().upper()
    if hemi not in ("N", "S", "E", "W"):
        return None
    if is_lat and hemi not in ("N", "S"):
        return None
    if not is_lat and hemi not in ("E", "W"):
        return None
    try:
        v = float(raw or 0.0)
    except ValueError:
        return None
    if not v:
        return None
    deg = int(v // 100)
    mins = v - (deg * 100)
    if mins < 0.0 or mins >= 60.0:
        return None
    dec = deg + (mins / 60.0)
    if is_lat and not (-90.0 <= dec <= 90.0):
        return None
    if not is_lat and not (-180.0 <= dec <= 180.0):
        return None
    if hemi in ("S", "W"):
        dec = -dec
    return dec


def gps_coord_jump_ok(lat, lon):
    try:
        prev_lat = float(latest.get("gpsLat", "--"))
        prev_lon = float(latest.get("gpsLon", "--"))
    except (TypeError, ValueError):
        return True
    if abs(prev_lat) < 0.00001 and abs(prev_lon) < 0.00001:
        return True
    if abs(prev_lon) < 1.0 and abs(float(lon)) > 1.0:
        return True
    return abs(float(lat) - prev_lat) <= 0.05 and abs(float(lon) - prev_lon) <= 0.05


def sys_from_talker(talker=""):
    if talker.startswith("GP"):
        return "GPS"
    if talker.startswith("GL"):
        return "GLONASS"
    if talker.startswith("GA"):
        return "GAL"
    if talker.startswith("GQ"):
        return "QZSS"
    if talker.startswith("GI"):
        return "NAVIC"
    if talker.startswith("GB") or talker.startswith("BD"):
        return "BDS"
    if talker.startswith("GN"):
        return "MIX"
    return talker or "UNK"


def normalize_system_for_prn(system, prn_value):
    try:
        prn = int(prn_value or 0)
    except ValueError:
        prn = 0

    if 120 <= prn <= 158:
        return "SBAS"
    if 65 <= prn <= 96:
        return "GLONASS"
    if system == "GLONASS":
        return "GLONASS"
    if system == "GPS":
        return "GPS"
    return system


def allowed_system(system):
    return system in ("GPS", "GLONASS", "SBAS", "BDS", "GAL", "QZSS", "NAVIC")


def gps_fix_quality_label(value):
    try:
        q = int(value or 0)
    except ValueError:
        q = 0
    return {
        0: "NO FIX",
        1: "GNSS FIX",
        2: "DGPS",
        4: "RTK FIX",
        5: "RTK FLOAT",
        6: "DR",
    }.get(q, f"FIX {q}")


def parse_rmc_datetime(time_field, date_field):
    tf = str(time_field or "").strip()
    df = str(date_field or "").strip()
    if len(tf) < 6 or len(df) != 6 or not tf[:6].isdigit() or not df.isdigit():
        return None, None
    hh, mm, ss = tf[:2], tf[2:4], tf[4:6]
    day, month, year = df[:2], df[2:4], df[4:6]
    year_full = 2000 + int(year)
    return f"{hh}:{mm}:{ss} UTC", f"{year_full:04d}-{month}-{day}"


def parse_gsv(line):
    head = line[1:6]
    talker = head[:2]
    system = sys_from_talker(talker)
    parts = line.split(",")
    if len(parts) < 4:
        return

    try:
        msg_num = int(parts[2] or "0")
    except ValueError:
        msg_num = 0

    with state_lock:
        if msg_num == 1:
            latest["gpsSatellites"] = [s for s in latest["gpsSatellites"] if s.get("system") != system]

        for i in range(4, len(parts), 4):
            if i + 3 >= len(parts):
                break
            prn = (parts[i] or "").strip()
            if not prn or not prn.isdigit():
                continue

            try:
                el = int(parts[i + 1]) if parts[i + 1] != "" else 0
            except ValueError:
                el = 0
            try:
                az = int(parts[i + 2]) if parts[i + 2] != "" else 0
            except ValueError:
                az = 0

            snr_raw = (parts[i + 3] or "").split("*")[0].strip()
            try:
                snr = int(snr_raw) if snr_raw != "" else None
            except ValueError:
                snr = None

            normalized = normalize_system_for_prn(system, prn)
            if not allowed_system(normalized):
                continue

            key = f"{normalized}-{prn}"
            used = prn in latest["gpsUsed"]
            row = {
                "key": key,
                "prn": prn,
                "system": normalized,
                "el": el,
                "az": az,
                "snr": snr,
                "used": used,
            }

            idx = next((j for j, s in enumerate(latest["gpsSatellites"]) if s.get("key") == key), -1)
            if idx >= 0:
                if latest["gpsSatellites"][idx].get("signals"):
                    row["signals"] = latest["gpsSatellites"][idx]["signals"]
                latest["gpsSatellites"][idx] = row
            else:
                latest["gpsSatellites"].append(row)

        latest["gpsSatellites"] = sorted(
            latest["gpsSatellites"][-64:],
            key=lambda s: (s.get("snr") is not None, s.get("snr") or -1),
            reverse=True,
        )
        update_gps_signal_summary_locked()


def parse_gsa(line):
    parts = line.split(",")
    if len(parts) < 3:
        return
    try:
        fix_type = int(parts[2] or "1")
    except ValueError:
        fix_type = 1

    now = time.time()
    for i in range(3, min(15, len(parts))):
        p = (parts[i] or "").strip()
        if p and p.isdigit():
            gps_used_seen[p] = now

    ttl = 5.0
    stale = [prn for prn, ts in gps_used_seen.items() if (now - ts) > ttl]
    for prn in stale:
        gps_used_seen.pop(prn, None)

    pdop = "--"
    hdop = "--"
    vdop = "--"
    dop_fields = parts[-3:] if len(parts) >= 3 else []
    if len(dop_fields) == 3:
        try:
            pdop = f"{float((dop_fields[0] or '').split('*')[0]):.2f}" if dop_fields[0] else "--"
        except ValueError:
            pdop = "--"
        try:
            hdop = f"{float((dop_fields[1] or '').split('*')[0]):.2f}" if dop_fields[1] else "--"
        except ValueError:
            hdop = "--"
        try:
            vdop = f"{float((dop_fields[2] or '').split('*')[0]):.2f}" if dop_fields[2] else "--"
        except ValueError:
            vdop = "--"

    with state_lock:
        latest["gps"] = "3D" if fix_type >= 3 else ("2D" if fix_type == 2 else "NO FIX")
        latest["gpsUsed"] = sorted(gps_used_seen.keys(), key=lambda x: int(x))
        latest["gpsPdop"] = pdop
        latest["gpsHdop"] = hdop
        latest["gpsVdop"] = vdop
        if latest["gpsSatellites"]:
            for sat in latest["gpsSatellites"]:
                sat["used"] = sat.get("prn") in latest["gpsUsed"]


def host_wifi_summary():
    ssid = ""
    ip = ""
    mode = "HOST"

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
    except OSError:
        ip = ""

    if os.uname().sysname == "Darwin":
        for dev in ("en0", "en1"):
            try:
                out = subprocess.check_output(
                    ["networksetup", "-getairportnetwork", dev],
                    stderr=subprocess.DEVNULL,
                    text=True,
                    timeout=1.5,
                ).strip()
                if "Current Wi-Fi Network:" in out:
                    ssid = out.split("Current Wi-Fi Network:", 1)[1].strip()
                    break
            except Exception:
                continue
    else:
        try:
            ssid = subprocess.check_output(
                ["iwgetid", "-r"], stderr=subprocess.DEVNULL, text=True, timeout=1.5
            ).strip()
        except Exception:
            ssid = ""

    if ssid and ip:
        return f"{mode} {ssid} {ip}"
    if ip:
        return f"{mode} LINK {ip}"
    return ""


def satnogs_norad_for_name(name):
    text = str(name or "").upper()
    for key, norad in SATNOGS_NORAD_MAP.items():
        if key in text:
            return norad
    return None


def satellite_name_key(name):
    text = str(name or "").upper()
    text = re.sub(r"\bAO-0+(\d+)\b", r"AO-\1", text)
    text = re.sub(r"\bFO-0+(\d+)\b", r"FO-\1", text)
    text = re.sub(r"[^A-Z0-9]+", "", text)
    return text


def normalize_supported_sat(name, fallback="AO-91"):
    text = str(name or "").strip().upper()
    for key in SUPPORTED_SATS:
        if key in text:
            return key
    return fallback


def find_amateur_catalog_item(name):
    text = str(name or "").strip().upper()
    if not text or text == "IDLE":
        return None
    text_key = satellite_name_key(text)
    for item in amateur_catalog():
        item_name = str(item.get("name") or "").upper()
        if item_name == text or satellite_name_key(item_name) == text_key:
            return item
    for item in amateur_catalog():
        item_name = str(item.get("name") or "").upper()
        item_key = satellite_name_key(item_name)
        if text in item_name or item_name in text or text_key in item_key:
            return item
    mapped = normalize_supported_sat(text, "")
    if mapped:
        want_norad = SATNOGS_NORAD_MAP.get(mapped)
        for item in amateur_catalog():
            if item["norad"] == want_norad:
                return item
    return None


def satnogs_fetch_json(path):
    req = Request(f"{SATNOGS_API_BASE}{path}", headers={"User-Agent": "W6PS-SatRotor/1.0"})
    with urlopen(req, timeout=8) as resp:
        return json.loads(resp.read().decode("utf-8"))


def satnogs_current_payload(sat_name):
    norad = satnogs_norad_for_name(sat_name)
    if not norad:
        return {
            "ok": False,
            "error": "unsupported_satellite",
            "sat": sat_name,
        }

    satellite_rows = satnogs_fetch_json(f"/satellites/?norad_cat_id={norad}&format=json")
    transmitter_rows = satnogs_fetch_json(f"/transmitters/?satellite__norad_cat_id={norad}&format=json")

    satellite = satellite_rows[0] if satellite_rows else {}
    transmitters = []
    for row in transmitter_rows:
        tx = {
            "description": row.get("description") or row.get("type") or "Transmitter",
            "mode": row.get("mode") or "--",
            "uplink_low": row.get("uplink_low"),
            "uplink_high": row.get("uplink_high"),
            "downlink_low": row.get("downlink_low"),
            "downlink_high": row.get("downlink_high"),
            "status": row.get("status") or "--",
            "alive": bool(row.get("alive")),
            "service": row.get("service") or "--",
            "uuid": row.get("uuid") or "",
        }
        transmitters.append(tx)

    return {
        "ok": True,
        "satellite": {
            "name": satellite.get("name") or sat_name,
            "names": satellite.get("names") or "",
            "sat_id": satellite.get("sat_id") or "",
            "norad_cat_id": satellite.get("norad_cat_id") or norad,
            "status": satellite.get("status") or "--",
            "launched": satellite.get("launched") or "",
            "operator": satellite.get("operator") or "",
            "countries": satellite.get("countries") or "",
            "website": satellite.get("website") or "",
            "updated": satellite.get("updated") or "",
            "citation": satellite.get("citation") or "",
        },
        "transmitters": transmitters,
    }


def _load_skyfield():
    global _skyfield_loader, _skyfield_ts, _skyfield_api
    if _skyfield_api is not None:
        return _skyfield_api

    try:
        from skyfield.api import EarthSatellite, Loader, wgs84
    except Exception as exc:
        raise RuntimeError(f"skyfield_unavailable: {exc}") from exc

    cache_dir = Path.home() / ".skyfield"
    _skyfield_loader = Loader(str(cache_dir))
    _skyfield_ts = _skyfield_loader.timescale()
    _skyfield_api = (EarthSatellite, wgs84)
    return _skyfield_api


def _tle_age_label(cache_key):
    cached = _tle_cache.get(str(cache_key))
    if not cached:
        return "--"
    dt = datetime.fromtimestamp(float(cached["ts"]), tz=timezone.utc).astimezone()
    return f"UPDATED {dt.strftime('%H:%M:%S %Z')}"


def _amateur_catalog_age_label():
    ts = float(_amateur_catalog_cache.get("ts") or 0.0)
    if not ts:
        return "--"
    dt = datetime.fromtimestamp(ts, tz=timezone.utc).astimezone()
    return f"UPDATED {dt.strftime('%H:%M:%S %Z')}"


def _tle_lines_for_sat(sat_name, norad, force_refresh=False):
    cache_key = str(norad)
    now = time.time()
    cached = _tle_cache.get(cache_key)
    if cached and not force_refresh and (now - cached["ts"]) < 6 * 3600:
        return cached["name"], cached["line1"], cached["line2"]

    text = fetch_text = None
    req = Request(
        CELESTRAK_GP_URL.format(norad=norad),
        headers={"User-Agent": "W6PS-SatRotor-Skyfield/1.0"},
    )
    with urlopen(req, timeout=10) as resp:
        text = resp.read().decode("utf-8", errors="replace")

    lines = [line.strip() for line in text.splitlines() if line.strip()]
    if len(lines) < 3:
        raise RuntimeError(f"bad_tle_for_{sat_name}")

    name, line1, line2 = lines[0], lines[1], lines[2]
    _tle_cache[cache_key] = {
        "ts": now,
        "name": name,
        "line1": line1,
        "line2": line2,
    }
    return name, line1, line2


def refresh_tles(force_refresh=False, sat_names=None):
    amateur_catalog(force_refresh=force_refresh)
    names = [normalize_supported_sat(name) for name in (sat_names or SUPPORTED_SATS)]
    refreshed = []
    for sat_name in names:
        norad = satnogs_norad_for_name(sat_name)
        if not norad:
            continue
        _tle_lines_for_sat(sat_name, norad, force_refresh=force_refresh)
        refreshed.append(sat_name)
    _tle_refresh_state["last_manual_refresh_ts"] = time.time()
    return refreshed


def _parse_tle_triplets(text):
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    out = []
    i = 0
    while i + 2 < len(lines):
        name, line1, line2 = lines[i], lines[i + 1], lines[i + 2]
        if line1.startswith("1 ") and line2.startswith("2 "):
            out.append((name, line1, line2))
            i += 3
        else:
            i += 1
    return out


def _norad_from_tle_line(line1):
    match = re.match(r"1\s+(\d+)", str(line1 or ""))
    return int(match.group(1)) if match else None


def _amsat_display_name(record):
    amsat_name = str(record.get("AMSAT_NAME") or "").strip()
    object_name = str(record.get("OBJECT_NAME") or "").strip()
    if amsat_name:
        return amsat_name
    return object_name or f"SAT-{record.get('NORAD_CAT_ID', '')}".strip("-")


def _satellite_from_catalog_item(item):
    EarthSatellite, _ = _load_skyfield()
    if item.get("omm"):
        sat = EarthSatellite.from_omm(_skyfield_ts, item["omm"])
        try:
            sat.name = item.get("name") or sat.name
        except Exception:
            pass
        return sat
    return EarthSatellite(item["line1"], item["line2"], item["name"], _skyfield_ts)


def amateur_catalog(force_refresh=False):
    now = time.time()
    cached = _amateur_catalog_cache
    if cached["items"] and not force_refresh and (now - cached["ts"]) < 6 * 3600:
        return cached["items"]

    items = []
    try:
        req = Request(
            AMSAT_GP_JSON_URL,
            headers={"User-Agent": "W6PS-SatRotor-Skyfield/1.0"},
        )
        with urlopen(req, timeout=15) as resp:
            records = json.loads(resp.read().decode("utf-8", errors="replace"))
        if isinstance(records, list):
            for record in records:
                if not isinstance(record, dict):
                    continue
                try:
                    norad = int(record.get("NORAD_CAT_ID"))
                except (TypeError, ValueError):
                    continue
                display_name = _amsat_display_name(record)
                items.append(
                    {
                        "name": display_name,
                        "norad": norad,
                        "omm": dict(record),
                        "source": "AMSAT GP",
                    }
                )
    except Exception:
        items = []

    if not items:
        req = Request(
            CELESTRAK_AMATEUR_URL,
            headers={"User-Agent": "W6PS-SatRotor-Skyfield/1.0"},
        )
        with urlopen(req, timeout=15) as resp:
            text = resp.read().decode("utf-8", errors="replace")

        for name, line1, line2 in _parse_tle_triplets(text):
            norad_match = re.match(r"1\s+(\d+)", line1)
            norad = int(norad_match.group(1)) if norad_match else None
            if not norad:
                continue
            display_name = (name or "").strip() or f"SAT-{norad}"
            items.append(
                {
                    "name": display_name,
                    "norad": norad,
                    "line1": line1,
                    "line2": line2,
                    "source": "CelesTrak",
                }
            )

    _amateur_catalog_cache["ts"] = now
    _amateur_catalog_cache["items"] = items
    _satellite_objects.clear()
    return items


def _fetch_json_url(url, headers=None, timeout=8):
    req = Request(url, headers=headers or {"User-Agent": "W6PS-SatRotor/1.0"})
    with urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8", errors="replace"))


def _latest_numeric_from_table(rows, candidates=(1, 2, -1)):
    if not isinstance(rows, list):
        return None
    for row in reversed(rows):
        if not isinstance(row, list):
            continue
        for idx in candidates:
            try:
                value = row[idx]
            except Exception:
                continue
            try:
                return float(value)
            except Exception:
                continue
    return None


def space_weather_summary():
    kp_resp = _fetch_json_url(SWPC_KP_URL)
    xray_resp = _fetch_json_url(SWPC_XRAY_URL)
    flux_resp = _fetch_json_url(SWPC_FLUX_URL)

    kp_value = None
    if isinstance(kp_resp, list):
        for row in reversed(kp_resp):
            if isinstance(row, dict):
                try:
                    kp_value = float(row.get("Kp"))
                    break
                except Exception:
                    continue
    flux_value = None
    if isinstance(flux_resp, list):
        for row in reversed(flux_resp):
            if isinstance(row, dict):
                try:
                    flux_value = float(row.get("flux"))
                    break
                except Exception:
                    continue
            elif isinstance(row, list):
                flux_value = _latest_numeric_from_table(flux_resp, (1, 2, -1))
                break

    xray_flux = None
    if isinstance(xray_resp, list):
        for row in reversed(xray_resp):
            if not isinstance(row, dict):
                continue
            try:
                xray_flux = float(row.get("flux"))
                break
            except Exception:
                continue

    storm_level = "--"
    if kp_value is not None:
        storm_level = "Active" if kp_value >= 5.0 else "Quiet"

    return {
        "ok": True,
        "kpIndex": None if kp_value is None else round(kp_value, 1),
        "solarFluxIndex": None if flux_value is None else round(flux_value, 1),
        "xrayFlux": None if xray_flux is None else f"{xray_flux:.3e}",
        "stormLevel": storm_level,
        "updatedAt": datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC"),
    }


def weather_summary():
    observer = current_observer_from_state()
    if observer is None:
        return {"ok": False, "error": "observer_gps_unavailable"}

    _obs, lat, lon, _elev_ft = observer
    return weather_summary_for_location(lat, lon)


def _wind_dir_from_degrees(deg):
    try:
        value = float(deg)
    except (TypeError, ValueError):
        return None
    labels = ("N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW")
    return labels[int((value + 11.25) / 22.5) % 16]


def _open_meteo_condition(code):
    try:
        code = int(code)
    except (TypeError, ValueError):
        return "Unknown"
    return {
        0: "Clear sky",
        1: "Mainly clear",
        2: "Partly cloudy",
        3: "Overcast",
        45: "Fog",
        48: "Depositing rime fog",
        51: "Light drizzle",
        53: "Moderate drizzle",
        55: "Dense drizzle",
        56: "Light freezing drizzle",
        57: "Dense freezing drizzle",
        61: "Slight rain",
        63: "Moderate rain",
        65: "Heavy rain",
        66: "Light freezing rain",
        67: "Heavy freezing rain",
        71: "Slight snow",
        73: "Moderate snow",
        75: "Heavy snow",
        77: "Snow grains",
        80: "Slight rain showers",
        81: "Moderate rain showers",
        82: "Violent rain showers",
        85: "Slight snow showers",
        86: "Heavy snow showers",
        95: "Thunderstorm",
        96: "Thunderstorm with slight hail",
        99: "Thunderstorm with heavy hail",
    }.get(code, f"Weather code {code}")


def _weather_summary_open_meteo(lat, lon):
    params = {
        "latitude": f"{float(lat):.5f}",
        "longitude": f"{float(lon):.5f}",
        "current": ",".join(
            (
                "temperature_2m",
                "relative_humidity_2m",
                "weather_code",
                "wind_speed_10m",
                "wind_direction_10m",
            )
        ),
        "temperature_unit": "fahrenheit",
        "wind_speed_unit": "mph",
        "precipitation_unit": "inch",
        "timezone": "auto",
    }
    data = _fetch_json_url(f"{OPEN_METEO_FORECAST_URL}?{urlencode(params)}")
    current = data.get("current") or {}
    wind_speed = current.get("wind_speed_10m")
    wind_dir = _wind_dir_from_degrees(current.get("wind_direction_10m"))
    temp = current.get("temperature_2m")
    humidity = current.get("relative_humidity_2m")

    return {
        "ok": True,
        "source": "Open-Meteo",
        "conditions": _open_meteo_condition(current.get("weather_code")),
        "tempF": None if temp is None else round(float(temp)),
        "windMph": None if wind_speed is None else round(float(wind_speed)),
        "windDir": wind_dir,
        "humidityPct": humidity,
        "lat": lat,
        "lon": lon,
    }


def _weather_summary_nws(lat, lon):
    headers = {"User-Agent": NWS_USER_AGENT}
    points_url = f"https://api.weather.gov/points/{lat},{lon}"
    points_data = _fetch_json_url(points_url, headers=headers)
    forecast_url = points_data["properties"]["forecastHourly"]
    forecast_data = _fetch_json_url(forecast_url, headers=headers)
    current = forecast_data["properties"]["periods"][0]

    wind_speed = None
    wind_text = str(current.get("windSpeed") or "")
    m = re.search(r"(\d+)", wind_text)
    if m:
        wind_speed = int(m.group(1))

    return {
        "ok": True,
        "source": "NWS",
        "conditions": current.get("shortForecast"),
        "tempF": current.get("temperature"),
        "windMph": wind_speed,
        "windDir": current.get("windDirection"),
        "humidityPct": current.get("relativeHumidity", {}).get("value"),
        "lat": lat,
        "lon": lon,
    }


def _format_reverse_place(payload):
    addr = payload.get("address") if isinstance(payload, dict) else {}
    if not isinstance(addr, dict):
        addr = {}
    locality = (
        addr.get("city")
        or addr.get("town")
        or addr.get("village")
        or addr.get("municipality")
        or addr.get("hamlet")
        or addr.get("county")
    )
    state = addr.get("state") or addr.get("region")
    country = addr.get("country")
    parts = []
    for part in (locality, state, country):
        if part and part not in parts:
            parts.append(str(part))
    if parts:
        return ", ".join(parts)
    display = str(payload.get("display_name") or "").strip()
    if display:
        return ", ".join([p.strip() for p in display.split(",")[:3] if p.strip()])
    return None


def reverse_geocode_location(lat, lon):
    global _last_reverse_geocode_ts
    try:
        lat_f = float(lat)
        lon_f = float(lon)
    except (TypeError, ValueError):
        return None
    key = (round(lat_f, 2), round(lon_f, 2))
    with _reverse_geocode_lock:
        cached = _reverse_geocode_cache.get(key)
        if cached and (time.time() - cached["ts"]) < 24 * 3600:
            return cached.get("placeName")
        wait_s = 1.05 - (time.time() - _last_reverse_geocode_ts)
    if wait_s > 0:
        time.sleep(wait_s)
    params = {
        "format": "jsonv2",
        "lat": f"{lat_f:.5f}",
        "lon": f"{lon_f:.5f}",
        "zoom": "10",
        "addressdetails": "1",
        "accept-language": "en",
    }
    try:
        payload = _fetch_json_url(
            f"{NOMINATIM_REVERSE_URL}?{urlencode(params)}",
            headers={"User-Agent": NWS_USER_AGENT, "Accept-Language": "en"},
            timeout=6,
        )
        place_name = _format_reverse_place(payload)
    except Exception:
        place_name = None
    with _reverse_geocode_lock:
        _last_reverse_geocode_ts = time.time()
        _reverse_geocode_cache[key] = {"ts": time.time(), "placeName": place_name}
    return place_name


def weather_summary_for_location(lat, lon, include_place=False):
    try:
        payload = _weather_summary_nws(lat, lon)
    except Exception:
        payload = _weather_summary_open_meteo(lat, lon)
    if include_place:
        payload["placeName"] = reverse_geocode_location(lat, lon)
    return payload


def _saturation_vapor_pressure_hpa(temp_c):
    return 6.112 * math.exp((17.67 * float(temp_c)) / (float(temp_c) + 243.5))


def _refractivity_n_units(pressure_hpa, temp_c, relative_humidity_pct):
    temp_k = float(temp_c) + 273.15
    e_hpa = max(0.0, min(100.0, float(relative_humidity_pct))) / 100.0 * _saturation_vapor_pressure_hpa(temp_c)
    return 77.6 * (float(pressure_hpa) / temp_k) + 3.73e5 * (e_hpa / (temp_k * temp_k))


def _ducting_class_from_gradient(gradient_m_per_km):
    if gradient_m_per_km <= 0:
        return "DUCTING"
    if gradient_m_per_km < 79:
        return "SUPER-REFRACTION"
    if gradient_m_per_km < 118:
        return "ENHANCED"
    return "NORMAL"


def tropospheric_ducting_for_location(lat, lon):
    try:
        lat_f = float(lat)
        lon_f = float(lon)
    except (TypeError, ValueError):
        return {"ok": False, "error": "bad_coordinates"}

    key = (round(lat_f, 2), round(lon_f, 2))
    with _ducting_profile_lock:
        cached = _ducting_profile_cache.get(key)
        if cached and (time.time() - cached["ts"]) < 15 * 60:
            return dict(cached["payload"])

    levels = (1000, 925, 850)
    variables = []
    for level in levels:
        variables.extend(
            (
                f"temperature_{level}hPa",
                f"relative_humidity_{level}hPa",
                f"geopotential_height_{level}hPa",
            )
        )
    params = {
        "latitude": f"{lat_f:.5f}",
        "longitude": f"{lon_f:.5f}",
        "hourly": ",".join(variables),
        "forecast_days": "1",
        "temperature_unit": "celsius",
        "timezone": "UTC",
    }
    data = _fetch_json_url(f"{OPEN_METEO_FORECAST_URL}?{urlencode(params)}")
    hourly = data.get("hourly") or {}
    times = hourly.get("time") or []
    if not times:
        return {"ok": False, "error": "no_profile_data"}

    now = datetime.now(timezone.utc)
    best_idx = 0
    best_delta = float("inf")
    for idx, iso in enumerate(times):
        try:
            t = datetime.fromisoformat(str(iso).replace("Z", "+00:00")).replace(tzinfo=timezone.utc)
        except Exception:
            continue
        delta = abs((t - now).total_seconds())
        if delta < best_delta:
            best_idx = idx
            best_delta = delta

    profile = []
    for level in levels:
        try:
            temp_c = hourly[f"temperature_{level}hPa"][best_idx]
            rh = hourly[f"relative_humidity_{level}hPa"][best_idx]
            height_m = hourly[f"geopotential_height_{level}hPa"][best_idx]
            n = _refractivity_n_units(level, temp_c, rh)
            m = n + 0.157 * float(height_m)
        except Exception:
            continue
        profile.append(
            {
                "levelHpa": level,
                "heightM": round(float(height_m), 1),
                "tempC": round(float(temp_c), 1),
                "rhPct": round(float(rh), 1),
                "nUnits": round(n, 1),
                "mUnits": round(m, 1),
            }
        )

    gradients = []
    for a, b in zip(profile, profile[1:]):
        dz_km = (float(b["heightM"]) - float(a["heightM"])) / 1000.0
        if dz_km <= 0:
            continue
        grad = (float(b["mUnits"]) - float(a["mUnits"])) / dz_km
        gradients.append(
            {
                "fromHpa": a["levelHpa"],
                "toHpa": b["levelHpa"],
                "gradientMPerKm": round(grad, 1),
                "class": _ducting_class_from_gradient(grad),
            }
        )
    if not gradients:
        return {"ok": False, "error": "insufficient_profile_data"}

    best = min(gradients, key=lambda row: row["gradientMPerKm"])
    payload = {
        "ok": True,
        "source": "Open-Meteo pressure profile",
        "lat": lat_f,
        "lon": lon_f,
        "profileTime": times[best_idx],
        "profile": profile,
        "gradients": gradients,
        "bestGradientMPerKm": best["gradientMPerKm"],
        "ductingClass": best["class"],
    }
    with _ducting_profile_lock:
        _ducting_profile_cache[key] = {"ts": time.time(), "payload": dict(payload)}
    return payload


def noaa_catalog(force_refresh=False):
    now = time.time()
    cached = _noaa_catalog_cache
    if cached["items"] and not force_refresh and (now - cached["ts"]) < 6 * 3600:
        return cached["items"]

    req = Request(
        CELESTRAK_NOAA_URL,
        headers={"User-Agent": "W6PS-SatRotor-Skyfield/1.0"},
    )
    with urlopen(req, timeout=15) as resp:
        text = resp.read().decode("utf-8", errors="replace")

    items = []
    for name, line1, line2 in _parse_tle_triplets(text):
        display_name = (name or "").strip()
        upper_name = display_name.upper()
        if not any(tag in upper_name for tag in ("NOAA", "METEOR", "METOP", "GOES", "HIMAWARI", "FENGYUN", "DMSP")):
            continue
        items.append({"name": display_name, "line1": line1, "line2": line2})

    _noaa_catalog_cache["ts"] = now
    _noaa_catalog_cache["items"] = items
    return items


def noaa_current_rows():
    observer_info = current_observer_from_state()
    if observer_info is None:
        return []

    observer, _lat, _lon, _elev_ft = observer_info
    EarthSatellite, _wgs84 = _load_skyfield()
    t_now = _skyfield_ts.from_datetime(datetime.now(timezone.utc))
    rows = []
    for item in noaa_catalog():
        try:
            sat = EarthSatellite(item["line1"], item["line2"], item["name"], _skyfield_ts)
            alt, az, distance = (sat - observer).at(t_now).altaz()
            rows.append(
                {
                    "name": item["name"],
                    "az": round(float(az.degrees), 1),
                    "el": round(float(alt.degrees), 1),
                    "rangeKm": round(float(distance.km), 1),
                }
            )
        except Exception:
            continue
    rows.sort(key=lambda row: row["el"], reverse=True)
    return rows[:12]


def get_satellite_object(sat_name):
    selected_item = find_amateur_catalog_item(sat_name)
    if not selected_item:
        return None

    name = selected_item["name"]
    cached = _satellite_objects.get(name)
    if cached is not None:
        return cached

    sat = _satellite_from_catalog_item(selected_item)
    _satellite_objects[name] = sat
    return sat


def current_observer_from_state():
    with state_lock:
        gps_lat = latest.get("gpsLat", "--")
        gps_lon = latest.get("gpsLon", "--")
        gps_alt_ft = latest.get("gpsAltFt", "--")
        gps_last_fix_ts = float(latest.get("gpsLastFixTs", 0.0))

    lat = None
    lon = None
    elev_ft = 0.0
    try:
        if gps_lat != "--" and gps_lon != "--" and gps_last_fix_ts > 0.0 and (time.time() - gps_last_fix_ts) <= 60.0:
            lat = float(gps_lat)
            lon = float(gps_lon)
            elev_ft = float(gps_alt_ft) if gps_alt_ft != "--" else 0.0
    except (TypeError, ValueError):
        lat = None
        lon = None

    if lat is None or lon is None:
        saved = saved_station_state()
        if saved is None:
            return None
        lat, lon, elev_ft = saved

    _, wgs84 = _load_skyfield()
    observer = wgs84.latlon(latitude_degrees=lat, longitude_degrees=lon, elevation_m=elev_ft * 0.3048)
    return observer, lat, lon, elev_ft


def saved_station_state():
    try:
        data = json.loads(STATION_STATE_PATH.read_text())
        lat = float(data.get("lat"))
        lon = float(data.get("lon"))
        elev_ft = float(data.get("elevFt", 0.0) or 0.0)
        if not (math.isfinite(lat) and math.isfinite(lon) and math.isfinite(elev_ft)):
            return None
        return lat, lon, elev_ft
    except Exception:
        return None


def compute_current_az_el(sat_name, lead_seconds=0.0):
    sat = get_satellite_object(sat_name)
    observer_info = current_observer_from_state()
    if sat is None or observer_info is None:
        return None

    observer, _lat, _lon, _elev_ft = observer_info
    target_time = datetime.now(timezone.utc) + timedelta(seconds=lead_seconds)
    t_now = _skyfield_ts.from_datetime(target_time)
    topocentric = (sat - observer).at(t_now)
    alt, az, distance = topocentric.altaz()
    return float(az.degrees), float(alt.degrees), float(distance.km)


def compute_range_rate_km_s(sat_name):
    sat = get_satellite_object(sat_name)
    observer_info = current_observer_from_state()
    if sat is None or observer_info is None:
        return None

    observer, _lat, _lon, _elev_ft = observer_info
    try:
        t_now = _skyfield_ts.from_datetime(datetime.now(timezone.utc))
        topocentric = (sat - observer).at(t_now)
        pos_km = topocentric.position.km
        vel_km_s = topocentric.velocity.km_per_s
        range_km = float((pos_km[0] ** 2 + pos_km[1] ** 2 + pos_km[2] ** 2) ** 0.5)
        if range_km < 1.0:
            return 0.0
        return float(
            (pos_km[0] * vel_km_s[0] + pos_km[1] * vel_km_s[1] + pos_km[2] * vel_km_s[2])
            / range_km
        )
    except Exception:
        return None


def visible_amateur_sat_rows(lat, lon, elev_ft, force_refresh=False):
    cache_key = (round(float(lat), 4), round(float(lon), 4), round(float(elev_ft), 0))
    now = time.time()
    if (
        _visible_sats_cache["rows"]
        and not force_refresh
        and _visible_sats_cache["observer"] == cache_key
        and (now - _visible_sats_cache["ts"]) < 5.0
    ):
        return list(_visible_sats_cache["rows"])

    EarthSatellite, wgs84 = _load_skyfield()
    observer = wgs84.latlon(latitude_degrees=float(lat), longitude_degrees=float(lon), elevation_m=float(elev_ft) * 0.3048)
    t_now = _skyfield_ts.from_datetime(datetime.now(timezone.utc))
    rows = []

    for item in amateur_catalog():
        try:
            sat = _satellite_from_catalog_item(item)
            topocentric = (sat - observer).at(t_now)
            alt, az, distance = topocentric.altaz()
            el = float(alt.degrees)
            az_deg = float(az.degrees)
            range_km = float(distance.km)
            if not (math.isfinite(el) and math.isfinite(az_deg) and math.isfinite(range_km)):
                continue
            if el < SAT_TRACK_MIN_EL_DEG:
                continue
            rows.append(
                {
                    "name": item["name"],
                    "sat": item["name"],
                    "az": round(az_deg, 2),
                    "el": round(el, 2),
                    "rangeKm": round(range_km, 1),
                    "status": "SKYFIELD IN VIEW",
                }
            )
        except Exception:
            continue

    rows.sort(key=lambda row: float(row.get("el") if row.get("el") is not None else -9999), reverse=True)
    _visible_sats_cache["ts"] = now
    _visible_sats_cache["observer"] = cache_key
    _visible_sats_cache["rows"] = rows
    return list(rows)


def clear_visible_sat_cache():
    _visible_sats_cache["ts"] = 0.0
    _visible_sats_cache["observer"] = None
    _visible_sats_cache["rows"] = []


def _fmt_local(dt):
    return dt.astimezone().strftime("%Y-%m-%d %H:%M:%S %Z")


def skyfield_current_payload(sat_name):
    requested_sat = str(sat_name or "AO-91").strip()
    catalog_item = find_amateur_catalog_item(requested_sat)
    if catalog_item:
        sat_name = catalog_item["name"]
        norad = catalog_item["norad"]
    else:
        sat_name = normalize_supported_sat(requested_sat)
        norad = satnogs_norad_for_name(sat_name)
    if not norad:
        return {
            "ok": False,
            "error": "unsupported_satellite",
            "sat": requested_sat,
        }

    observer_info = current_observer_from_state()
    if observer_info is None:
        return {
            "ok": False,
            "error": "observer_gps_unavailable",
            "sat": sat_name,
        }

    EarthSatellite, wgs84 = _load_skyfield()
    if catalog_item:
        tle_name = sat_name
        sat = _satellite_from_catalog_item(catalog_item)
    else:
        tle_name, line1, line2 = _tle_lines_for_sat(sat_name, norad)
        sat = EarthSatellite(line1, line2, tle_name, _skyfield_ts)

    observer, lat, lon, elev_ft = observer_info

    now = datetime.now(timezone.utc)
    t_now = _skyfield_ts.from_datetime(now)
    difference = sat - observer
    topocentric = difference.at(t_now)
    alt, az, distance = topocentric.altaz()
    geocentric = sat.at(t_now)
    subpoint = wgs84.subpoint_of(geocentric)

    lookahead_end = now + timedelta(hours=24)
    t0 = _skyfield_ts.from_datetime(now)
    t1 = _skyfield_ts.from_datetime(lookahead_end)
    pass_events = []
    try:
        times, events = sat.find_events(observer, t0, t1, altitude_degrees=0.0)
        for ti, ev in zip(times, events):
            dt = ti.utc_datetime().replace(tzinfo=timezone.utc)
            label = {0: "rise", 1: "culminate", 2: "set"}.get(int(ev), f"event-{ev}")
            pass_events.append({"event": label, "utc": dt.isoformat(), "local": _fmt_local(dt)})
            if label == "set":
                break
    except Exception:
        pass

    orbit_path = []
    try:
        mean_motion_rad_per_min = float(getattr(getattr(sat, "model", None), "no_kozai", 0.0) or 0.0)
        period_minutes = (2.0 * 3.141592653589793 / mean_motion_rad_per_min) if mean_motion_rad_per_min > 0.0 else 0.0
        if not period_minutes or period_minutes < 60.0 or period_minutes > 1000.0:
            period_minutes = 96.0
        sample_count = 96
        start_dt = now - timedelta(minutes=period_minutes / 2.0)
        step_minutes = period_minutes / sample_count
        for idx in range(sample_count + 1):
            sample_dt = start_dt + timedelta(minutes=step_minutes * idx)
            st = _skyfield_ts.from_datetime(sample_dt)
            sp = wgs84.subpoint_of(sat.at(st))
            orbit_path.append(
                {
                    "lat": float(sp.latitude.degrees),
                    "lon": float(sp.longitude.degrees),
                    "ts": sample_dt.isoformat(),
                }
            )
    except Exception:
        orbit_path = []

    return {
        "ok": True,
        "data": {
            "selectedSat": sat_name,
            "trackedSat": sat_name,
            "tleName": tle_name,
            "norad": norad,
            "timestamp": int(time.time() * 1000),
            "observerLat": lat,
            "observerLon": lon,
            "observerAltFt": elev_ft,
            "selectedAz": round(float(az.degrees), 2),
            "selectedEl": round(float(alt.degrees), 2),
            "selectedRangeKm": round(float(distance.km), 1),
            "selectedRangeMi": round(float(distance.km) * 0.621371, 1),
            "selectedLat": round(float(subpoint.latitude.degrees), 5),
            "selectedLon": round(float(subpoint.longitude.degrees), 5),
            "selectedAltKm": round(float(subpoint.elevation.km), 3),
            "inView": bool(alt.degrees > 0.0),
            "nextPass": pass_events,
            "orbitPath": orbit_path,
            "tleAge": _amateur_catalog_age_label() if catalog_item else _tle_age_label(norad),
            "telemetry": f"SKYFIELD {'IN VIEW' if alt.degrees > 0.0 else 'BELOW HORIZON'}",
        },
    }


def active_rotator_profile(config=None):
    selected = str((config or rotator_link_config).get("selectedProfile", "internal-head"))
    return ROTATOR_PROFILE_INDEX.get(selected, ROTATOR_PROFILE_INDEX["internal-head"])


def active_rotator_profile_id(config=None):
    return str((config or rotator_link_config).get("selectedProfile", "internal-head") or "internal-head")


def active_rotator_is_internal():
    return active_rotator_profile_id() == "internal-head"


def current_rotator_link_payload():
    profile = active_rotator_profile()
    payload = dict(rotator_link_config)
    payload.update(
        {
            "ok": True,
            "profiles": list(ROTATOR_PROFILE_CATALOG),
            "activeProfile": dict(profile),
            "externalSerialPath": EXTERNAL_ROTATOR_SERIAL_PATH or "",
        }
    )
    return payload


def current_civ_router_payload():
    payload = dict(civ_router_config)
    payload.update(
        {
            "ok": True,
            "radioTypes": [
                {"id": "standard-icom", "label": "Standard Icom"},
                {"id": "icom-820-821-910", "label": "Icom 820H/821H/910H"},
                {"id": "icom-7100", "label": "Icom 7100"},
            ],
            "vfoTypes": [
                {"id": "main-up-sub-down", "label": "Main Up / Sub Down"},
                {"id": "main-down-sub-up", "label": "Main Down / Sub Up"},
                {"id": "single-vfo", "label": "Single VFO"},
            ],
            "afterPassActions": [
                {"id": "none", "label": "Do nothing"},
                {"id": "restore-vfo", "label": "Restore VFO"},
                {"id": "set-frequency", "label": "Set frequency"},
                {"id": "memory-channel", "label": "Memory channel"},
            ],
        }
    )
    return payload


def save_rotator_link_config_locked():
    payload = dict(rotator_link_config)
    payload["savedAt"] = datetime.now(timezone.utc).isoformat()
    ROTATOR_LINK_CONFIG_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def save_civ_router_config_locked():
    payload = dict(civ_router_config)
    payload["savedAt"] = datetime.now(timezone.utc).isoformat()
    CIV_ROUTER_CONFIG_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def load_rotator_link_config():
    global rotator_link_config
    rotator_link_config = default_rotator_link_config()
    try:
        if not ROTATOR_LINK_CONFIG_PATH.exists():
            return
        payload = json.loads(ROTATOR_LINK_CONFIG_PATH.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            return
        merged = default_rotator_link_config()
        merged.update({k: v for k, v in payload.items() if k in merged})
        if merged.get("selectedProfile") not in ROTATOR_PROFILE_INDEX:
            merged["selectedProfile"] = "internal-head"
        rotator_link_config = merged
    except Exception:
        rotator_link_config = default_rotator_link_config()


def load_civ_router_config():
    global civ_router_config
    civ_router_config = default_civ_router_config()
    try:
        if not CIV_ROUTER_CONFIG_PATH.exists():
            return
        payload = json.loads(CIV_ROUTER_CONFIG_PATH.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            return
        merged = default_civ_router_config()
        merged.update({k: v for k, v in payload.items() if k in merged})
        merged["enabled"] = bool(merged.get("enabled"))
        merged["baud"] = int(merged.get("baud") or 19200)
        merged["updateIntervalMs"] = int(merged.get("updateIntervalMs") or 500)
        merged["minFreqChangeHz"] = int(merged.get("minFreqChangeHz") or 25)
        merged["address"] = re.sub(r"[^0-9A-Fa-f]", "", str(merged.get("address", "A2")))[:2].upper() or "A2"
        civ_router_config = merged
    except Exception:
        civ_router_config = default_civ_router_config()


_CIV_CONTROLLER_ADDR = 0xE0
_SPEED_OF_LIGHT_KM_S = 299792.458
_civ_serial = None
_civ_serial_lock = threading.Lock()
_civ_last_tx_hz = 0
_civ_last_rx_hz = 0
_civ_last_send_ts = 0.0
_civ_sat_freqs = {}


def _civ_freq_to_bcd(freq_hz):
    digits = f"{int(freq_hz):010d}"
    out = []
    for i in range(0, 10, 2):
        lo = int(digits[i + 1])
        hi = int(digits[i])
        out.append((hi << 4) | lo)
    return bytes(reversed(out))


def _build_civ_freq_frame(radio_addr, freq_hz):
    return bytes([0xFE, 0xFE, radio_addr, _CIV_CONTROLLER_ADDR, 0x05]) + _civ_freq_to_bcd(freq_hz) + bytes([0xFD])


def _build_civ_mode_frame(radio_addr, mode_byte, filter_byte=0x01):
    return bytes([0xFE, 0xFE, radio_addr, _CIV_CONTROLLER_ADDR, 0x06, mode_byte, filter_byte, 0xFD])


def _civ_mode_byte(mode_str):
    mode = (mode_str or "FM").upper().strip()
    return {
        "LSB": 0x00,
        "USB": 0x01,
        "AM": 0x02,
        "CW": 0x03,
        "FM": 0x05,
        "NFM": 0x05,
        "FMN": 0x05,
        "CWR": 0x07,
        "DV": 0x17,
        "DD": 0x18,
    }.get(mode, 0x05)


def _civ_open_serial():
    global _civ_serial
    with _civ_serial_lock:
        path = str(civ_router_config.get("routerSerialPath") or "").strip()
        baud = int(civ_router_config.get("baud") or 19200)
        if not path:
            return None
        if _civ_serial is not None:
            try:
                if _civ_serial.is_open and _civ_serial.port == path and _civ_serial.baudrate == baud:
                    return _civ_serial
            except Exception:
                pass
            try:
                _civ_serial.close()
            except Exception:
                pass
            _civ_serial = None
        try:
            _civ_serial = serial.Serial(path, baudrate=baud, timeout=0.1)
            with state_lock:
                latest["status"] = f"CI-V OPEN {path} {baud}"
            return _civ_serial
        except Exception as exc:
            with state_lock:
                latest["status"] = f"CI-V OPEN FAIL {exc}"
            return None


def _civ_write_frame(frame):
    global _civ_serial
    ser = _civ_open_serial()
    if ser is None:
        return False
    try:
        with _civ_serial_lock:
            ser.write(frame)
            ser.flush()
        return True
    except Exception as exc:
        with _civ_serial_lock:
            try:
                _civ_serial.close()
            except Exception:
                pass
            _civ_serial = None
        with state_lock:
            latest["status"] = f"CI-V WRITE FAIL {exc}"
        return False


def _compute_doppler_hz(nominal_hz, range_rate_km_s):
    return int(round(float(nominal_hz) + (-float(nominal_hz) * float(range_rate_km_s) / _SPEED_OF_LIGHT_KM_S)))


def _fetch_civ_sat_freqs(sat_name):
    cached = _civ_sat_freqs.get(sat_name)
    if cached:
        return cached

    try:
        catalog_item = find_amateur_catalog_item(sat_name)
        norad = catalog_item["norad"] if catalog_item else satnogs_norad_for_name(sat_name)
        if not norad:
            return {}
        rows = satnogs_fetch_json(f"/transmitters/?satellite__norad_cat_id={norad}&format=json")
        for row in (rows or []):
            if str(row.get("status") or "").lower() != "active":
                continue
            downlink = row.get("downlink_low")
            uplink = row.get("uplink_low")
            mode = str(row.get("mode") or "FM").upper()
            if not downlink:
                continue
            result = {
                "downlink_hz": int(downlink),
                "uplink_hz": int(uplink) if uplink else int(downlink),
                "mode": mode,
            }
            _civ_sat_freqs[sat_name] = result
            return result
    except Exception:
        pass
    return {}


def send_civ_doppler(sat_name, range_rate_km_s):
    global _civ_last_tx_hz, _civ_last_rx_hz, _civ_last_send_ts

    if not civ_router_config.get("enabled"):
        return False

    try:
        radio_addr = int(str(civ_router_config.get("address") or "A2"), 16)
    except ValueError:
        radio_addr = 0xA2

    update_interval_s = float(civ_router_config.get("updateIntervalMs") or 500) / 1000.0
    min_change_hz = int(civ_router_config.get("minFreqChangeHz") or 25)
    now = time.time()
    if now - _civ_last_send_ts < update_interval_s:
        return False

    freqs = _fetch_civ_sat_freqs(sat_name)
    if freqs:
        downlink_nominal = freqs["downlink_hz"]
        uplink_nominal = freqs["uplink_hz"]
        mode_str = freqs.get("mode", "FM")
    else:
        with state_lock:
            tx_mhz = float(latest.get("tx") or 0.0)
            rx_mhz = float(latest.get("rx") or 0.0)
            mode_str = str(latest.get("txMode") or "FM")
        if not tx_mhz and not rx_mhz:
            return False
        uplink_nominal = int(tx_mhz * 1e6)
        downlink_nominal = int(rx_mhz * 1e6)

    downlink_corrected = _compute_doppler_hz(downlink_nominal, range_rate_km_s)
    uplink_corrected = _compute_doppler_hz(uplink_nominal, range_rate_km_s)
    downlink_changed = abs(downlink_corrected - _civ_last_rx_hz) >= min_change_hz
    uplink_changed = abs(uplink_corrected - _civ_last_tx_hz) >= min_change_hz
    if not downlink_changed and not uplink_changed and _civ_last_send_ts > 0:
        return False

    mode_byte = _civ_mode_byte(mode_str)
    vfo_type = str(civ_router_config.get("vfoType") or "main-up-sub-down")
    ok = True

    if vfo_type == "main-down-sub-up":
        if downlink_changed:
            ok &= _civ_write_frame(bytes([0xFE, 0xFE, radio_addr, _CIV_CONTROLLER_ADDR, 0x07, 0x00, 0xFD]))
            ok &= _civ_write_frame(_build_civ_freq_frame(radio_addr, downlink_corrected))
            ok &= _civ_write_frame(_build_civ_mode_frame(radio_addr, mode_byte))
        if uplink_changed:
            ok &= _civ_write_frame(bytes([0xFE, 0xFE, radio_addr, _CIV_CONTROLLER_ADDR, 0x07, 0x01, 0xFD]))
            ok &= _civ_write_frame(_build_civ_freq_frame(radio_addr, uplink_corrected))
            ok &= _civ_write_frame(_build_civ_mode_frame(radio_addr, mode_byte))
    else:
        if uplink_changed:
            ok &= _civ_write_frame(bytes([0xFE, 0xFE, radio_addr, _CIV_CONTROLLER_ADDR, 0x07, 0x00, 0xFD]))
            ok &= _civ_write_frame(_build_civ_freq_frame(radio_addr, uplink_corrected))
            ok &= _civ_write_frame(_build_civ_mode_frame(radio_addr, mode_byte))
        if downlink_changed:
            ok &= _civ_write_frame(bytes([0xFE, 0xFE, radio_addr, _CIV_CONTROLLER_ADDR, 0x07, 0x01, 0xFD]))
            ok &= _civ_write_frame(_build_civ_freq_frame(radio_addr, downlink_corrected))
            ok &= _civ_write_frame(_build_civ_mode_frame(radio_addr, mode_byte))

    if ok:
        _civ_last_tx_hz = uplink_corrected
        _civ_last_rx_hz = downlink_corrected
        _civ_last_send_ts = now
        with state_lock:
            latest["tx"] = round(uplink_corrected / 1e6, 6)
            latest["rx"] = round(downlink_corrected / 1e6, 6)
            latest["txMode"] = mode_str
            latest["rxMode"] = mode_str
            latest["status"] = (
                f"CI-V UP={uplink_corrected / 1e6:.4f} "
                f"DN={downlink_corrected / 1e6:.4f} {mode_str} "
                f"dV={float(range_rate_km_s):+.2f}km/s"
            )
    return ok


def send_civ_after_pass():
    if not civ_router_config.get("enabled"):
        return False
    action = str(civ_router_config.get("afterPassAction") or "none")
    if action == "none":
        return True

    try:
        radio_addr = int(str(civ_router_config.get("address") or "A2"), 16)
    except ValueError:
        radio_addr = 0xA2

    if action == "set-frequency":
        try:
            freq_mhz = float(str(civ_router_config.get("afterPassValue") or "0").replace(",", "."))
            freq_hz = int(freq_mhz * 1e6)
            if freq_hz <= 0:
                return False
            mode_str = str(civ_router_config.get("afterPassMode") or "FM")
            ok = _civ_write_frame(_build_civ_freq_frame(radio_addr, freq_hz))
            ok &= _civ_write_frame(_build_civ_mode_frame(radio_addr, _civ_mode_byte(mode_str)))
            with state_lock:
                latest["status"] = f"CI-V PARK {freq_mhz:.4f} {mode_str}"
            return ok
        except Exception as exc:
            with state_lock:
                latest["status"] = f"CI-V PARK FAIL {exc}"
            return False

    return False


def exit_kiosk_to_desktop():
    def worker():
        env = dict(os.environ)
        env.setdefault("DISPLAY", ":0")
        env.setdefault("XAUTHORITY", "/home/phil/.Xauthority")
        env.setdefault("XDG_RUNTIME_DIR", "/run/user/1000")
        env.setdefault("DBUS_SESSION_BUS_ADDRESS", "unix:path=/run/user/1000/bus")

        def run_as_phil(cmd, wait=True):
            full_cmd = ["/usr/sbin/runuser", "-u", "phil", "--"] + cmd
            try:
                proc = subprocess.Popen(
                    full_cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    env=env,
                )
                if wait:
                    proc.wait(timeout=5)
                return proc
            except Exception:
                return None

        if Path("/home/phil/bin/satrotor-desktop-mode.sh").exists():
            run_as_phil(["/home/phil/bin/satrotor-desktop-mode.sh"])
        else:
            run_as_phil(["systemctl", "--user", "stop", "satrotor-kiosk.service"])
            run_as_phil(["systemctl", "--user", "reset-failed", "satrotor-kiosk.service"])
            try:
                subprocess.Popen(
                    ["pkill", "-u", "phil", "-f", "chromium.*chromium-satrotor-kiosk"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception:
                pass
            time.sleep(0.5)
            run_as_phil(["/home/phil/bin/satrotor-restore-desktop.sh"])

    threading.Thread(target=worker, daemon=True).start()


def launch_external_site(url):
    if not str(url).startswith("https://"):
        return False

    def worker():
        try:
            runtime_dir = f"/run/user/{os.getuid()}"
            env = dict(os.environ)
            env.setdefault("DISPLAY", ":0")
            env.setdefault("XDG_RUNTIME_DIR", runtime_dir)
            env.setdefault("DBUS_SESSION_BUS_ADDRESS", f"unix:path={runtime_dir}/bus")

            candidates = [
                ["/usr/bin/chromium", "--new-window", "--start-maximized", str(url)],
                ["/usr/bin/chromium-browser", "--new-window", "--start-maximized", str(url)],
                ["xdg-open", str(url)],
            ]

            for cmd in candidates:
                try:
                    subprocess.Popen(
                        cmd,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        env=env,
                    )
                    return
                except FileNotFoundError:
                    continue
        except Exception:
            pass

    threading.Thread(target=worker, daemon=True).start()
    return True


def request_pi_shutdown():
    # Fire and return immediately. Waiting on systemd here makes the UI feel hung
    # because the web server disappears while the HTTP request is still open.
    candidates = (
        ("sudo", "-n", "systemctl", "poweroff"),
        ("systemctl", "poweroff"),
        ("sudo", "-n", "/usr/sbin/shutdown", "-h", "now"),
    )

    def worker():
        time.sleep(0.15)
        env = os.environ.copy()
        env["PATH"] = "/usr/sbin:/usr/bin:/sbin:/bin:" + env.get("PATH", "")
        for cmd in candidates:
            try:
                subprocess.Popen(
                    cmd,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    env=env,
                    start_new_session=True,
                )
                return
            except FileNotFoundError:
                continue
            except Exception:
                continue

    with state_lock:
        latest["status"] = "PI SHUTDOWN REQUESTED"
    threading.Thread(target=worker, daemon=True).start()
    return True, "shutdown_requested"


def request_pi_reboot():
    candidates = (
        ("sudo", "-n", "systemctl", "reboot"),
        ("systemctl", "reboot"),
        ("sudo", "-n", "/usr/sbin/shutdown", "-r", "now"),
    )

    def worker():
        time.sleep(0.15)
        env = os.environ.copy()
        env["PATH"] = "/usr/sbin:/usr/bin:/sbin:/bin:" + env.get("PATH", "")
        for cmd in candidates:
            try:
                subprocess.Popen(
                    cmd,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    env=env,
                    start_new_session=True,
                )
                return
            except FileNotFoundError:
                continue
            except Exception:
                continue

    with state_lock:
        latest["status"] = "PI REBOOT REQUESTED"
    threading.Thread(target=worker, daemon=True).start()
    return True, "reboot_requested"


def _read_first_float(path, divisor=1.0):
    try:
        return float(Path(path).read_text().strip()) / divisor
    except Exception:
        return None


def _format_uptime(seconds):
    seconds = max(0, int(seconds))
    days, rem = divmod(seconds, 86400)
    hours, rem = divmod(rem, 3600)
    minutes, _ = divmod(rem, 60)
    if days:
        return f"{days}d {hours}h {minutes}m"
    if hours:
        return f"{hours}h {minutes}m"
    return f"{minutes}m"


def _vcgencmd_throttled():
    candidates = ("/usr/bin/vcgencmd", "/bin/vcgencmd", "vcgencmd")
    for cmd in candidates:
        try:
            proc = subprocess.run(
                [cmd, "get_throttled"],
                check=False,
                capture_output=True,
                text=True,
                timeout=1.5,
            )
        except FileNotFoundError:
            continue
        except Exception:
            continue
        out = (proc.stdout or "").strip()
        if proc.returncode == 0 and out:
            return out
    return "--"


def _decode_throttle(raw):
    if not raw or raw == "--":
        return "vcgencmd unavailable"
    match = re.search(r"0x([0-9a-fA-F]+)", raw)
    if not match:
        return raw
    value = int(match.group(1), 16)
    if value == 0:
        return "OK"
    labels = []
    flags = (
        (0, "under-voltage now"),
        (1, "freq capped now"),
        (2, "throttled now"),
        (3, "soft temp limit now"),
        (16, "under-voltage seen"),
        (17, "freq capped seen"),
        (18, "throttled seen"),
        (19, "soft temp limit seen"),
    )
    for bit, label in flags:
        if value & (1 << bit):
            labels.append(label)
    return ", ".join(labels) if labels else raw


def _sensirion_crc8(two_bytes):
    crc = 0xFF
    for byte in two_bytes:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def _read_shtc3_environment():
    # SparkFun SHTC3 breakout, default I2C address 0x70.
    i2c_slave = 0x0703
    try:
        with open(SHTC3_I2C_BUS, "r+b", buffering=0) as bus:
            fcntl.ioctl(bus, i2c_slave, SHTC3_I2C_ADDR)
            bus.write(bytes((0x35, 0x17)))  # wake
            time.sleep(0.002)
            bus.write(bytes((0x78, 0x66)))  # measure, temp first, no clock stretching
            time.sleep(0.015)
            data = bus.read(6)
            try:
                bus.write(bytes((0xB0, 0x98)))  # sleep
            except Exception:
                pass
    except Exception as exc:
        return {"ok": False, "error": str(exc)}

    if len(data) != 6:
        return {"ok": False, "error": "short_read"}
    temp_raw_bytes = data[0:2]
    rh_raw_bytes = data[3:5]
    if _sensirion_crc8(temp_raw_bytes) != data[2] or _sensirion_crc8(rh_raw_bytes) != data[5]:
        return {"ok": False, "error": "crc"}
    temp_raw = (temp_raw_bytes[0] << 8) | temp_raw_bytes[1]
    rh_raw = (rh_raw_bytes[0] << 8) | rh_raw_bytes[1]
    temp_c = -45.0 + 175.0 * (temp_raw / 65535.0)
    humidity = 100.0 * (rh_raw / 65535.0)
    return {
        "ok": True,
        "tempC": round(temp_c, 1),
        "tempF": round((temp_c * 9.0 / 5.0) + 32.0, 1),
        "humidityPct": round(max(0.0, min(100.0, humidity)), 1),
        "address": f"0x{SHTC3_I2C_ADDR:02x}",
        "bus": SHTC3_I2C_BUS,
    }


def pi_diagnostics_payload():
    with state_lock:
        state_snapshot = dict(latest)
        rotor_snapshot = dict(rotor_live)
        try:
            movement_az, movement_el, raw_query_az, raw_el, zero_az = planned_rotctld_current_locked()
            reported_az, reported_el = reported_rotor_locked()
            raw_pan_az = raw_referenced_pan_az_locked()
        except Exception:
            movement_az = movement_el = raw_query_az = raw_el = zero_az = 0.0
            reported_az = reported_el = 0.0
            raw_pan_az = None
        serial_age = (time.time() - last_serial_ts) if last_serial_ts else None
        gps_age = (time.time() - last_gps_serial_ts) if last_gps_serial_ts else None
        fix_age = (time.time() - float(latest.get("gpsLastFixTs", 0.0))) if latest.get("gpsLastFixTs") else None

    temp_c = _read_first_float("/sys/class/thermal/thermal_zone0/temp", 1000.0)
    try:
        load1, load5, load15 = os.getloadavg()
    except Exception:
        load1 = load5 = load15 = None

    memory = {}
    try:
        for line in Path("/proc/meminfo").read_text().splitlines():
            parts = line.split()
            if len(parts) >= 2:
                memory[parts[0].rstrip(":")] = int(parts[1])
    except Exception:
        pass
    mem_total_kb = memory.get("MemTotal", 0)
    mem_available_kb = memory.get("MemAvailable", 0)
    mem_used_kb = max(0, mem_total_kb - mem_available_kb) if mem_total_kb else 0
    mem_percent = (mem_used_kb / mem_total_kb * 100.0) if mem_total_kb else None

    try:
        disk = os.statvfs(str(BASE_DIR))
        disk_total = disk.f_blocks * disk.f_frsize
        disk_free = disk.f_bavail * disk.f_frsize
        disk_used = max(0, disk_total - disk_free)
        disk_percent = (disk_used / disk_total * 100.0) if disk_total else None
    except Exception:
        disk_total = disk_used = 0
        disk_percent = None

    uptime_text = "--"
    try:
        uptime_text = _format_uptime(float(Path("/proc/uptime").read_text().split()[0]))
    except Exception:
        pass

    throttled_raw = _vcgencmd_throttled()
    enclosure = _read_shtc3_environment()
    return {
        "ok": True,
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "pi": {
            "tempC": round(temp_c, 1) if temp_c is not None else None,
            "tempF": round((temp_c * 9.0 / 5.0) + 32.0, 1) if temp_c is not None else None,
            "load1": round(load1, 2) if load1 is not None else None,
            "load5": round(load5, 2) if load5 is not None else None,
            "load15": round(load15, 2) if load15 is not None else None,
            "memUsedMb": round(mem_used_kb / 1024.0, 1) if mem_total_kb else None,
            "memTotalMb": round(mem_total_kb / 1024.0, 1) if mem_total_kb else None,
            "memPercent": round(mem_percent, 1) if mem_percent is not None else None,
            "diskUsedGb": round(disk_used / (1024.0 ** 3), 2) if disk_total else None,
            "diskTotalGb": round(disk_total / (1024.0 ** 3), 2) if disk_total else None,
            "diskPercent": round(disk_percent, 1) if disk_percent is not None else None,
            "uptime": uptime_text,
            "throttledRaw": throttled_raw,
            "throttledText": _decode_throttle(throttled_raw),
        },
        "enclosure": enclosure,
        "nucleo": {
            "serialPath": SERIAL_PATH,
            "serialAgeSec": round(serial_age, 1) if serial_age is not None else None,
            "resetStatus": state_snapshot.get("nucleoResetStatus", "--"),
            "rotorSource": state_snapshot.get("rotorSource", "--"),
            "az": state_snapshot.get("az", 0.0),
            "el": state_snapshot.get("el", 0.0),
            "movementAz": round(float(movement_az), 1),
            "movementEl": round(float(movement_el), 1),
            "reportedAz": round(float(reported_az), 1),
            "reportedEl": round(float(reported_el), 1),
            "rawPanAz": None if raw_pan_az is None else round(float(raw_pan_az), 1),
            "zeroRawAz": round(float(zero_az), 1),
            "status": state_snapshot.get("status", "--"),
            "queryValid": bool(rotor_snapshot.get("queryValid", False)),
            "queryPanOk": bool(rotor_snapshot.get("queryPanOk", False)),
            "queryTiltOk": bool(rotor_snapshot.get("queryTiltOk", False)),
            "queryState": str(rotor_snapshot.get("queryState", "--")),
            "queryPanRaw": int(rotor_snapshot.get("queryPanRaw", 0)),
            "queryTiltRaw": int(rotor_snapshot.get("queryTiltRaw", 0)),
            "rawAz": round(float(rotor_snapshot.get("queryAz", 0.0)), 1),
            "rawEl": round(float(rotor_snapshot.get("queryEl", 0.0)), 1),
            "rotorNetEnabled": bool(rotor_net.get("enabled")),
            "rotorNetActive": bool(rotor_net.get("active")),
            "rotorNetBusy": bool(rotor_net.get("busy")),
            "rotorNetClient": str(rotor_net.get("lastClient", "")),
            "rotorNetTargetAz": round(float(rotor_net.get("targetAz") or 0.0), 1),
            "rotorNetTargetEl": round(float(rotor_net.get("targetEl") or 0.0), 1),
        },
        "gps": {
            "serialPath": GPS_SERIAL_PATH or "nucleo-forwarded",
            "ageSec": round(gps_age, 1) if gps_age is not None else None,
            "fixAgeSec": round(fix_age, 1) if fix_age is not None else None,
            "fix": state_snapshot.get("gps", "NO FIX"),
            "sats": state_snapshot.get("sats", 0),
            "lat": state_snapshot.get("gpsLat", "--"),
            "lon": state_snapshot.get("gpsLon", "--"),
        },
        "network": {
            "wifi": state_snapshot.get("wifi", ""),
            "rotctldHost": ROTCTLD_HOST,
            "rotctldPort": ROTCTLD_PORT,
            "bridgePort": PORT,
        },
    }


def refresh_host_wifi():
    summary = host_wifi_summary()
    if not summary:
        return
    with state_lock:
        if not latest.get("wifi"):
            latest["wifi"] = summary


def parse_nmea(line):
    global last_gps_serial_ts
    if not line.startswith("$"):
        return False
    last_gps_serial_ts = time.time()

    with state_lock:
        latest["gpsLastNmea"] = line

    if "GSV" in line:
        parse_gsv(line)
    elif "GSA" in line:
        parse_gsa(line)

    if "GGA" in line:
        parts = line.split(",")
        fix = int(parts[6] or "0") if len(parts) > 6 and (parts[6] or "").isdigit() else 0
        sats = int(parts[7] or "0") if len(parts) > 7 and (parts[7] or "").isdigit() else 0
        lat = parse_nmea_coord(parts[2] if len(parts) > 2 else "", parts[3] if len(parts) > 3 else "", True)
        lon = parse_nmea_coord(parts[4] if len(parts) > 4 else "", parts[5] if len(parts) > 5 else "", False)
        try:
            alt_m = float(parts[9] or 0.0)
        except ValueError:
            alt_m = 0.0
        with state_lock:
            if sats:
                latest["sats"] = sats
            latest["gps"] = "OK" if fix > 0 else "NO FIX"
            latest["gpsFixQuality"] = gps_fix_quality_label(fix)
            coord_ok = fix > 0 and lat is not None and lon is not None and gps_coord_jump_ok(lat, lon)
            if coord_ok:
                latest["gpsLastFixTs"] = time.time()
            latest["status"] = "GPS NMEA LIVE"
            if coord_ok:
                latest["gpsLat"] = f"{lat:.5f}"
                latest["gpsLon"] = f"{lon:.5f}"
            if alt_m:
                latest["gpsAltFt"] = str(round(alt_m * 3.28084))
            hdop = (parts[8] if len(parts) > 8 else "").split("*")[0].strip()
            if hdop:
                latest["gpsHdop"] = hdop
    elif "RMC" in line:
        parts = line.split(",")
        utc_text, date_text = parse_rmc_datetime(parts[1] if len(parts) > 1 else "", parts[9] if len(parts) > 9 else "")
        try:
            speed_knots = float(parts[7] or 0.0) if len(parts) > 7 else 0.0
        except ValueError:
            speed_knots = 0.0
        try:
            track_deg = float((parts[8] or "").split("*")[0]) if len(parts) > 8 and parts[8] else None
        except ValueError:
            track_deg = None
        with state_lock:
            if utc_text:
                latest["gpsUtc"] = utc_text
            if date_text:
                latest["gpsDate"] = date_text
            latest["gpsSpeedKmh"] = f"{speed_knots * 1.852:.2f}"
            latest["gpsSpeedMph"] = f"{speed_knots * 1.15078:.2f}"
            latest["gpsTrackDeg"] = f"{track_deg:.1f}°" if track_deg is not None else "--"
    elif "VTG" in line:
        parts = line.split(",")
        try:
            track_true = float(parts[1] or 0.0) if len(parts) > 1 and parts[1] else None
        except ValueError:
            track_true = None
        try:
            speed_kmh = float((parts[7] or "").split("*")[0]) if len(parts) > 7 and parts[7] else None
        except ValueError:
            speed_kmh = None
        with state_lock:
            if track_true is not None:
                latest["gpsTrackDeg"] = f"{track_true:.1f}°"
            if speed_kmh is not None:
                latest["gpsSpeedKmh"] = f"{speed_kmh:.2f}"
                latest["gpsSpeedMph"] = f"{speed_kmh * 0.621371:.2f}"
    return True


GPS_RE = re.compile(r"^gps chars=\d+\s+sats=(\d+)\s+fix=(yes|no)$", re.I)
GPS_SIG_RE = re.compile(
    r"^gpssig\s+sys=(\S+)\s+prn=(\d+)\s+sig=([A-Za-z0-9]+)\s+cno=(-?\d+)\s+used=([01])(?:\s+qual=(\d+))?$",
    re.I,
)
GPS_AUX_RE = re.compile(
    r"^gps aux pps=(\d+)\s+seen=(yes|no)\s+rst=(hi|lo)\s+en=(hi|lo)\s+eint=(hi|lo)\s+sbt=(hi|lo)$",
    re.I,
)
WIFI_RE = re.compile(r"^wifi mode=(\S+)\s+ssid=(.+?)\s+ip=(\S+)$", re.I)
ENC_AZ_RE = re.compile(r"^enc az([+-])$", re.I)
ENC_EL_RE = re.compile(r"^enc el([+-])$", re.I)
ENC_MENU_RE = re.compile(r"^enc menu([+-])$", re.I)
SWITCH_RE = re.compile(r"^(AZ_SW|EL_SW|MENU_SW)\s+(pressed|released)$", re.I)
ROTOR_RE = re.compile(
    r"^rotor az=([-\d.]+)\s+el=([-\d.]+)\s+tx=([-\d.]+)\s+rx=([-\d.]+)"
    r"(?:\s+sat=(\S+))?(?:\s+txmode=(\S+))?(?:\s+rxmode=(\S+))?$",
    re.I,
)
ROTOR_TOTALAZ_RE = re.compile(r"^rotor azTotal=([-\d.]+)$", re.I)
ROTOR_QUERY_RE = re.compile(r"^rotor query panok=(\d+)\s+panraw=(\d+)\s+tiltok=(\d+)\s+tiltraw=(\d+)$", re.I)


def update_gps_signal_summary_locked():
    dual_band = 0
    best_l5 = None
    qzss_seen = 0

    for sat in latest.get("gpsSatellites", []):
        signals = sat.get("signals") or []
        if len(signals) >= 2:
            dual_band += 1
        if str(sat.get("system", "")).upper() == "QZSS":
            qzss_seen += 1
        for signal in signals:
            band = str(signal.get("band", "")).upper()
            try:
                cno = int(signal.get("cno"))
            except Exception:
                continue
            if "L5" in band and (best_l5 is None or cno > best_l5):
                best_l5 = cno

    latest["gpsDualBandCount"] = dual_band
    latest["gpsBestL5Snr"] = "--" if best_l5 is None else best_l5
    if dual_band or best_l5 is not None or qzss_seen:
        parts = [f"DUAL {dual_band}"]
        if best_l5 is not None:
            parts.append(f"L5 {best_l5} dB")
        if qzss_seen:
            parts.append(f"QZSS {qzss_seen}")
        latest["gpsSignalAux"] = " | ".join(parts)
        latest["gpsAux"] = latest["gpsSignalAux"]
    elif latest.get("gpsPpsAux") and latest.get("gpsPpsAux") != "--":
        latest["gpsAux"] = latest["gpsPpsAux"]


def parse_gps_signal_line(line):
    m = GPS_SIG_RE.match(line.strip())
    if not m:
        return False

    system = normalize_system_for_prn(m.group(1).upper(), m.group(2))
    if not allowed_system(system):
        return True
    prn = m.group(2)
    band = m.group(3).upper()
    try:
        cno = int(m.group(4))
    except ValueError:
        cno = -1
    used = m.group(5) == "1"
    quality = int(m.group(6) or 0)
    key = f"{system}-{prn}"

    with state_lock:
        idx = next((j for j, s in enumerate(latest["gpsSatellites"]) if s.get("key") == key), -1)
        if idx >= 0:
            row = latest["gpsSatellites"][idx]
        else:
            row = {
                "key": key,
                "prn": prn,
                "system": system,
                "el": "",
                "az": "",
                "snr": None,
                "used": used,
                "signals": [],
            }
            latest["gpsSatellites"].append(row)

        signals = list(row.get("signals") or [])
        signal_row = {"band": band, "cno": cno, "used": used, "quality": quality}
        existing = next((j for j, s in enumerate(signals) if str(s.get("band", "")).upper() == band), -1)
        if existing >= 0:
            signals[existing] = signal_row
        else:
            signals.append(signal_row)
        signals = sorted(signals, key=lambda s: str(s.get("band", "")))

        row["signals"] = signals
        if cno >= 0:
            row["snr"] = max(cno, int(row.get("snr") or 0))
        row["used"] = bool(row.get("used")) or used
        latest["gpsSatellites"] = sorted(
            latest["gpsSatellites"][-64:],
            key=lambda s: (s.get("snr") is not None, s.get("snr") or -1),
            reverse=True,
        )
        update_gps_signal_summary_locked()
        latest["status"] = "GPS NAV-SIG"
    return True


def handle_serial_line(line):
    global last_serial_ts
    last_serial_ts = time.time()
    s = line.strip()
    if not s:
        return

    if parse_nmea(s):
        return

    if parse_gps_signal_line(s):
        return

    m = GPS_RE.match(s)
    if m:
        # When the Pi GPS UART is live, prefer it over stale controller-side
        # `gps chars=0` summaries from the H743.
        if last_gps_serial_ts and (time.time() - last_gps_serial_ts) < 5.0:
            return
        with state_lock:
            latest["sats"] = int(m.group(1))
            latest["gps"] = "OK" if m.group(2).lower() == "yes" else "NO FIX"
            latest["status"] = "GPS LIVE"
        return

    m = GPS_AUX_RE.match(s)
    if m:
        with state_lock:
            latest["gpsPpsCount"] = int(m.group(1))
            latest["gpsPpsSeen"] = m.group(2).lower() == "yes"
            latest["gpsRst"] = m.group(3).upper()
            latest["gpsEn"] = m.group(4).upper()
            latest["gpsExtint"] = m.group(5).upper()
            latest["gpsSafeboot"] = m.group(6).upper()
            latest["gpsPpsAux"] = (
                f"TIMEPULSE {'LOCKED' if latest['gpsPpsSeen'] else 'WAITING'} "
                f"| PPS {latest['gpsPpsCount']} "
                f"| RX {'ON' if latest['gpsEn'] == 'HI' else 'OFF'}"
            )
            if not latest.get("gpsSignalAux") or latest.get("gpsSignalAux") == "--":
                latest["gpsAux"] = latest["gpsPpsAux"]
            latest["status"] = "GPS AUX"
        return

    m = WIFI_RE.match(s)
    if m:
        with state_lock:
            latest["wifi"] = f"{m.group(1)} {m.group(2)} {m.group(3)}"
            latest["status"] = "WIFI LIVE"
        return

    if re.match(r"^nucleo alive$", s, re.I):
        with state_lock:
            latest["nucleoResetStatus"] = f"NUCLEO BACK ONLINE {datetime.now().strftime('%H:%M:%S')}"
            latest["status"] = "NUCLEO ALIVE"
        return

    if re.match(r"^giga alive$", s, re.I):
        with state_lock:
            latest["status"] = "GIGA ALIVE"
        return

    if re.match(r"^(rs485 |serial4 |gps raw=)", s, re.I):
        with state_lock:
            latest["status"] = "TELEMETRY"
        return

    if re.match(r"^target (armed|drive|hold)", s, re.I):
        with state_lock:
            latest["targetDiag"] = s
            latest["status"] = s.upper()[:64]
        return

    m = SWITCH_RE.match(s)
    if m:
        label = m.group(1).upper()
        event = m.group(2).upper()
        with state_lock:
            latest["status"] = f"{label} {event}"
            if label == "MENU_SW" and event == "PRESSED":
                latest["menuPresses"] = int(latest.get("menuPresses", 0)) + 1
        return

    m = ENC_AZ_RE.match(s)
    if m:
        with state_lock:
            if sat_lock.get("enabled") or rotor_net.get("lastClient") == "sat-lock":
                sat_lock["enabled"] = False
                clear_sat_lock_last_good_locked()
                rotor_net["active"] = False
                rotor_net["lastCmd"] = "PTZ STOP"
                rotor_net["lastSendMs"] = 0.0
                rotor_net["homePrevAzErr"] = None
                rotor_net["homePrevElErr"] = None
                rotor_net["homeWorseTicks"] = 0
            latest["status"] = "ENCODER AZ"
        return

    m = ENC_EL_RE.match(s)
    if m:
        with state_lock:
            if sat_lock.get("enabled") or rotor_net.get("lastClient") == "sat-lock":
                sat_lock["enabled"] = False
                clear_sat_lock_last_good_locked()
                rotor_net["active"] = False
                rotor_net["lastCmd"] = "PTZ STOP"
                rotor_net["lastSendMs"] = 0.0
                rotor_net["homePrevAzErr"] = None
                rotor_net["homePrevElErr"] = None
                rotor_net["homeWorseTicks"] = 0
            latest["status"] = "ENCODER EL"
        return

    m = ENC_MENU_RE.match(s)
    if m:
        step = 1 if m.group(1) == "+" else -1
        with state_lock:
            latest["status"] = "ENCODER MENU"
            latest["menuTicks"] = int(latest.get("menuTicks", 0)) + step
        return

    m = ROTOR_TOTALAZ_RE.match(s)
    if m:
        with state_lock:
            rotor_live["azTotal"] = float(m.group(1))
        return

    m = ROTOR_QUERY_RE.match(s)
    if m:
        with state_lock:
            rotor_live["queryPanOk"] = m.group(1) == "1"
            rotor_live["queryPanRaw"] = int(m.group(2))
            rotor_live["queryTiltOk"] = m.group(3) == "1"
            rotor_live["queryTiltRaw"] = int(m.group(4))
        return

    m = ROTOR_RE.match(s)
    if m:
        with state_lock:
            pan_ok = bool(rotor_live.get("queryPanOk", False))
            tilt_ok = bool(rotor_live.get("queryTiltOk", False))
            pose_ok = pan_ok and tilt_ok
            if pan_ok:
                rotor_live["queryAz"] = float(m.group(1))
                refresh_fused_az_locked()
            if tilt_ok:
                rotor_live["queryEl"] = float(m.group(2))
                rotor_live["el"] = float(m.group(2))
            rotor_live["queryValid"] = pose_ok
            latest["tx"] = float(m.group(3))
            latest["rx"] = float(m.group(4))
            if m.group(5):
                latest["sat"] = m.group(5)
            if m.group(6):
                latest["txMode"] = m.group(6)
            if m.group(7):
                latest["rxMode"] = m.group(7)
            if pose_ok:
                latest["rotorSource"] = "LIVE"
                latest["status"] = "ROTOR LIVE"
                if rotor_zero["enabled"]:
                    apply_encoder_overlay_locked()
                else:
                    latest["az"] = round(norm_az(float(rotor_live["queryAz"])), 1)
                    latest["azTotal"] = round(norm_az(float(rotor_live["queryAz"])), 1)
                    latest["el"] = round(float(rotor_live["queryEl"]), 1)
            else:
                latest["status"] = "ROTOR QUERY INVALID"
        return


def configure_serial(path, baud):
    if os.uname().sysname == "Darwin":
        cmd = ["stty", "-f", path, str(baud), "raw", "-echo"]
    else:
        cmd = ["stty", "-F", path, str(baud), "raw", "-echo"]
    subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.5)


def open_serial():
    global serial_port, serial_file
    for path in candidate_serial_paths():
        try:
            serial_port = path
            serial_file = serial.Serial(path, SERIAL_BAUD, timeout=1)
            try:
                serial_file.dtr = False
                serial_file.rts = False
                time.sleep(0.05)
                serial_file.reset_input_buffer()
                serial_file.dtr = True
                time.sleep(0.25)
            except Exception:
                pass
            with state_lock:
                latest["status"] = f"SERIAL {path} @ {SERIAL_BAUD}"
            return True
        except Exception:
            continue
    with state_lock:
        latest["status"] = "SERIAL OPEN ERROR"
    return False


def open_gps_serial():
    global gps_serial_port, gps_serial_file
    if not GPS_SERIAL_PATH or not os.path.exists(GPS_SERIAL_PATH):
        return False
    try:
        configure_serial(GPS_SERIAL_PATH, GPS_SERIAL_BAUD)
        fd = os.open(GPS_SERIAL_PATH, os.O_RDONLY | os.O_NOCTTY)
        gps_serial_port = GPS_SERIAL_PATH
        gps_serial_file = os.fdopen(fd, "rb", buffering=0)
        with state_lock:
            latest["status"] = f"GPS UART {GPS_SERIAL_PATH} @ {GPS_SERIAL_BAUD}"
        return True
    except OSError:
        gps_serial_port = None
        gps_serial_file = None
        return False


def serial_reader():
    global serial_file, serial_port
    buf = bytearray()
    while True:
        try:
            if serial_file is None and not open_serial():
                time.sleep(1.5)
                continue

            chunk = serial_file.read(1)
            if not chunk:
                raise OSError("serial eof")
            if chunk == b"\n":
                line = buf.decode("utf-8", errors="replace")
                buf.clear()
                handle_serial_line(line)
            elif chunk != b"\r":
                buf.extend(chunk)
        except Exception as exc:
            with state_lock:
                latest["status"] = f"SERIAL RECOVER: {exc}"
            try:
                if serial_file is not None:
                    serial_file.close()
            except Exception:
                pass
            serial_file = None
            serial_port = None
            buf.clear()
            time.sleep(1.0)


def gps_serial_reader():
    global gps_serial_file, gps_serial_port, last_gps_serial_ts
    buf = bytearray()
    while True:
        try:
            if gps_serial_file is None and not open_gps_serial():
                time.sleep(1.5)
                continue

            chunk = gps_serial_file.read(1)
            if not chunk:
                raise OSError("gps serial eof")
            last_gps_serial_ts = time.time()
            if chunk == b"\n":
                line = buf.decode("utf-8", errors="replace")
                buf.clear()
                if parse_nmea(line.strip()):
                    with state_lock:
                        latest["status"] = "GPS PI UART LIVE"
                continue
            if chunk != b"\r":
                buf.extend(chunk)
        except Exception:
            try:
                if gps_serial_file is not None:
                    gps_serial_file.close()
            except Exception:
                pass
            gps_serial_file = None
            gps_serial_port = None
            buf.clear()
            time.sleep(1.0)


def serial_write(line):
    if not line:
        return False
    payload = (line + "\n").encode("utf-8")
    for attempt in range(3):
        reopen_reason = ""
        with serial_writer_lock:
            if serial_file is not None:
                try:
                    serial_file.write(payload)
                    serial_file.flush()
                    return True
                except Exception as exc:
                    reopen_reason = f"write {exc}"
        if reopen_reason:
            force_serial_reopen(reopen_reason)
        elif attempt == 0:
            force_serial_reopen("write unavailable")
        time.sleep(0.15)
    return False


def pulse_nucleo_reset():
    # The Pi is connected to the Nucleo's native USB peripheral, not the
    # ST-Link VCOM. DTR/RTS cannot hit NRST on this path, so ask firmware to
    # reset itself with NVIC_SystemReset().
    return serial_write("RESET NUCLEO")


def serial_burst_write(line, count=1, gap_ms=35):
    count = max(1, int(count))
    for idx in range(count):
        if not serial_write(line):
            return False
        if idx + 1 < count:
            time.sleep(gap_ms / 1000.0)
    return True


def close_external_rotator_serial_locked():
    global external_rotator_serial, external_rotator_profile_id
    try:
        if external_rotator_serial is not None:
            external_rotator_serial.close()
    except Exception:
        pass
    external_rotator_serial = None
    external_rotator_profile_id = ""


def ensure_external_rotator_serial():
    global external_rotator_serial, external_rotator_profile_id
    profile_id = active_rotator_profile_id()
    profile = active_rotator_profile()
    if profile.get("transport") != "rs232-db9" or not EXTERNAL_ROTATOR_SERIAL_PATH:
        return None

    with external_rotator_lock:
        if (
            external_rotator_serial is not None
            and getattr(external_rotator_serial, "is_open", False)
            and external_rotator_profile_id == profile_id
        ):
            return external_rotator_serial

        close_external_rotator_serial_locked()
        try:
            external_rotator_serial = serial.Serial(
                EXTERNAL_ROTATOR_SERIAL_PATH,
                baudrate=int(profile.get("baud") or 9600),
                bytesize=int(profile.get("dataBits") or 8),
                parity=str(profile.get("parity") or "N"),
                stopbits=int(profile.get("stopBits") or 1),
                timeout=1,
            )
            external_rotator_profile_id = profile_id
            return external_rotator_serial
        except Exception as exc:
            external_rotator_serial = None
            external_rotator_profile_id = ""
            with state_lock:
                latest["status"] = f"EXT ROTATOR OPEN FAIL {exc}"
            return None


def encode_yaesu_gs232(az_deg, el_deg):
    az_int = int(round(norm_az(float(az_deg)))) % 360
    el_int = max(0, min(180, int(round(float(el_deg)))))
    return f"W{az_int:03d} {el_int:03d}\r\n".encode("ascii")


def send_spid_native_position(az_deg, el_deg):
    # Native SPID binary mode is intentionally guarded. Prefer Yaesu-emulation
    # on SPID controllers until the rot2prog package and hardware are present.
    try:
        from rot2prog import ROT2Prog  # type: ignore
    except Exception:
        with state_lock:
            latest["status"] = "SPID NATIVE NEEDS rot2prog"
        return False

    try:
        rotor = ROT2Prog(EXTERNAL_ROTATOR_SERIAL_PATH)
        if hasattr(rotor, "set_pos"):
            rotor.set_pos(float(az_deg), float(el_deg))
        elif hasattr(rotor, "set_position"):
            rotor.set_position(float(az_deg), float(el_deg))
        else:
            with state_lock:
                latest["status"] = "SPID NATIVE API UNKNOWN"
            return False
        return True
    except Exception as exc:
        with state_lock:
            latest["status"] = f"SPID SEND FAIL {exc}"
        return False


def send_external_rotator_position(az_deg, el_deg):
    profile_id = active_rotator_profile_id()
    if profile_id in ("yaesu-gs232", "spid-md0x-yaesu", "antrunner-pro"):
        ser = ensure_external_rotator_serial()
        if ser is None:
            return False
        frame = encode_yaesu_gs232(az_deg, el_deg)
        try:
            ser.write(frame)
            ser.flush()
            with state_lock:
                latest["status"] = f"EXT ROTATOR {frame.decode('ascii', errors='replace').strip()}"
            return True
        except Exception as exc:
            with state_lock:
                latest["status"] = f"EXT ROTATOR SEND FAIL {exc}"
            return False

    if profile_id in ("spid-rot1prog", "spid-rot2prog"):
        return send_spid_native_position(az_deg, el_deg)

    if profile_id == "hamlib-rotctld":
        with state_lock:
            latest["status"] = "HAMLIB PROFILE SELECTED"
        return True

    return False


def send_position_command(az_deg, el_deg):
    if active_rotator_is_internal():
        return send_absolute_position(az_deg, el_deg)
    if active_rotator_profile_id() == "hamlib-rotctld":
        with state_lock:
            latest["status"] = "HAMLIB PROFILE SELECTED"
        return True
    return send_external_rotator_position(az_deg, el_deg)


def sync_az_limit_zero_reference():
    with state_lock:
        rotor_limit["prevReferencedAz"] = None
        rotor_limit["travelAz"] = 0.0
        latest["status"] = "AZ LIMIT DISABLED"
    return True


def ptz_command(action, axis="", direction=""):
    action = (action or "").lower()
    axis = (axis or "").lower()
    direction = (direction or "").lower()
    if action == "stop":
        return "PTZ STOP"
    if action == "jog" and axis == "az":
        return "PTZ PAN -" if direction == "neg" else "PTZ PAN +"
    if action == "jog" and axis == "el":
        return "PTZ TILT -" if direction == "neg" else "PTZ TILT +"
    return ""


def send_absolute_position(az_deg, el_deg):
    # The firmware understands an absolute "go to this exact angle" command
    # (parsed as "P <az> <el>", calling pelco.setPosition() in firmware),
    # distinct from the jog/pulse commands used elsewhere in this file. Letting
    # the head's own Pelco-D positioning logic handle the actual movement
    # smoothing avoids the externally-simulated jog/pulse/stop cycling that
    # causes visible jerkiness.
    az_deg = norm_az(float(az_deg))
    el_deg = max(ROTOR_PHYSICAL_EL_MIN_DEG, min(ROTOR_PHYSICAL_EL_MAX_DEG, float(el_deg)))
    line = f"P {az_deg:.2f} {el_deg:.2f}"
    print(f"SENDING ABSOLUTE POSITION: {line}")
    ok = serial_write(line)
    print(f"SERIAL WRITE RESULT: {ok}")
    if ok:
        with state_lock:
            latest["status"] = f"ROTOR TRACK {line}"
    return ok


def display_target_to_raw_position(az_deg, el_deg):
    display_az = norm_az(float(az_deg))
    display_el = float(el_deg)
    with state_lock:
        zero_enabled = bool(rotor_zero.get("enabled"))
        zero_az = norm_az(float(rotor_zero.get("az", 0.0)))
        zero_el = float(rotor_zero.get("el", 0.0))

    if zero_enabled:
        raw_az = norm_az(zero_az + display_az)
        raw_el = zero_el - display_el
    else:
        raw_az = norm_az(display_az + ROTOR_AZ_PHASE_OFFSET_DEG)
        raw_el = display_el

    return raw_az, max(ROTOR_PHYSICAL_EL_MIN_DEG, min(ROTOR_PHYSICAL_EL_MAX_DEG, raw_el))


def send_display_absolute_position(az_deg, el_deg):
    raw_az, raw_el = display_target_to_raw_position(az_deg, el_deg)
    ok = send_absolute_position(raw_az, raw_el)
    if ok:
        with state_lock:
            latest["status"] = (
                f"ROTOR ABS DISPLAY AZ {float(az_deg):.1f} EL {float(el_deg):.1f} "
                f"RAW {raw_az:.1f}/{raw_el:.1f}"
            )
    return ok


def send_ptz_command(action, axis="", direction=""):
    cmd = ptz_command(action, axis, direction)
    if not cmd:
        return False, ""

    burst_count = 2 if action == "stop" else 1
    ok = serial_burst_write(cmd, burst_count, 35)
    if ok:
        with state_lock:
            latest["status"] = cmd
    return ok, cmd


def send_ptz_pulse(action, axis="", direction="", pulse_ms=120):
    cmd = ptz_command(action, axis, direction)
    if not cmd:
        return False, ""
    if not serial_write(cmd):
        return False, cmd
    time.sleep(max(0.02, float(pulse_ms) / 1000.0))
    stop_ok = serial_burst_write("PTZ STOP", 2, 35)
    if stop_ok:
        with state_lock:
            latest["status"] = f"{cmd} PULSE"
    return stop_ok, cmd


def rotor_sleep_with_cancel(duration):
    end = time.time() + max(0.0, float(duration))
    start = time.time()
    while time.time() < end:
        if rotor_net_cancel_event.is_set():
            break
        with state_lock:
            if not bool(rotor_net.get("active", False)):
                break
        time.sleep(min(0.05, max(0.0, end - time.time())))
    return max(0.0, time.time() - start), rotor_net_cancel_event.is_set()


def planned_axis_time(delta_deg, speed_dps):
    speed = float(speed_dps)
    if speed <= 0.0:
        return 0.0
    safety = max(0.5, min(1.05, float(ROTOR_PLANNED_TIME_SAFETY_FACTOR)))
    return min(max(0.0, float(ROTOR_PLANNED_MAX_AXIS_SEC)), abs(float(delta_deg)) / speed * safety)


def calibration_log_record(payload):
    record = dict(payload)
    record["ts"] = datetime.now(timezone.utc).isoformat()
    line = json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n"
    try:
        with calibration_log_lock:
            with ROTOR_CALIBRATION_LOG_PATH.open("a", encoding="utf-8") as fh:
                fh.write(line)
    except Exception as exc:
        with state_lock:
            latest["status"] = f"CAL LOG FAIL {exc}"


def rotor_calibration_snapshot_locked():
    current_az, current_el, raw_query_az, raw_el, zero_az = planned_rotctld_current_locked()
    reported_az, reported_el = reported_rotor_locked()
    raw_pan_display_az = raw_referenced_pan_az_locked()
    pan_display_az = referenced_pan_az_locked()
    readout_corr = 0.0 if raw_pan_display_az is None else readout_az_correction(raw_pan_display_az)
    display_corr = display_az_correction(current_az)
    return {
        "displayAz": round(float(current_az), 3),
        "displayEl": round(float(current_el), 3),
        "movementAz": round(float(current_az), 3),
        "movementEl": round(float(current_el), 3),
        "reportedAz": round(float(reported_az), 3),
        "reportedEl": round(float(reported_el), 3),
        "rawAz": round(float(raw_query_az), 3),
        "rawEl": round(float(raw_el), 3),
        "zeroRawAz": round(float(zero_az), 3),
        "zeroRawEl": round(float(rotor_zero.get("el", 0.0)), 3),
        "zeroPanRaw": None if rotor_zero.get("panRaw") is None else int(current_zero_pan_raw_locked()),
        "rawPanAz": None if raw_pan_display_az is None else round(float(raw_pan_display_az), 3),
        "movementAzFromPanRaw": None if raw_pan_display_az is None else round(float(raw_pan_display_az), 3),
        "movementAzCorrected": None if pan_display_az is None else round(float(pan_display_az), 3),
        "displayAzFromPanRaw": None if raw_pan_display_az is None else round(float(raw_pan_display_az), 3),
        "displayAzCorrected": None if pan_display_az is None else round(float(pan_display_az), 3),
        "readoutAzCorrection": round(float(readout_corr), 3),
        "displayAzCorrection": round(float(display_corr), 3),
        "queryValid": bool(rotor_live.get("queryValid", False)),
        "queryPanOk": bool(rotor_live.get("queryPanOk", False)),
        "queryTiltOk": bool(rotor_live.get("queryTiltOk", False)),
        "queryState": str(rotor_live.get("queryState", "")),
        "queryPanRaw": int(rotor_live.get("queryPanRaw", 0)),
        "queryTiltRaw": int(rotor_live.get("queryTiltRaw", 0)),
    }


def planned_rotctld_current_locked():
    raw_query_az = norm_az(float(rotor_live.get("queryAz", rotor_live.get("az", latest.get("az", 0.0)))))
    raw_el = float(rotor_live.get("queryEl", rotor_live.get("el", latest.get("el", 0.0))))
    if rotor_zero["enabled"]:
        zero_az = norm_az(float(rotor_zero.get("az", 0.0)))
        pan_az = referenced_pan_az_locked()
        display_az = pan_az if pan_az is not None else norm_az(raw_query_az - zero_az)
        return display_az, float(rotor_zero["el"]) - raw_el, raw_query_az, raw_el, zero_az
    return referenced_rotor_locked()[0], referenced_rotor_locked()[1], raw_query_az, raw_el, 0.0


def run_planned_rotctld_move(target_az, target_el, move_seq):
    move_started_ts = time.time()
    move_seq = int(move_seq or 0)
    requested_target_az = norm_az(float(target_az))
    drive_target_az, az_correction = corrected_planned_target_az(requested_target_az)
    display_target_el = float(target_el)
    if not math.isfinite(display_target_el) or display_target_el < 0.0 or display_target_el > 90.0:
        with state_lock:
            if int(rotor_net.get("commandSeq", 0)) == move_seq:
                rotor_net["active"] = False
                rotor_net["targetAz"] = None
                rotor_net["targetEl"] = None
                rotor_net["lastCmd"] = "PTZ STOP"
                rotor_net["lastSendMs"] = 0.0
                latest["status"] = f"ROTOR PLAN HALT UNSAFE EL {display_target_el:.1f}"
        send_ptz_command("stop", "", "")
        return

    with state_lock:
        query_valid = bool(rotor_live.get("queryValid", False))
        if query_valid:
            current_az, current_el, raw_query_az, raw_el, zero_az = planned_rotctld_current_locked()
        else:
            current_az, current_el = float(latest.get("az", 0.0)), float(latest.get("el", 0.0))
            raw_query_az, raw_el, zero_az = current_az, current_el, 0.0
        start_snapshot = rotor_calibration_snapshot_locked() if query_valid else {
            "displayAz": round(float(current_az), 3),
            "displayEl": round(float(current_el), 3),
            "rawAz": round(float(raw_query_az), 3),
            "rawEl": round(float(raw_el), 3),
            "zeroRawAz": round(float(zero_az), 3),
            "zeroRawEl": round(float(rotor_zero.get("el", 0.0)), 3),
            "queryValid": False,
        }

    d_az = normalize_angle_delta(float(drive_target_az) - float(current_az))
    d_el = float(target_el) - float(current_el)

    if abs(d_az) <= ROTOR_PLANNED_AZ_TOL_DEG and abs(d_el) <= ROTOR_PLANNED_EL_TOL_DEG:
        ok, _ = send_ptz_command("stop", "", "")
        with state_lock:
            if int(rotor_net.get("commandSeq", 0)) == move_seq:
                rotor_net["active"] = False
                rotor_net["lastCmd"] = "PTZ STOP"
                rotor_net["lastSendMs"] = time.time() if ok else rotor_net["lastSendMs"]
                latest["status"] = "ROTOR PLAN HOLD"
        calibration_log_record({
            "event": "planned-hold",
            "commandSeq": move_seq,
            "client": str(rotor_net.get("lastClient", "")),
            "target": {"az": round(float(requested_target_az), 3), "el": round(float(target_el), 3)},
            "driveTarget": {"az": round(float(drive_target_az), 3), "azCorrection": round(float(az_correction), 3)},
            "start": start_snapshot,
            "delta": {"az": round(float(d_az), 3), "el": round(float(d_el), 3)},
        })
        return

    rotor_net_cancel_event.clear()
    with state_lock:
        latest["status"] = (
            f"ROTOR PLAN AZ {requested_target_az:.1f}->{drive_target_az:.1f} EL {float(target_el):.1f} "
            f"CUR {current_az:.1f}/{current_el:.1f} RAW {raw_query_az:.1f}/{raw_el:.1f} ZERO {zero_az:.1f}"
        )

    axes = []
    axes_log = []
    if abs(d_az) > ROTOR_PLANNED_AZ_TOL_DEG:
        az_duration = planned_axis_time(d_az, ROTOR_PLANNED_AZ_SPEED_DPS)
        axes.append(
            (
                "az",
                "pos" if d_az > 0.0 else "neg",
                az_duration,
                d_az,
            )
        )
        axes_log.append({"axis": "az", "direction": "pos" if d_az > 0.0 else "neg", "durationSec": round(float(az_duration), 3), "deltaDeg": round(float(d_az), 3)})
    if abs(d_el) > ROTOR_PLANNED_EL_TOL_DEG:
        el_duration = planned_axis_time(d_el, ROTOR_PLANNED_EL_SPEED_DPS)
        axes.append(
            (
                "el",
                "pos" if d_el > 0.0 else "neg",
                el_duration,
                d_el,
            )
        )
        axes_log.append({"axis": "el", "direction": "pos" if d_el > 0.0 else "neg", "durationSec": round(float(el_duration), 3), "deltaDeg": round(float(d_el), 3)})

    interrupted = False
    stop_failures = 0
    for axis, direction, duration, delta in axes:
        if duration <= 0.0:
            continue
        ok, sent_cmd = send_ptz_command("jog", axis, direction)
        if not ok:
            with state_lock:
                latest["status"] = f"ROTOR PLAN SEND FAIL {axis.upper()}"
            interrupted = True
            break
        with state_lock:
            rotor_net["lastCmd"] = sent_cmd
            rotor_net["lastSendMs"] = time.time()
            latest["status"] = (
                f"ROTOR PLAN {sent_cmd} {duration:.2f}s "
                f"ERR {float(delta):.1f}"
            )
        _, canceled = rotor_sleep_with_cancel(duration)
        if not serial_burst_write("PTZ STOP", 4, 35):
            stop_failures += 1
        if canceled:
            interrupted = True
            break

    if not serial_burst_write("PTZ STOP", 4, 35):
        stop_failures += 1
    if ROTOR_PLANNED_READBACK_DELAY_SEC > 0.0:
        time.sleep(min(2.0, max(0.0, float(ROTOR_PLANNED_READBACK_DELAY_SEC))))
    with state_lock:
        final_snapshot = rotor_calibration_snapshot_locked()
    final_az_error = normalize_angle_delta(float(target_az) - float(final_snapshot["displayAz"]))
    final_el_error = float(target_el) - float(final_snapshot["displayEl"])
    calibration_log_record({
        "event": "planned-move",
        "commandSeq": move_seq,
        "client": str(rotor_net.get("lastClient", "")),
        "target": {"az": round(float(requested_target_az), 3), "el": round(float(target_el), 3)},
        "driveTarget": {"az": round(float(drive_target_az), 3), "azCorrection": round(float(az_correction), 3)},
        "start": start_snapshot,
        "delta": {"az": round(float(d_az), 3), "el": round(float(d_el), 3)},
        "axes": axes_log,
        "final": final_snapshot,
        "finalError": {"az": round(float(final_az_error), 3), "el": round(float(final_el_error), 3)},
        "interrupted": bool(interrupted),
        "stopFailures": int(stop_failures),
        "elapsedSec": round(time.time() - move_started_ts, 3),
    })
    with state_lock:
        if int(rotor_net.get("commandSeq", 0)) == move_seq:
            rotor_net["active"] = False
            rotor_net["lastCmd"] = "PTZ STOP"
            rotor_net["lastSendMs"] = time.time()
            latest["status"] = (
                "ROTOR PLAN INTERRUPTED"
                if interrupted
                else f"ROTOR PLAN DONE ERR {final_az_error:.1f}/{final_el_error:.1f}"
            )
        else:
            latest["status"] = "ROTOR PLAN SUPERSEDED"


def set_rotor_net_target(az_deg, el_deg, client_name=""):
    antenna_az = norm_az(float(az_deg))
    antenna_el = clamp(float(el_deg), 0.0, 90.0)
    rotor_net_cancel_event.set()
    with state_lock:
        rotor_net["commandSeq"] = int(rotor_net.get("commandSeq", 0)) + 1
        rotor_net["targetAz"] = antenna_az
        rotor_net["targetEl"] = antenna_el
        rotor_net["active"] = True
        rotor_net["busy"] = False
        rotor_net["lastClient"] = client_name or ""
        rotor_net["lastUpdateMs"] = time.time()
        if client_name == "ui-home":
            rotor_net["homeHoldTs"] = 0.0
        latest["rotorSource"] = "NET"
        latest["status"] = f"ROTOR TARGET AZ {antenna_az:.1f} EL {antenna_el:.1f}"


def classify_rotctld_set_pos_client(client_name):
    now = time.time()
    key = str(client_name or "rotctld")
    stream = rotctld_client_streams.get(key, {})
    last_ts = float(stream.get("lastTs") or 0.0)
    tracking_until = float(stream.get("trackingUntil") or 0.0)
    recent = last_ts > 0.0 and (now - last_ts) <= ROTCTLD_TRACK_PROMOTE_WINDOW_SEC
    count = int(stream.get("count") or 0) + 1 if recent else 1
    is_tracking = now <= tracking_until or (recent and count >= 2)
    rotctld_client_streams[key] = {
        "lastTs": now,
        "count": count,
        "trackingUntil": now + ROTCTLD_TRACK_STALE_SEC if is_tracking else tracking_until,
    }
    return "rotctld-track" if is_tracking else key


def set_rotor_net_streaming_target(az_deg, el_deg, client_name="rotctld-track"):
    antenna_az = norm_az(float(az_deg))
    antenna_el = clamp(float(el_deg), 0.0, 90.0)
    with state_lock:
        entering_stream = rotor_net.get("lastClient") != client_name
        if entering_stream:
            rotor_net["commandSeq"] = int(rotor_net.get("commandSeq", 0)) + 1
            rotor_net["lastSendMs"] = 0.0
            rotor_net["lastCmd"] = "PTZ STOP"
        rotor_net["targetAz"] = antenna_az
        rotor_net["targetEl"] = antenna_el
        rotor_net["active"] = True
        rotor_net["busy"] = False
        rotor_net["lastClient"] = client_name
        rotor_net["lastUpdateMs"] = time.time()
        latest["rotorSource"] = "NET"
        latest["status"] = f"ROTCTLD TRACK TARGET AZ {antenna_az:.1f} EL {antenna_el:.1f}"
    if entering_stream:
        rotor_net_cancel_event.set()


def current_rotor_position():
    with state_lock:
        return float(latest.get("az", 0.0)), float(latest.get("el", 0.0))


def rotctl_pos_lines():
    az, el = current_rotor_position()
    return f"{az:.1f}\n{el:.1f}\n"


def easycomm_axis_line(axis):
    az, el = current_rotor_position()
    if axis == "AZ":
        return f"AZ{az:.1f}\n"
    return f"EL{el:.1f}\n"


def easycomm_pos_line():
    az, el = current_rotor_position()
    return f"AZ{az:.1f} EL{el:.1f}\n"


def parse_rotctl_two_floats(line):
    parts = str(line or "").strip().split()
    if len(parts) < 3:
        return None
    try:
        return float(parts[1]), float(parts[2])
    except ValueError:
        return None


def parse_easycomm_value(line, axis):
    text = str(line or "").strip()
    if not text.upper().startswith(axis):
        return None
    payload = text[2:].lstrip(" \t=")
    if not payload:
        return None
    try:
        return float(payload)
    except ValueError:
        return None


def run_rotor_net_track_tick():
    with state_lock:
        sat_tracking_enabled = bool(sat_lock.get("enabled"))
        sat_tracking_name = str(sat_lock.get("sat") or "").strip()

    if sat_tracking_enabled and sat_tracking_name:
        catalog_item = find_amateur_catalog_item(sat_tracking_name)
        if catalog_item:
            norad = catalog_item.get("norad")
            cached_tle = _tle_cache.get(str(norad)) if norad else None
            catalog_ts = float(_amateur_catalog_cache.get("ts") or 0.0)
            tle_is_stale = False
            if cached_tle:
                tle_is_stale = (time.time() - float(cached_tle.get("ts", 0.0))) > 48 * 3600
            else:
                tle_is_stale = catalog_ts == 0.0 or (time.time() - catalog_ts) > 48 * 3600
            if tle_is_stale:
                with state_lock:
                    sat_lock["enabled"] = False
                    rotor_net["active"] = False
                    latest["status"] = f"TRACK HALT {sat_tracking_name} TLE TOO OLD"
                return

        actual_now_result = compute_current_az_el(sat_tracking_name)
        if actual_now_result is None:
            with state_lock:
                last_good_az = sat_lock.get("lastGoodAz")
                last_good_el = sat_lock.get("lastGoodEl")
                last_good_ts = float(sat_lock.get("lastGoodTs") or 0.0)
            if last_good_az is not None and last_good_el is not None and time.time() - last_good_ts <= 5.0:
                set_rotor_net_target(last_good_az, last_good_el, "sat-lock")
                with state_lock:
                    latest["status"] = f"TRACK HOLD {sat_tracking_name} LAST GOOD TARGET"
                return
            with state_lock:
                latest["status"] = f"TRACK WAIT {sat_tracking_name} NO DATA"
            return
        actual_az_now, actual_el_now, _actual_range_km = actual_now_result

        if actual_el_now < SAT_TRACK_END_EL_DEG:
            should_stop = False
            still_same_sat_locked = False
            with state_lock:
                latest["status"] = f"PASS COMPLETE {sat_tracking_name} BELOW {SAT_TRACK_END_EL_DEG:.1f} DEG - HOLDING"
                if rotor_net.get("lastClient") == "sat-lock" and rotor_net.get("active"):
                    rotor_net["active"] = False
                    rotor_net["targetAz"] = None
                    rotor_net["targetEl"] = None
                    rotor_net["lastCmd"] = "PTZ STOP"
                    rotor_net["lastSendMs"] = 0.0
                    should_stop = True
                still_same_sat_locked = (
                    bool(sat_lock.get("enabled"))
                    and str(sat_lock.get("sat") or "").strip() == sat_tracking_name
                )
            if should_stop:
                send_ptz_command("stop", "", "")
            if still_same_sat_locked:
                with state_lock:
                    sat_lock["enabled"] = False
                    clear_sat_lock_last_good_locked()
                    latest["status"] = f"PASS COMPLETE {sat_tracking_name} BELOW {SAT_TRACK_END_EL_DEG:.1f} DEG - HOLDING"
                send_civ_after_pass()
            return

        # Use the satellite's actual current position \u2014 not lead-adjusted \u2014
        # as the basis for the commanded target. Near peak elevation, azimuth
        # can change very fast in real life; projecting too far ahead in that
        # situation sends the rotor to a wildly different bearing than where
        # the satellite currently is, producing large, jarring swings instead
        # of smooth tracking.
        lead_result = compute_current_az_el(sat_tracking_name, lead_seconds=ROTOR_TRACK_LEAD_SECONDS)
        lead_az, lead_el, _range_km = lead_result if lead_result is not None else actual_now_result
        if not (
            math.isfinite(float(actual_az_now))
            and math.isfinite(float(actual_el_now))
            and math.isfinite(float(lead_az))
            and math.isfinite(float(lead_el))
        ):
            with state_lock:
                sat_lock["enabled"] = False
                rotor_net["active"] = False
                rotor_net["targetAz"] = None
                rotor_net["targetEl"] = None
                rotor_net["lastCmd"] = "PTZ STOP"
                rotor_net["lastSendMs"] = 0.0
                latest["status"] = f"TRACK HALT {sat_tracking_name} BAD SKYFIELD TARGET"
            send_ptz_command("stop", "", "")
            return
        # Cap how far a single tick is allowed to move the commanded azimuth
        # away from the satellite's actual current bearing, regardless of
        # what the lead-time projection suggests. This keeps fast, near-overhead
        # passes from producing a multi-degree instantaneous jump.
        MAX_LEAD_AZ_DELTA_DEG = 4.0
        az_lead_delta = normalize_angle_delta(lead_az - actual_az_now)
        if abs(az_lead_delta) > MAX_LEAD_AZ_DELTA_DEG:
            az_lead_delta = MAX_LEAD_AZ_DELTA_DEG if az_lead_delta > 0 else -MAX_LEAD_AZ_DELTA_DEG
        az_deg = norm_az(actual_az_now + az_lead_delta)
        el_deg = max(SAT_TRACK_END_EL_DEG, lead_el)
        if actual_el_now >= SAT_TRACK_ZENITH_HOLD_EL_DEG:
            with state_lock:
                held_az = sat_lock.get("lastGoodAz")
            if held_az is not None:
                az_deg = norm_az(float(held_az))
        if el_deg < 0.0 or el_deg > 90.0:
            with state_lock:
                sat_lock["enabled"] = False
                rotor_net["active"] = False
                rotor_net["targetAz"] = None
                rotor_net["targetEl"] = None
                rotor_net["lastCmd"] = "PTZ STOP"
                rotor_net["lastSendMs"] = 0.0
                latest["status"] = f"TRACK HALT {sat_tracking_name} UNSAFE EL {el_deg:.1f}"
            send_ptz_command("stop", "", "")
            return

        now = time.time()
        with state_lock:
            prev_cmd_az = sat_lock.get("commandAz")
            prev_cmd_el = sat_lock.get("commandEl")
            prev_cmd_ts = float(sat_lock.get("commandTs") or 0.0)
        if prev_cmd_az is None or prev_cmd_el is None or prev_cmd_ts <= 0.0:
            command_az = az_deg
            command_el = el_deg
        else:
            dt = max(0.05, min(2.0, now - prev_cmd_ts))
            command_az = advance_az_toward(
                float(prev_cmd_az),
                az_deg,
                ROTOR_TRACK_LEAD_AZ_RATE_DEG_PER_SEC * dt,
            )
            command_el = advance_linear_toward(
                float(prev_cmd_el),
                el_deg,
                ROTOR_TRACK_LEAD_EL_RATE_DEG_PER_SEC * dt,
            )
            command_el = max(SAT_TRACK_END_EL_DEG, min(90.0, command_el))

        with state_lock:
            active_target_az = rotor_net.get("targetAz")
            active_target_el = rotor_net.get("targetEl")
        target_delta_az = 999.0 if active_target_az is None else abs(normalize_angle_delta(command_az - float(active_target_az)))
        target_delta_el = 999.0 if active_target_el is None else abs(command_el - float(active_target_el))
        if target_delta_az >= ROTOR_TRACK_MIN_CMD_AZ_DELTA_DEG or target_delta_el >= ROTOR_TRACK_MIN_CMD_EL_DELTA_DEG:
            set_rotor_net_target(command_az, command_el, "sat-lock")
        with state_lock:
            sat_lock["commandAz"] = float(command_az)
            sat_lock["commandEl"] = float(command_el)
            sat_lock["commandTs"] = now
            sat_lock["lastGoodAz"] = float(command_az)
            sat_lock["lastGoodEl"] = float(command_el)
            sat_lock["lastGoodTs"] = time.time()
        with state_lock:
            latest["status"] = f"TRACKING {sat_tracking_name} lead az={command_az:.1f} el={command_el:.1f}"
        if civ_router_config.get("enabled"):
            range_rate_km_s = compute_range_rate_km_s(sat_tracking_name)
            if range_rate_km_s is not None:
                send_civ_doppler(sat_tracking_name, range_rate_km_s)

    if not rotor_net["enabled"] or not rotor_net["active"] or rotor_net["busy"]:
        return

    rotor_net["busy"] = True
    try:
        with state_lock:
            target_az = rotor_net["targetAz"]
            target_el = rotor_net["targetEl"]
            current_az = float(latest.get("az", 0.0))
            current_el = float(latest.get("el", 0.0))
            last_cmd = str(rotor_net.get("lastCmd", "PTZ STOP"))
            last_send_ms = float(rotor_net.get("lastSendMs", 0.0))
            last_client = str(rotor_net.get("lastClient", ""))
            command_seq = int(rotor_net.get("commandSeq", 0))

        if target_az is None or target_el is None:
            return

        if not active_rotator_is_internal():
            now = time.time()
            should_send = last_client != "sat-lock" or (now - last_send_ms) >= max(0.15, TRACK_UPDATE_MS / 1000.0)
            if last_client == "ui-home":
                should_send = True
            if should_send:
                ok = send_position_command(target_az, target_el)
                with state_lock:
                    rotor_net["lastCmd"] = f"EXT {float(target_az):.1f} {float(target_el):.1f}" if ok else rotor_net["lastCmd"]
                    rotor_net["lastSendMs"] = now if ok else rotor_net["lastSendMs"]
                    if last_client == "ui-home" and ok:
                        rotor_net["active"] = False
                        latest["status"] = "EXT ROTATOR PARK SENT"
            return

        if last_client == "sat-lock":
            now = time.time()
            should_send = last_cmd != "ABS TRACK" or (now - last_send_ms) >= max(0.15, TRACK_UPDATE_MS / 1000.0)
            if should_send:
                display_target_el = float(target_el)
                if not math.isfinite(display_target_el) or display_target_el < SAT_TRACK_END_EL_DEG or display_target_el > 90.0:
                    with state_lock:
                        sat_lock["enabled"] = False
                        rotor_net["active"] = False
                        rotor_net["targetAz"] = None
                        rotor_net["targetEl"] = None
                        rotor_net["lastCmd"] = "PTZ STOP"
                        rotor_net["lastSendMs"] = 0.0
                        latest["status"] = f"TRACK HALT UNSAFE EL {display_target_el:.1f}"
                    send_ptz_command("stop", "", "")
                    return
                with state_lock:
                    acquire_active = bool(sat_lock.get("acquireActive", False))
                    acquire_started_ts = float(sat_lock.get("acquireStartedTs") or 0.0)
                if acquire_active and last_cmd == "ABS TRACK":
                    acquire_age = now - acquire_started_ts if acquire_started_ts > 0.0 else 0.0
                    acquire_az_error = abs(normalize_angle_delta(float(target_az) - float(current_az)))
                    acquire_el_error = abs(float(target_el) - float(current_el))
                    acquire_close = (
                        acquire_az_error <= ROTOR_TRACK_ACQUIRE_AZ_WINDOW_DEG
                        and acquire_el_error <= ROTOR_TRACK_ACQUIRE_EL_WINDOW_DEG
                    )
                    acquire_expired = acquire_age >= ROTOR_TRACK_ACQUIRE_MAX_HOLD_SEC
                    if not acquire_close and not acquire_expired:
                        with state_lock:
                            latest["status"] = (
                                f"TRACK ACQUIRE AZERR={acquire_az_error:.1f} "
                                f"ELERR={acquire_el_error:.1f}"
                            )
                        return
                    with state_lock:
                        sat_lock["acquireActive"] = False
                        sat_lock["acquireStartedTs"] = 0.0
                ok = send_display_absolute_position(target_az, target_el)
                with state_lock:
                    rotor_net["lastCmd"] = "ABS TRACK" if ok else rotor_net["lastCmd"]
                    rotor_net["lastSendMs"] = now if ok else rotor_net["lastSendMs"]
                    latest["status"] = (
                        f"ROTOR ABS TRACK AZ {float(target_az):.1f} "
                        f"EL {float(target_el):.1f}"
                    ) if ok else "ROTOR ABS TRACK SEND FAIL"
            return

        if last_client == "rotctld-track":
            now = time.time()
            if now - float(rotor_net.get("lastUpdateMs") or 0.0) > ROTCTLD_TRACK_STALE_SEC:
                ok, _ = send_ptz_command("stop", "", "")
                with state_lock:
                    rotor_net["active"] = False
                    rotor_net["lastCmd"] = "PTZ STOP"
                    rotor_net["lastSendMs"] = now if ok else rotor_net["lastSendMs"]
                    latest["status"] = "ROTCTLD TRACK STALE HOLD"
                return

            display_target_el = float(target_el)
            if not math.isfinite(display_target_el) or display_target_el < 0.0 or display_target_el > 90.0:
                ok, _ = send_ptz_command("stop", "", "")
                with state_lock:
                    rotor_net["active"] = False
                    rotor_net["lastCmd"] = "PTZ STOP"
                    rotor_net["lastSendMs"] = now if ok else rotor_net["lastSendMs"]
                    latest["status"] = f"ROTCTLD TRACK HALT EL {display_target_el:.1f}"
                return

            should_send = last_cmd != "ABS ROTCTLD TRACK" or (now - last_send_ms) >= max(
                0.15, ROTCTLD_TRACK_SEND_MS / 1000.0
            )
            if should_send:
                ok = send_display_absolute_position(target_az, target_el)
                with state_lock:
                    rotor_net["lastCmd"] = "ABS ROTCTLD TRACK" if ok else rotor_net["lastCmd"]
                    rotor_net["lastSendMs"] = now if ok else rotor_net["lastSendMs"]
                    latest["status"] = (
                        f"ROTCTLD ABS TRACK AZ {float(target_az):.1f} "
                        f"EL {float(target_el):.1f}"
                    ) if ok else "ROTCTLD ABS TRACK SEND FAIL"
            return

        is_home_target = last_client == "ui-home"

        if is_home_target:
            with state_lock:
                query_valid = bool(rotor_live.get("queryValid", False))
                if query_valid:
                    current_az, current_el = referenced_rotor_locked()
                else:
                    current_az, current_el = float(latest.get("az", 0.0)), float(latest.get("el", 0.0))

            if query_valid:
                d_az_home = home_az_error_from_reference(current_az)
                d_el_home = float(target_el) - float(current_el)
                if abs(d_az_home) <= ROTOR_HOME_FINAL_AZ_WINDOW_DEG and abs(d_el_home) <= ROTOR_HOME_FINAL_EL_WINDOW_DEG:
                    with state_lock:
                        hold_started = float(rotor_net.get("homeHoldTs") or 0.0)
                        if hold_started <= 0.0:
                            rotor_net["homeHoldTs"] = time.time()
                            latest["status"] = "ROTOR HOME VERIFY"
                            return
                        home_stable = (time.time() - hold_started) >= 1.5
                    if home_stable:
                        ok, _ = send_ptz_command("stop", "", "")
                        with state_lock:
                            rotor_net["active"] = False
                            rotor_net["targetAz"] = 0.0
                            rotor_net["targetEl"] = target_el
                            rotor_net["lastCmd"] = "PTZ STOP"
                            rotor_net["lastSendMs"] = time.time() if ok else rotor_net["lastSendMs"]
                            rotor_net["homePrevAzErr"] = None
                            rotor_net["homePrevElErr"] = None
                            rotor_net["homeWorseTicks"] = 0
                            rotor_net["homeHoldTs"] = time.time()
                            latest["status"] = "ROTOR HOME HOLD"
                        return
                else:
                    with state_lock:
                        rotor_net["homeHoldTs"] = 0.0

            now = time.time()
            should_send = last_cmd != "ABS HOME" or (now - last_send_ms) >= 1.0
            if should_send:
                ok = send_display_absolute_position(target_az, target_el)
                with state_lock:
                    rotor_net["active"] = True
                    rotor_net["targetAz"] = float(target_az)
                    rotor_net["targetEl"] = float(target_el)
                    rotor_net["lastCmd"] = "ABS HOME" if ok else rotor_net["lastCmd"]
                    rotor_net["lastSendMs"] = now if ok else rotor_net["lastSendMs"]
                    rotor_net["homePrevAzErr"] = None
                    rotor_net["homePrevElErr"] = None
                    rotor_net["homeWorseTicks"] = 0
                    latest["status"] = (
                        f"ROTOR ABS HOME AZ {float(target_az):.1f} EL {float(target_el):.1f}"
                    ) if ok else "ROTOR ABS HOME SEND FAIL"
            return

        if ROTOR_PLANNED_MOVE_ENABLE and last_client and last_client not in ("sat-lock", "ui-home"):
            run_planned_rotctld_move(target_az, target_el, command_seq)
            return

        if is_home_target:
            with state_lock:
                query_valid = bool(rotor_live.get("queryValid", False))
                if query_valid:
                    current_az, current_el = referenced_rotor_locked()
            if not query_valid:
                now = time.time()
                if last_cmd != "PTZ STOP" or (now - last_send_ms) >= 1.0:
                    ok, _ = send_ptz_command("stop", "", "")
                    with state_lock:
                        rotor_net["lastCmd"] = "PTZ STOP"
                        rotor_net["lastSendMs"] = now if ok else rotor_net["lastSendMs"]
                with state_lock:
                    latest["status"] = "ROTOR HOME WAIT QUERY"
                return
            d_az = -home_az_error_from_reference(current_az)
            d_el = float(target_el) - float(current_el)
        else:
            d_az = normalize_angle_delta(float(target_az) - float(current_az))
            d_el = float(target_el) - float(current_el)

        az_tol = 1.0
        el_tol = 1.0

        if is_home_target and abs(d_az) <= ROTOR_HOME_AZ_DEADBAND_DEG and abs(d_el) <= ROTOR_HOME_EL_DEADBAND_DEG:
            ok, _ = send_ptz_command("stop", "", "")
            with state_lock:
                rotor_net["active"] = False
                rotor_net["targetAz"] = 0.0
                rotor_net["targetEl"] = target_el
                rotor_net["lastCmd"] = "PTZ STOP"
                rotor_net["lastSendMs"] = time.time() if ok else rotor_net["lastSendMs"]
                rotor_net["homePrevAzErr"] = None
                rotor_net["homePrevElErr"] = None
                rotor_net["homeWorseTicks"] = 0
                rotor_net["homeHoldTs"] = time.time()
                latest["status"] = "ROTOR HOME HOLD"
            return

        if is_home_target:
            with state_lock:
                rotor_net["homePrevAzErr"] = abs(d_az)
                rotor_net["homePrevElErr"] = abs(d_el)
                rotor_net["homeWorseTicks"] = 0

        desired_cmd = "PTZ STOP"
        action = "stop"
        axis = ""
        direction = ""

        TRACK_PULSE_WINDOW_DEG = 8.0
        PULSE_MIN_MS = 12.0
        PULSE_SETTLE_SECONDS = 0.65

        in_final_az_window = abs(d_az) <= ROTOR_HOME_FINAL_AZ_WINDOW_DEG
        in_final_el_window = abs(d_el) <= ROTOR_HOME_FINAL_EL_WINDOW_DEG
        pulse_window_az = ROTOR_HOME_FINAL_AZ_WINDOW_DEG
        pulse_window_el = ROTOR_HOME_FINAL_EL_WINDOW_DEG

        def scaled_pulse_ms(error_deg, window_deg):
            # Linearly shrink the pulse as the error shrinks within the window,
            # so corrections near the target are gentle instead of a fixed-size
            # nudge that can overshoot a small remaining error.
            fraction = max(0.0, min(1.0, abs(error_deg) / max(window_deg, 0.001)))
            return max(PULSE_MIN_MS, ROTOR_HOME_PULSE_MS * fraction)

        if abs(d_az) > az_tol and in_final_az_window:
            pulse_axis = "az"
            pulse_dir = "neg" if d_az < 0.0 else "pos"
            pulse_ms = scaled_pulse_ms(d_az, pulse_window_az)
            ok, sent_cmd = send_ptz_pulse("jog", pulse_axis, pulse_dir, pulse_ms=pulse_ms)
            if ok:
                with state_lock:
                    rotor_net["lastCmd"] = "PTZ STOP"
                    rotor_net["lastSendMs"] = time.time()
                    latest["status"] = f"ROTOR PULSE {sent_cmd} ({pulse_ms:.0f}ms)"
            time.sleep(PULSE_SETTLE_SECONDS)
            return
        if abs(d_az) <= az_tol and abs(d_el) > el_tol and in_final_el_window:
            pulse_axis = "el"
            pulse_dir = "neg" if d_el < 0.0 else "pos"
            pulse_ms = scaled_pulse_ms(d_el, pulse_window_el)
            ok, sent_cmd = send_ptz_pulse("jog", pulse_axis, pulse_dir, pulse_ms=pulse_ms)
            if ok:
                with state_lock:
                    rotor_net["lastCmd"] = "PTZ STOP"
                    rotor_net["lastSendMs"] = time.time()
                    latest["status"] = f"ROTOR PULSE {sent_cmd} ({pulse_ms:.0f}ms)"
            time.sleep(PULSE_SETTLE_SECONDS)
            return

        if abs(d_az) > az_tol:
            desired_cmd = "PTZ PAN -" if d_az < 0.0 else "PTZ PAN +"
            action = "jog"
            axis = "az"
            direction = "neg" if d_az < 0.0 else "pos"
        elif abs(d_el) > el_tol:
            desired_cmd = "PTZ TILT -" if d_el < 0.0 else "PTZ TILT +"
            action = "jog"
            axis = "el"
            direction = "neg" if d_el < 0.0 else "pos"
            desired_cmd = "PTZ TILT -" if direction == "neg" else "PTZ TILT +"
        else:
            ok, _ = send_ptz_command("stop", "", "")
            with state_lock:
                rotor_net["active"] = False
                rotor_net["lastCmd"] = "PTZ STOP"
                rotor_net["lastSendMs"] = time.time() if ok else rotor_net["lastSendMs"]
                rotor_net["homePrevAzErr"] = None
                rotor_net["homePrevElErr"] = None
                rotor_net["homeWorseTicks"] = 0
                latest["status"] = "ROTOR NET HOLD"
            return

        now = time.time()
        should_send = desired_cmd != last_cmd or (now - last_send_ms) >= 2.0
        if should_send:
            ok, sent_cmd = send_ptz_command(action, axis, direction)
            if ok:
                with state_lock:
                    rotor_net["lastCmd"] = sent_cmd
                    rotor_net["lastSendMs"] = now
                    latest["status"] = f"ROTOR NET {sent_cmd}"
    finally:
        rotor_net["busy"] = False


class RotctldHandler(socketserver.StreamRequestHandler):
    def handle(self):
        while True:
            raw = self.rfile.readline()
            if not raw:
                return

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            if line.startswith("+"):
                line = line[1:].strip()

            line_upper = line.upper()
            parts = line.split()
            raw_cmd = parts[0] if parts else ""
            cmd = raw_cmd.lower()

            if cmd in ("q", "\\quit", "quit"):
                return

            if raw_cmd == "p" or cmd in ("\\get_pos", "get_pos"):
                self.wfile.write(rotctl_pos_lines().encode("utf-8"))
                self.wfile.flush()
                continue

            if cmd in ("s", "\\stop", "stop"):
                rotor_net_cancel_event.set()
                with state_lock:
                    rotor_net["active"] = False
                send_ptz_command("stop", "", "")
                self.wfile.write(b"RPRT 0\n")
                self.wfile.flush()
                continue

            if cmd in ("_", "\\get_info", "get_info"):
                self.wfile.write(b"W6PS SatRotor rotctld bridge v1\n")
                self.wfile.flush()
                continue

            if raw_cmd == "P" or cmd in ("\\set_pos", "set_pos"):
                values = parse_rotctl_two_floats(line)
                if values is None:
                    self.wfile.write(b"RPRT -1\n")
                    self.wfile.flush()
                    continue
                az, el = values
                client = f"{self.client_address[0]}:{self.client_address[1]}"
                client_mode = classify_rotctld_set_pos_client(client)
                if client_mode == "rotctld-track":
                    set_rotor_net_streaming_target(az, el, client_mode)
                else:
                    set_rotor_net_target(az, el, client)
                self.wfile.write(b"RPRT 0\n")
                self.wfile.flush()
                continue

            if line_upper in ("AZEL", "AZ EL"):
                self.wfile.write(easycomm_pos_line().encode("utf-8"))
                self.wfile.flush()
                continue

            if cmd in ("az", "az?"):
                self.wfile.write(easycomm_axis_line("AZ").encode("utf-8"))
                self.wfile.flush()
                continue

            if cmd in ("el", "el?"):
                self.wfile.write(easycomm_axis_line("EL").encode("utf-8"))
                self.wfile.flush()
                continue

            az_value = parse_easycomm_value(line, "AZ")
            if az_value is not None:
                _, current_el = current_rotor_position()
                client = f"{self.client_address[0]}:{self.client_address[1]}"
                set_rotor_net_target(az_value, current_el, client)
                self.wfile.write(easycomm_axis_line("AZ").encode("utf-8"))
                self.wfile.flush()
                continue

            el_value = parse_easycomm_value(line, "EL")
            if el_value is not None:
                current_az, _ = current_rotor_position()
                client = f"{self.client_address[0]}:{self.client_address[1]}"
                set_rotor_net_target(current_az, el_value, client)
                self.wfile.write(easycomm_axis_line("EL").encode("utf-8"))
                self.wfile.flush()
                continue

            self.wfile.write(b"RPRT -1\n")
            self.wfile.flush()


class RotctldServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True


def rotctld_listener():
    if not ROTCTLD_ENABLE:
        with state_lock:
            latest["status"] = "ROTCTLD DISABLED"
        return

    with RotctldServer((ROTCTLD_HOST, ROTCTLD_PORT), RotctldHandler) as server:
        with state_lock:
            latest["status"] = f"ROTCTLD {ROTCTLD_HOST}:{ROTCTLD_PORT}"
        server.serve_forever()


def rotor_net_loop():
    while True:
        try:
            run_rotor_net_track_tick()
        except Exception as exc:
            import traceback
            print('TRACK TICK ERROR:', exc)
            traceback.print_exc()
        time.sleep(0.15)


def json_safe(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def send_json(self, payload, status=HTTPStatus.OK):
        data = json.dumps(json_safe(payload), allow_nan=False).encode("utf-8")
        try:
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            return

    def send_file(self, path, content_type):
        data = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def send_html(self, html, status=HTTPStatus.OK):
        data = html.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def send_text(self, text, content_type="text/plain; charset=utf-8", status=HTTPStatus.OK):
        data = text.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def read_json_body(self):
        try:
            length = int(self.headers.get("Content-Length", "0") or "0")
        except Exception:
            length = 0
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        if not raw:
            return {}
        try:
            payload = json.loads(raw.decode("utf-8"))
        except Exception:
            raise ValueError("bad_json")
        if not isinstance(payload, dict):
            raise ValueError("bad_json")
        return payload

    def do_GET(self):
        refresh_host_wifi()
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/index.html"):
            return self.send_file(INDEX_PATH, "text/html; charset=utf-8")
        if parsed.path == "/webview":
            qs = parse_qs(parsed.query)
            target = unquote(qs.get("target", [""])[0]).strip()
            title = (qs.get("title", ["External View"])[0] or "External View").strip()
            if not target.startswith("https://"):
                return self.send_error(HTTPStatus.BAD_REQUEST)
            return self.send_html(WEBVIEW_HTML.format(target=target, title=title))
        if parsed.path == "/three.min.js":
            return self.send_file(THREE_PATH, "application/javascript; charset=utf-8")
        if parsed.path == "/earth-texture-data.js":
            return self.send_text(
                f"window.EARTH_TEXTURE_DATA_URL = {json.dumps(EARTH_TEXTURE_DATA_URL)};\n",
                "application/javascript; charset=utf-8",
            )
        if parsed.path == "/earth-blue-marble-2048.png":
            return self.send_file(EARTH_TEXTURE_PATH, "image/png")
        if parsed.path == "/world-map.geojson":
            return self.send_file(WORLD_MAP_PATH, "application/geo+json; charset=utf-8")
        if parsed.path == "/admin1-lines.geojson":
            return self.send_file(ADMIN1_LINES_PATH, "application/geo+json; charset=utf-8")
        if parsed.path == "/api/diagnostics":
            return self.send_json(pi_diagnostics_payload())
        if parsed.path == "/api/health":
            return self.send_json({"ok": True, "service": "satrotor-bridge", "time": time.time()})
        if parsed.path == "/api/state-debug":
            with state_lock:
                limit_relative_az = float(current_limit_relative_az_locked())
                payload = dict(latest)
                payload.update(
                    {
                        "rawAz": round(float(rotor_live.get("queryAz", 0.0)), 1),
                        "rawEl": round(float(rotor_live.get("queryEl", 0.0)), 1),
                        "fusedAz": round(float(rotor_live.get("az", 0.0)), 1),
                        "rawAzTotal": round(float(rotor_live.get("azTotal", 0.0)), 1),
                        "rawAzUnwrapped": round(float(rotor_live.get("azUnwrapped", 0.0)), 1),
                        "queryState": str(rotor_live.get("queryState", "")),
                        "fusionUsingEstimate": bool(rotor_live.get("fusionUsingEstimate", False)),
                        "queryPanOk": bool(rotor_live.get("queryPanOk", False)),
                        "queryTiltOk": bool(rotor_live.get("queryTiltOk", False)),
                        "queryPanRaw": int(rotor_live.get("queryPanRaw", 0)),
                        "queryTiltRaw": int(rotor_live.get("queryTiltRaw", 0)),
                        "zeroEnabled": bool(rotor_zero.get("enabled")),
                        "zeroRawAz": round(float(rotor_zero.get("az", 0.0)), 1),
                        "zeroRawEl": round(float(rotor_zero.get("el", 0.0)), 1),
                        "zeroRawAzUnwrapped": round(float(rotor_zero.get("azUnwrapped", 0.0)), 1),
                        "zeroPanRaw": None if rotor_zero.get("panRaw") is None else int(current_zero_pan_raw_locked()),
                        "displayAzFromPanRaw": None if raw_referenced_pan_az_locked() is None else round(float(raw_referenced_pan_az_locked()), 1),
                        "displayAzCorrected": None if referenced_pan_az_locked() is None else round(float(referenced_pan_az_locked()), 1),
                        "readoutAzCorrection": None if raw_referenced_pan_az_locked() is None else round(float(readout_az_correction(raw_referenced_pan_az_locked())), 1),
                        "limitRelativeAz": round(limit_relative_az, 1),
                        "azLimitWarning": False,
                        "azLimitHard": False,
                    }
                )
            return self.send_json(payload)
        if parsed.path == "/api/rotor/tcp-info":
            with state_lock:
                wf = str(latest.get("wifi", ""))
            host_match = re.search(r"(\d+\.\d+\.\d+\.\d+)$", wf)
            host = host_match.group(1) if host_match else ""
            return self.send_json({
                "ok": True,
                "enabled": ROTCTLD_ENABLE,
                "protocol": "hamlib-rotctld",
                "host": host,
                "port": ROTCTLD_PORT,
                "status": "LISTENING" if ROTCTLD_ENABLE else "DISABLED",
            })
        if parsed.path == "/api/rotator/link-config":
            with state_lock:
                return self.send_json(current_rotator_link_payload())
        if parsed.path == "/api/civ/config":
            with state_lock:
                return self.send_json(current_civ_router_payload())
        if parsed.path == "/api/sat/options":
            options = [item["name"] for item in amateur_catalog()]
            if not options:
                options = list(SUPPORTED_SATS)
            return self.send_json({"ok": True, "options": options})
        if parsed.path == "/api/sat/live":
            with state_lock:
                requested = parse_qs(parsed.query).get("sat", [sat_lock.get("sat") or latest.get("sat", "AO-91")])[0]
                base_tx = latest.get("tx", 0.0)
                base_rx = latest.get("rx", 0.0)
                base_tx_mode = latest.get("txMode", "FM")
                base_rx_mode = latest.get("rxMode", "FM")
                base_gps = latest.get("gps", "NO FIX")
                base_sats = latest.get("sats", 0)
                base_gps_lat = latest.get("gpsLat", "--")
                base_gps_lon = latest.get("gpsLon", "--")
                base_gps_alt_ft = latest.get("gpsAltFt", "--")
                base_status = latest.get("status", "--")
                lock_enabled = bool(sat_lock.get("enabled"))
                lock_sat = str(sat_lock.get("sat") or "")

            selected_sat = lock_sat if lock_enabled and lock_sat else requested
            selected_item = find_amateur_catalog_item(selected_sat)
            if selected_item:
                selected_sat = selected_item["name"]
            else:
                selected_sat = normalize_supported_sat(selected_sat)

            norad = satnogs_norad_for_name(selected_sat)
            has_cached_tle = bool(norad and _tle_cache.get(str(norad)))
            if selected_item or has_cached_tle or norad:
                try:
                    selected_payload = skyfield_current_payload(selected_sat)
                except Exception as exc:
                    selected_payload = {
                        "ok": False,
                        "error": str(exc),
                        "sat": selected_sat,
                    }
            else:
                selected_payload = {
                    "ok": False,
                    "error": "satellite_cache_empty",
                    "sat": selected_sat,
                }
            if not isinstance(selected_payload, dict) or not selected_payload.get("ok"):
                selected_data = {
                    "selectedSat": selected_sat,
                    "trackedSat": selected_sat,
                    "timestamp": int(time.time() * 1000),
                    "observerLat": None,
                    "observerLon": None,
                    "observerAltFt": 0.0,
                    "selectedAz": None,
                    "selectedEl": None,
                    "selectedRangeKm": None,
                    "selectedRangeMi": None,
                    "selectedLat": None,
                    "selectedLon": None,
                    "selectedAltKm": None,
                    "inView": False,
                    "nextPass": [],
                    "orbitPath": [],
                    "tleAge": "--",
                    "telemetry": f"{base_status} | SAT DATA UNAVAILABLE | GPS {base_gps}",
                    "satLiveError": selected_payload.get("error", "skyfield unavailable") if isinstance(selected_payload, dict) else "skyfield unavailable",
                }
            else:
                selected_data = selected_payload["data"]

            observer_lat = selected_data.get("observerLat")
            observer_lon = selected_data.get("observerLon")
            observer_alt_ft = selected_data.get("observerAltFt", 0.0)
            if observer_lat is None or observer_lon is None:
                try:
                    observer_lat = float(base_gps_lat)
                    observer_lon = float(base_gps_lon)
                    observer_alt_ft = float(base_gps_alt_ft) if base_gps_alt_ft != "--" else 0.0
                    selected_data["observerLat"] = observer_lat
                    selected_data["observerLon"] = observer_lon
                    selected_data["observerAltFt"] = observer_alt_ft
                except (TypeError, ValueError):
                    pass
            in_view_rows = []
            if observer_lat is not None and observer_lon is not None:
                try:
                    in_view_rows = visible_amateur_sat_rows(observer_lat, observer_lon, observer_alt_ft)
                except Exception:
                    in_view_rows = list(_visible_sats_cache.get("rows") or [])
            selected_data.update(
                {
                    "lockState": "TRACKING" if float(selected_data.get("selectedEl") or 0.0) >= SAT_TRACK_MIN_EL_DEG else "STANDBY",
                    "lockEnabled": lock_enabled,
                    "lockSat": lock_sat,
                    "lockDetail": f"LOCKED {lock_sat}" if lock_enabled and lock_sat else "",
                    "telemetry": f"{base_status} | SKYFIELD {'TRACKABLE' if float(selected_data.get('selectedEl') or 0.0) >= SAT_TRACK_MIN_EL_DEG else 'BELOW MIN EL'} | GPS {base_gps}",
                    "inView": in_view_rows,
                    "tx": base_tx,
                    "rx": base_rx,
                    "txMode": base_tx_mode,
                    "rxMode": base_rx_mode,
                    "gps": base_gps,
                    "sats": base_sats,
                    "gpsLat": base_gps_lat,
                    "gpsLon": base_gps_lon,
                    "gpsAltFt": base_gps_alt_ft,
                }
            )
            return self.send_json({"ok": True, "data": selected_data})
        if parsed.path == "/api/sat/skyfield-live":
            sat_name = parse_qs(parsed.query).get("sat", [latest.get("sat", "AO-91")])[0]
            try:
                return self.send_json(skyfield_current_payload(sat_name))
            except Exception as exc:
                return self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_GATEWAY)
        if parsed.path == "/api/satnogs/current":
            sat_name = parse_qs(parsed.query).get("sat", [latest.get("sat", "AO-91")])[0]
            try:
                return self.send_json(satnogs_current_payload(sat_name))
            except Exception as exc:
                return self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_GATEWAY)
        if parsed.path == "/api/satnogs/summary":
            try:
                rows = _fetch_json_url("https://network.satnogs.org/api/observations/")
                return self.send_json({"ok": True, "observationsToday": len(rows) if isinstance(rows, list) else 0})
            except Exception as exc:
                return self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_GATEWAY)
        if parsed.path == "/api/spaceweather/summary":
            try:
                return self.send_json(space_weather_summary())
            except Exception as exc:
                return self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_GATEWAY)
        if parsed.path == "/api/weather/summary":
            try:
                payload = weather_summary()
                return self.send_json(payload, status=HTTPStatus.OK if payload.get("ok") else HTTPStatus.BAD_GATEWAY)
            except Exception as exc:
                return self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_GATEWAY)
        if parsed.path == "/api/weather/at":
            try:
                weather_qs = parse_qs(parsed.query)
                lat = float(weather_qs.get("lat", [None])[0])
                lon = float(weather_qs.get("lon", [None])[0])
            except (TypeError, ValueError):
                return self.send_json({"ok": False, "error": "bad_coordinates"}, status=HTTPStatus.BAD_REQUEST)
            try:
                payload = weather_summary_for_location(lat, lon, include_place=True)
                return self.send_json(payload, status=HTTPStatus.OK if payload.get("ok") else HTTPStatus.BAD_GATEWAY)
            except Exception as exc:
                return self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_GATEWAY)
        if parsed.path == "/api/ducting/at":
            try:
                duct_qs = parse_qs(parsed.query)
                lat = float(duct_qs.get("lat", [None])[0])
                lon = float(duct_qs.get("lon", [None])[0])
            except (TypeError, ValueError):
                return self.send_json({"ok": False, "error": "bad_coordinates"}, status=HTTPStatus.BAD_REQUEST)
            try:
                payload = tropospheric_ducting_for_location(lat, lon)
                return self.send_json(payload, status=HTTPStatus.OK if payload.get("ok") else HTTPStatus.BAD_GATEWAY)
            except Exception as exc:
                return self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_GATEWAY)
        if parsed.path == "/api/noaa/satellites":
            try:
                return self.send_json({"ok": True, "satellites": noaa_current_rows()})
            except Exception as exc:
                return self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_GATEWAY)
        if parsed.path == "/api/qso/recent":
            try:
                entries = load_recent_qsos(limit=20)
                total_count = 0
                if QSO_RECENT_PATH.exists():
                    with open(QSO_RECENT_PATH, "r", encoding="utf-8") as handle:
                        total_count = sum(1 for line in handle if line.strip())
                return self.send_json({"ok": True, "entries": entries, "totalCount": total_count})
            except Exception as exc:
                return self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_GATEWAY)
        if parsed.path == "/api/upload/credentials/status":
            return self.send_json({"ok": True, "status": credentials_status_summary()})

        return self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self):
        refresh_host_wifi()
        parsed = urlparse(self.path)
        qs = parse_qs(parsed.query)

        if parsed.path == "/api/desktop":
            exit_kiosk_to_desktop()
            return self.send_json({"ok": True})

        if parsed.path == "/api/pi/shutdown":
            ok, detail = request_pi_shutdown()
            if ok:
                return self.send_json({"ok": True, "status": "shutdown_requested"})
            return self.send_json({"ok": False, "error": detail}, status=HTTPStatus.SERVICE_UNAVAILABLE)

        if parsed.path == "/api/pi/reboot":
            ok, detail = request_pi_reboot()
            if ok:
                return self.send_json({"ok": True, "status": "reboot_requested"})
            return self.send_json({"ok": False, "error": detail}, status=HTTPStatus.SERVICE_UNAVAILABLE)

        if parsed.path == "/api/nucleo/reset":
            ok = pulse_nucleo_reset()
            if ok:
                with state_lock:
                    latest["status"] = "NUCLEO RESET SENT"
                    latest["nucleoResetStatus"] = f"RESET SENT {datetime.now().strftime('%H:%M:%S')} - WAITING"
                return self.send_json({"ok": True, "status": latest["nucleoResetStatus"]})
            return self.send_json({"ok": False, "error": "serial_not_ready"}, status=HTTPStatus.SERVICE_UNAVAILABLE)

        if parsed.path == "/api/open-site":
            target = qs.get("target", [""])[0].strip()
            if not launch_external_site(target):
                return self.send_json({"ok": False, "error": "bad_target"}, status=HTTPStatus.BAD_REQUEST)
            return self.send_json({"ok": True})

        if parsed.path == "/api/qso/log":
            try:
                body = self.read_json_body()
            except ValueError:
                return self.send_json({"ok": False, "error": "bad_json"}, status=HTTPStatus.BAD_REQUEST)

            callsign = str(body.get("callsign", "")).strip().upper()
            if not callsign:
                return self.send_json({"ok": False, "error": "callsign_required"}, status=HTTPStatus.BAD_REQUEST)

            with state_lock:
                current_az = float(latest.get("az", 0.0))
                current_el = float(latest.get("el", 0.0))
                current_sat = str(latest.get("sat", "") or "")

            qso = {
                "callsign": callsign,
                "freq_mhz": str(body.get("freqMhz", "")).strip(),
                "mode": str(body.get("mode", "")).strip().upper(),
                "rst_sent": str(body.get("rstSent", "")).strip(),
                "rst_rcvd": str(body.get("rstRcvd", "")).strip(),
                "az": current_az,
                "el": current_el,
                "sat_name": str(body.get("satName", "")).strip() or current_sat,
                "notes": str(body.get("notes", "")).strip(),
            }

            try:
                append_qso_to_adif(qso)
                recent_entry = append_qso_recent_entry(qso)
            except Exception as exc:
                return self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.INTERNAL_SERVER_ERROR)

            with state_lock:
                latest["status"] = f"QSO LOGGED {callsign}"

            return self.send_json({"ok": True, "logged": recent_entry})

        if parsed.path == "/api/upload/credentials/set":
            try:
                body = self.read_json_body()
            except ValueError:
                return self.send_json({"ok": False, "error": "bad_json"}, status=HTTPStatus.BAD_REQUEST)

            service = str(body.get("service", "")).strip().lower()
            if service not in ("qrz", "clublog", "eqsl"):
                return self.send_json({"ok": False, "error": "bad_service"}, status=HTTPStatus.BAD_REQUEST)

            creds = load_upload_credentials()
            service_creds = {k: str(v).strip() for k, v in body.items() if k != "service"}
            creds[service] = service_creds
            try:
                save_upload_credentials(creds)
            except Exception as exc:
                return self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.INTERNAL_SERVER_ERROR)

            return self.send_json({"ok": True, "service": service, "status": credentials_status_summary()})

        if parsed.path == "/api/rotor/zero":
            with state_lock:
                if not bool(rotor_live.get("queryValid", False)):
                    return self.send_json(
                        {
                            "ok": False,
                            "error": "head_query_invalid",
                            "status": "Refusing to zero: head query is not valid",
                            "queryPanOk": bool(rotor_live.get("queryPanOk", False)),
                            "queryTiltOk": bool(rotor_live.get("queryTiltOk", False)),
                        },
                        status=HTTPStatus.SERVICE_UNAVAILABLE,
                    )
                rotor_zero["az"] = norm_az(float(rotor_live.get("queryAz", rotor_live.get("az", latest.get("az", 0.0)))))
                rotor_zero["el"] = float(rotor_live.get("queryEl", rotor_live.get("el", latest.get("el", 0.0))))
                rotor_zero["azUnwrapped"] = rotor_zero["az"]
                rotor_zero["totalAz"] = 0.0
                rotor_zero["panRaw"] = normalize_pan_raw(rotor_live.get("queryPanRaw", 0)) if bool(rotor_live.get("queryPanOk", False)) else inferred_pan_raw_from_az(rotor_zero["az"])
                rotor_zero["enabled"] = True
                reset_limit_tracker_locked(0.0)
                save_rotor_zero_locked()
                apply_encoder_overlay_locked()
                latest["status"] = "ROTOR ZEROED 0/0"
                payload = {"ok": True, "az": latest["az"], "el": latest["el"], "source": latest["rotorSource"]}
            sync_az_limit_zero_reference()
            return self.send_json(payload)

        if parsed.path == "/api/rotor/query":
            with state_lock:
                if not rotor_zero["enabled"] and bool(rotor_live.get("queryValid", False)):
                    rotor_zero["az"] = norm_az(float(rotor_live.get("queryAz", rotor_live.get("az", latest.get("az", 0.0)))))
                    rotor_zero["el"] = float(rotor_live.get("queryEl", rotor_live.get("el", latest.get("el", 0.0))))
                    rotor_zero["azUnwrapped"] = rotor_zero["az"]
                    rotor_zero["totalAz"] = 0.0
                    rotor_zero["panRaw"] = normalize_pan_raw(rotor_live.get("queryPanRaw", 0)) if bool(rotor_live.get("queryPanOk", False)) else inferred_pan_raw_from_az(rotor_zero["az"])
                    rotor_zero["enabled"] = True
                    reset_limit_tracker_locked(0.0)
                    save_rotor_zero_locked()
                    apply_encoder_overlay_locked()
                    latest["status"] = "HEAD QUERY ZERO CAPTURED"
                    should_sync_limit_zero = False
                else:
                    apply_encoder_overlay_locked()
                    latest["status"] = "HEAD QUERY SNAPSHOT" if bool(rotor_live.get("queryValid", False)) else "HEAD QUERY INVALID"
                    should_sync_limit_zero = False
                payload = {
                    "ok": True,
                    "queryValid": bool(rotor_live.get("queryValid", False)),
                    "az": round(float(latest.get("az", 0.0)), 1),
                    "el": round(float(latest.get("el", 0.0)), 1),
                    "rawAz": round(float(rotor_live.get("queryAz", latest.get("az", 0.0))), 1),
                    "rawEl": round(float(rotor_live.get("queryEl", latest.get("el", 0.0))), 1),
                    "queryPanOk": bool(rotor_live.get("queryPanOk", False)),
                    "queryTiltOk": bool(rotor_live.get("queryTiltOk", False)),
                    "queryPanRaw": int(rotor_live.get("queryPanRaw", 0)),
                    "queryTiltRaw": int(rotor_live.get("queryTiltRaw", 0)),
                    "source": str(latest.get("rotorSource", "--")),
                    "status": str(latest.get("status", "")),
                }
            return self.send_json(payload)

        if parsed.path == "/api/rotor/home":
            with state_lock:
                rotor_net["active"] = False
                rotor_net["lastCmd"] = "PTZ STOP"
                rotor_net["homePrevAzErr"] = None
                rotor_net["homePrevElErr"] = None
                rotor_net["homeWorseTicks"] = 0
                latest["status"] = "ROTOR HOME DISABLED"
            return self.send_json(
                {"ok": False, "error": "home_disabled", "status": latest["status"]},
                status=HTTPStatus.SERVICE_UNAVAILABLE,
            )

        if parsed.path == "/api/rotor/park":
            with state_lock:
                sat_lock["enabled"] = False
                clear_sat_lock_last_good_locked()
            set_rotor_net_target(0.0, ROTOR_HOME_EL_TARGET_DEG, "ui-park")
            with state_lock:
                rotor_net["lastCmd"] = "PTZ STOP"
                rotor_net["lastSendMs"] = 0.0
                rotor_net["homePrevAzErr"] = None
                rotor_net["homePrevElErr"] = None
                rotor_net["homeWorseTicks"] = 0
                latest["status"] = "ROTOR PARKING"
            return self.send_json(
                {
                    "ok": True,
                    "status": "parking",
                    "targetAz": 0.0,
                    "targetEl": ROTOR_HOME_EL_TARGET_DEG,
                },
            )

        if parsed.path == "/api/rotor/target":
            try:
                body = self.read_json_body()
                az = float(body.get("az"))
                el = float(body.get("el"))
            except (TypeError, ValueError):
                return self.send_json({"ok": False, "error": "bad_target"}, status=HTTPStatus.BAD_REQUEST)
            if not math.isfinite(az) or not math.isfinite(el):
                return self.send_json({"ok": False, "error": "bad_target"}, status=HTTPStatus.BAD_REQUEST)
            set_rotor_net_target(az, el, "ui-manual")
            with state_lock:
                sat_lock["enabled"] = False
                clear_sat_lock_last_good_locked()
                latest["status"] = f"ROTOR MANUAL TARGET AZ {norm_az(az):.1f} EL {clamp(el, 0.0, 90.0):.1f}"
            return self.send_json(
                {
                    "ok": True,
                    "status": "manual_target",
                    "targetAz": norm_az(az),
                    "targetEl": clamp(el, 0.0, 90.0),
                },
            )

        if parsed.path == "/api/rotator/link-config":
            try:
                body = self.read_json_body()
            except ValueError:
                return self.send_json({"ok": False, "error": "bad_json"}, status=HTTPStatus.BAD_REQUEST)
            selected = str(body.get("selectedProfile", "") or "").strip()
            if selected not in ROTATOR_PROFILE_INDEX:
                return self.send_json({"ok": False, "error": "bad_profile"}, status=HTTPStatus.BAD_REQUEST)
            with state_lock:
                rotator_link_config["selectedProfile"] = selected
                save_rotator_link_config_locked()
                latest["status"] = f"ROTATOR PROFILE {ROTATOR_PROFILE_INDEX[selected]['label'].upper()}"
                payload = current_rotator_link_payload()
            with external_rotator_lock:
                close_external_rotator_serial_locked()
            return self.send_json(payload)

        if parsed.path == "/api/civ/config":
            try:
                body = self.read_json_body()
            except ValueError:
                return self.send_json({"ok": False, "error": "bad_json"}, status=HTTPStatus.BAD_REQUEST)
            allowed_radio_types = {"standard-icom", "icom-820-821-910", "icom-7100"}
            allowed_vfo_types = {"main-up-sub-down", "main-down-sub-up", "single-vfo"}
            allowed_after_actions = {"none", "restore-vfo", "set-frequency", "memory-channel"}
            try:
                baud = int(body.get("baud", 19200))
                update_ms = int(body.get("updateIntervalMs", 500))
                min_hz = int(body.get("minFreqChangeHz", 25))
            except (TypeError, ValueError):
                return self.send_json({"ok": False, "error": "bad_numeric"}, status=HTTPStatus.BAD_REQUEST)
            address = re.sub(r"[^0-9A-Fa-f]", "", str(body.get("address", "A2")))[:2].upper() or "A2"
            radio_type = str(body.get("radioType", "standard-icom") or "standard-icom")
            vfo_type = str(body.get("vfoType", "main-up-sub-down") or "main-up-sub-down")
            after_action = str(body.get("afterPassAction", "none") or "none")
            if radio_type not in allowed_radio_types:
                radio_type = "standard-icom"
            if vfo_type not in allowed_vfo_types:
                vfo_type = "main-up-sub-down"
            if after_action not in allowed_after_actions:
                after_action = "none"
            baud = baud if baud in (9600, 19200, 38400) else 19200
            update_ms = min(max(update_ms, 100), 5000)
            min_hz = min(max(min_hz, 1), 1000)
            with state_lock:
                civ_router_config.update(
                    {
                        "enabled": bool(body.get("enabled")),
                        "routerSerialPath": str(body.get("routerSerialPath", "") or "").strip(),
                        "radioType": radio_type,
                        "baud": baud,
                        "address": address,
                        "vfoType": vfo_type,
                        "updateIntervalMs": update_ms,
                        "minFreqChangeHz": min_hz,
                        "afterPassAction": after_action,
                        "afterPassValue": str(body.get("afterPassValue", "") or "").strip(),
                        "afterPassMode": str(body.get("afterPassMode", "FM") or "FM").strip().upper()[:12],
                        "afterPassPl": str(body.get("afterPassPl", "") or "").strip(),
                        "port1Label": str(body.get("port1Label", "IC-9700") or "IC-9700").strip(),
                        "port2Label": str(body.get("port2Label", "Icom Aux 1") or "Icom Aux 1").strip(),
                        "port3Label": str(body.get("port3Label", "Icom Aux 2") or "Icom Aux 2").strip(),
                    }
                )
                save_civ_router_config_locked()
                latest["status"] = "CI-V ROUTER ON" if civ_router_config["enabled"] else "CI-V ROUTER OFF"
                payload = current_civ_router_payload()
            return self.send_json(payload)

        if parsed.path == "/api/control":
            cmd = ptz_command(
                qs.get("action", [""])[0],
                qs.get("axis", [""])[0],
                qs.get("dir", [""])[0],
            )
            if not cmd:
                return self.send_json({"ok": False, "error": "bad_command"}, status=HTTPStatus.BAD_REQUEST)
            ok = serial_write(cmd)
            if ok:
                with state_lock:
                    if cmd == "PTZ STOP":
                        rotor_net_cancel_event.set()
                        rotor_net["active"] = False
                        rotor_net["lastCmd"] = "PTZ STOP"
                        sat_lock["enabled"] = False
                        clear_sat_lock_last_good_locked()
                    latest["status"] = cmd
                    apply_encoder_overlay_locked()
                return self.send_json({"ok": True, "sent": cmd, "via": ["serial"]})
            return self.send_json({"ok": False, "sent": cmd, "error": "serial_not_ready"}, status=HTTPStatus.SERVICE_UNAVAILABLE)

        if parsed.path in (
            "/api/sat/mode",
            "/api/sat/profile",
        ):
            return self.send_json({"ok": True})

        if parsed.path == "/api/sat/tle/refresh":
            requested = qs.get("sat", [latest.get("sat", "AO-91")])[0]
            selected_item = find_amateur_catalog_item(requested)
            selected_sat = selected_item["name"] if selected_item else normalize_supported_sat(requested)
            try:
                refreshed = refresh_tles(force_refresh=True, sat_names=[selected_sat, *SUPPORTED_SATS])
                clear_visible_sat_cache()
                return self.send_json(
                    {
                        "ok": True,
                        "refreshed": refreshed,
                        "selectedSat": selected_sat,
                        "tleAge": _tle_age_label(satnogs_norad_for_name(selected_sat)),
                    }
                )
            except Exception as exc:
                return self.send_json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_GATEWAY)

        if parsed.path == "/api/sat/lock":
            requested = qs.get("sat", [sat_lock.get("sat") or latest.get("sat", "AO-91")])[0]
            selected_item = find_amateur_catalog_item(requested)
            selected_sat = selected_item["name"] if selected_item else normalize_supported_sat(requested)
            mode = str(qs.get("mode", ["toggle"])[0] or "toggle").strip().lower()
            should_stop_tracking = False
            if mode == "toggle":
                sat_lock["enabled"] = not bool(sat_lock.get("enabled"))
            elif mode in ("on", "enable", "enabled", "1", "true"):
                sat_lock["enabled"] = True
            elif mode in ("off", "disable", "disabled", "0", "false"):
                sat_lock["enabled"] = False
            sat_lock["sat"] = selected_sat
            if not sat_lock["enabled"]:
                clear_sat_lock_last_good_locked()
                rotor_net["active"] = False
                rotor_net["targetAz"] = None
                rotor_net["targetEl"] = None
                rotor_net["lastCmd"] = "PTZ STOP"
                rotor_net["lastSendMs"] = 0.0
                rotor_net["lastClient"] = ""
                rotor_net["homePrevAzErr"] = None
                rotor_net["homePrevElErr"] = None
                rotor_net["homeWorseTicks"] = 0
                latest["status"] = "SAT LOCK OFF"
                should_stop_tracking = True
            else:
                selected_payload = skyfield_current_payload(selected_sat)
                selected_data = selected_payload.get("data", {}) if isinstance(selected_payload, dict) else {}
                selected_el = float(selected_data.get("selectedEl") or -999.0)
                if not isinstance(selected_payload, dict) or not selected_payload.get("ok") or selected_el < SAT_TRACK_MIN_EL_DEG:
                    sat_lock["enabled"] = False
                    clear_sat_lock_last_good_locked()
                    rotor_net["active"] = False
                    rotor_net["targetAz"] = None
                    rotor_net["targetEl"] = None
                    rotor_net["lastCmd"] = "PTZ STOP"
                    rotor_net["lastSendMs"] = 0.0
                    rotor_net["lastClient"] = ""
                    rotor_net["homePrevAzErr"] = None
                    rotor_net["homePrevElErr"] = None
                    rotor_net["homeWorseTicks"] = 0
                    latest["status"] = f"SAT LOCK REFUSED {selected_sat} BELOW MIN EL {SAT_TRACK_MIN_EL_DEG:.1f}"
                    send_ptz_command("stop", "", "")
                    return self.send_json(
                        {
                            "ok": False,
                            "error": "satellite_below_min_elevation",
                            "lockEnabled": False,
                            "lockSat": selected_sat,
                            "lockState": "UNLOCKED",
                            "elevation": round(selected_el, 1) if selected_el > -900.0 else None,
                            "minElevation": SAT_TRACK_MIN_EL_DEG,
                        },
                        status=HTTPStatus.CONFLICT,
                    )
                rotor_net["lastCmd"] = "PTZ STOP"
                rotor_net["lastSendMs"] = 0.0
                rotor_net["homePrevAzErr"] = None
                rotor_net["homePrevElErr"] = None
                rotor_net["homeWorseTicks"] = 0
                sat_lock["lastGoodAz"] = float(selected_data.get("selectedAz") or 0.0)
                sat_lock["lastGoodEl"] = float(selected_data.get("selectedEl") or 0.0)
                sat_lock["lastGoodTs"] = time.time()
                sat_lock["commandAz"] = float(selected_data.get("selectedAz") or 0.0)
                sat_lock["commandEl"] = float(selected_data.get("selectedEl") or 0.0)
                sat_lock["commandTs"] = time.time()
                sat_lock["acquireActive"] = True
                sat_lock["acquireStartedTs"] = time.time()
                latest["status"] = f"SAT LOCK {selected_sat}"
            if should_stop_tracking:
                send_ptz_command("stop", "", "")
            return self.send_json(
                {
                    "ok": True,
                    "lockEnabled": bool(sat_lock["enabled"]),
                    "lockSat": sat_lock["sat"],
                    "lockState": "LOCKED" if sat_lock["enabled"] else "UNLOCKED",
                }
            )

        return self.send_error(HTTPStatus.NOT_FOUND)


def stale_watchdog():
    last_stale_check = 0.0
    while True:
        time.sleep(0.2)
        now = time.time()
        if (now - last_stale_check) < 3.0:
            continue
        last_stale_check = now
        refresh_host_wifi()
        if last_serial_ts and (time.time() - last_serial_ts) > 15:
            with state_lock:
                stale_seconds = int(time.time() - last_serial_ts)
                should_recover = not str(latest.get("status", "")).startswith("SERIAL RECOVER:")
                latest["status"] = f"SERIAL STALE {stale_seconds}s"
                if sat_lock.get("enabled"):
                    sat_lock["enabled"] = False
                    clear_sat_lock_last_good_locked()
                    rotor_net["active"] = False
                    rotor_net["lastCmd"] = "PTZ STOP"
                    rotor_net["lastSendMs"] = 0.0
                    latest["status"] = f"SERIAL STALE {stale_seconds}s - TRACKING HALTED"
            if should_recover:
                force_serial_reopen(f"stale {stale_seconds}s")


def main():
    refresh_host_wifi()
    load_rotor_zero_state()
    load_rotator_link_config()
    load_civ_router_config()
    threading.Thread(target=serial_reader, daemon=True).start()
    if GPS_SERIAL_PATH:
        threading.Thread(target=gps_serial_reader, daemon=True).start()
    threading.Thread(target=stale_watchdog, daemon=True).start()
    threading.Thread(target=rotor_net_loop, daemon=True).start()
    threading.Thread(target=rotctld_listener, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    gps_label = GPS_SERIAL_PATH or "nucleo-forwarded"
    print(f"simple_bridge listening on http://127.0.0.1:{PORT} serial={SERIAL_PATH} gps={gps_label}")
    server.serve_forever()


if __name__ == "__main__":
    main()
