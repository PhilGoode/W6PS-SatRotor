// GIGA R1 WiFi -- Pelco-D driver for the PT-305-0DZ / PTS-3050DZ head,
// plus the GPS and encoder subsystems wired alongside it.
//
// Role in the split architecture: this sketch owns everything specific to
// THIS head's quirks (elevation inversion, azimuth phase offset, the
// low-speed jog stall bug) so that the UNO Q side can speak plain, generic
// az/el numbers and not care which head is on the other end. It also owns
// the GPS (SparkFun NEO-F10N) and the two Bourns EM14 encoders, since all
// three live on this board's pins.
//
// Talks to the UNO Q over the existing proven USB-serial link (115200 baud).
// Protocol, one line at a time:
//
//   UNO Q -> GIGA   "TARGET <az> <el>\n"   start driving toward a target
//   UNO Q -> GIGA   "POS?\n"               request current rotor position
//   UNO Q -> GIGA   "STOP\n"               stop and hold
//   UNO Q -> GIGA   "GPS?\n"               request latest GPS fix
//   UNO Q -> GIGA   "ENC?\n"               request running signed encoder step counts
//
// Encoders also work as a manual jog dial independent of the UNO Q link --
// these Bourns EM14s are continuous/smooth (no click-stops), so turning one
// steadily produces a steady stream of quadrature steps. This is now an
// OPEN-LOOP velocity jog (matching the old W6PS SatRotor Nucleo firmware's
// applyJogMotion()/maybeStopEncoderJog()), not closed-loop target tracking:
// each fresh step sends a raw "move this direction" Pelco command and resets
// a short dead-man timer (ENCODER_JOG_HOLD_MS); stop turning and, once that
// timer lapses with no new steps, it sends STOP and ends the jog session.
// No position math is involved in jog at all -- deliberately, since nudging
// a target and letting TARGET/driveTowardTarget()'s tolerance-based bang-
// bang loop chase it (the original design here) is what caused a "waggle"
// right at the stop point once jog speed was raised to fix a grinding
// noise. Real absolute TARGET commands still use the closed-loop tolerance
// system; jog no longer does.
//   (debug) "QUERY\n"    forces a live Pelco-D query right now, bypassing the
//                        95s startup gate -- reports OK/FAIL + raw pan/tilt,
//                        does NOT read cached state like POS? does
//   (debug) "PRESET <n>\n" sends a raw "call preset n" command -- used to
//                        test the preset 125 hypothesis (see notes in
//                        handleLine). Use with caution, see inline comment.
//   (debug) "SKIP_STARTUP\n" bench-only override -- use after reflashing the
//                        GIGA when the head itself stayed powered on and is
//                        already well past its own startup dance. Do NOT
//                        use this after a real shared power-up -- the
//                        default (setup() arms the 95s wait) is correct
//                        for that normal, deployed case.
//   (debug) "HEAD_POWERED_ON\n" re-arms the 95s wait -- only needed if you
//                        cycle the head's power alone without the GIGA
//   (debug) "AZSET <realAz>\n" resyncs the azimuth offset so the CURRENT
//                        physical position (wherever the head happens to be
//                        pointed right now) reads as <realAz> from now on.
//                        Doesn't move anything -- purely corrects our one
//                        fixed assumption about the head's raw pan count vs
//                        true north, for when the head's own azimuth
//                        reference has drifted. NOT persisted -- resets to
//                        the 180 degree default on every reboot/reflash.
//   (debug) "ELSET <realEl>\n" same idea, elevation axis -- resyncs so the
//                        CURRENT physical elevation reads as <realEl> from
//                        now on. Doesn't move anything. NOT persisted --
//                        resets to the 0 degree default on every
//                        reboot/reflash.
//   (debug) "SPEED?\n"    reports current pan/tilt jog speed (0-63)
//   (debug) "SPEED PAN <n>\n" / "SPEED TILT <n>\n" sets the fixed jog speed
//                        used by encoder-jog driving and JOGUP/JOGDOWN, live,
//                        no reflash needed. NOT persisted -- resets to the
//                        defaults (panSpeed=0x20, tiltSpeed=0x3F) on every
//                        reboot/reflash. Added to test whether a too-low
//                        commanded azimuth speed is behind a grindy noise
//                        heard under encoder jog.
//   (debug) "SPEED PANMIN <n>\n" sets the deceleration floor TARGET moves
//                        are allowed to slow pan down to on final approach
//                        (see stagedPanSpeed()). Defaults to 63 (= no
//                        deceleration, holds flat at panSpeed) since a
//                        hardcoded floor of 20 audibly ground/nearly
//                        stalled the gears -- only lower this after briefly
//                        testing a candidate value and confirming it
//                        doesn't grind.
//   (debug) "TOL?\n"      reports current stop/resume position tolerance
//   (debug) "TOL STOP <deg>\n" / "TOL RESUME <deg>\n" tune the hysteresis
//                        band live, no reflash needed. NOT persisted --
//                        resets to defaults (stop=1.0, resume=3.0) on
//                        every reboot/reflash. Added to kill a "waggle"
//                        right before stopping that showed up once jog
//                        speed was raised -- resume must stay looser than
//                        stop or this does nothing.
//
//   GIGA  -> UNO Q   "POS <az> <el> <moving 0|1>\n"        reply to POS?/TARGET/STOP
//   GIGA  -> UNO Q   "GPS <lat> <lon> <altM> <fixQ> <numSat>\n"  reply to GPS? (has fix)
//   GIGA  -> UNO Q   "GPS NOFIX <fixQ> <numSat>\n"                reply to GPS? (no fix yet)
//   GIGA  -> UNO Q   "ENC <azCount> <elCount>\n"            reply to ENC?
//
// All az/el values crossing the USB link are REAL-WORLD values (0-360 az,
// true elevation where +90 = zenith, 0 = horizon, negative = below
// horizon). Everything head-specific happens inside encodePan/encodeTilt/
// decodePan/decodeTilt below, never outside this file.
//
// Encoders are a real direction-aware quadrature decode (both A and B
// channels, standard 4-bit transition table) and are wired up as an
// open-loop velocity jog dial (see applyEncoderJog() -- direct raw motion
// command + dead-man timeout, not closed-loop target tracking; matches the
// old Nucleo firmware's approach). ENC? still reports the running signed
// step count on each axis regardless.
//
// GPS parsing only reads GGA sentences (position/fix/altitude/satellite
// count) -- enough for now. Time/date (RMC) can be added later if needed.
//
// Pelco-D query byte framing (queryPosition()) is carried over from the
// W6PS SatRotor project's calibration notes and has not yet been confirmed
// against this specific physical head -- verify once the RS-485 link to
// the actual head is live, adjust if it doesn't answer.
//
// NOTE (2026-08-02): this is a ROLLBACK to the last confirmed-working
// version -- the one right after the encoder jog-dial work, BEFORE the
// persistent-calibration/KVStore addition. That addition was flashed and
// immediately afterward QUERY started returning FAIL/FAIL on both axes.
// This file removes kvstore_global_api.h and the Calibration struct/CAL
// commands entirely and goes back to plain hardcoded constants, so we can
// test empirically whether the KVStore code was actually the cause.

#include <Arduino.h>

// ---------------------------------------------------------------------
// Config -- values carried over from the proven W6PS SatRotor firmware
// for this same head model, plus the fixed-speed/tolerance strategy from
// the independent Habr writeup + belovictor/pelco_d_rotator reference
// (see PELCO_D_LOW_SPEED_STALL_WORKAROUND.md).
// ---------------------------------------------------------------------

static constexpr uint8_t PELCO_ADDRESS = 1;
static constexpr uint32_t PELCO_BAUD = 2400;  // confirmed matches the head's actual DIP switches

// MAX13487 has auto direction control -- no DE/RE pin to manage, unlike the
// MAX485-based modules the old project used.
HardwareSerial &pelcoSerial = Serial1;  // GIGA D0(RX)/D1(TX) -> RS-485 board

static constexpr float AZ_MIN_DEG = 0.0f;
static constexpr float AZ_MAX_DEG = 359.99f;
static constexpr float EL_MIN_DEG = -20.0f;
static constexpr float EL_MAX_DEG = 90.0f;

// The antenna is mounted backward relative to how this head's tilt axis was
// originally calibrated for a camera: positive real elevation corresponds
// to commanding negative Pelco tilt, and vice versa on readback.
static constexpr bool INVERT_ELEVATION_USING_NEGATIVE_TILT = true;

// This head's azimuth zero is nominally 180 degrees from Phil's physical
// zero -- but this is a single fixed assumption about the relationship
// between the head's own internal pan count and true-north, and this head's
// azimuth reference (spoofboard included) is known to be able to drift
// after enough movement. Runtime-adjustable via AZSET <realAz> (see
// handleLine) so it can be resynced on the spot without a reflash whenever
// the reported position stops matching physical reality -- NOT persisted,
// resets to this default on every reboot/reflash.
float azPhaseOffsetDeg = 180.0f;

// Same idea for elevation -- ELSET <realEl> resyncs this on the spot.
// Additive correction: subtracted in encodeTilt() before the existing invert
// logic, added back in decodeTilt() after it, so "real elevation X" can be
// re-anchored to true physical reality independent of whatever the head's
// own internal tilt zero happens to be. NOT persisted.
float elOffsetDeg = 0.0f;

// Fixed-speed + tolerance-based stop, NOT a slow creep-in speed. This head's
// firmware silently refuses to move at all at low commanded speeds -- see
// PELCO_D_LOW_SPEED_STALL_WORKAROUND.md. Starting values below are the ones
// an independent reference implementation reported as working on this same
// head family.
//
// These are runtime-adjustable (SPEED PAN <n> / SPEED TILT <n> / SPEED?,
// values 0-63, matching Pelco-D's standard speed byte range) instead of
// fixed compile-time constants -- added 2026-08-02 to let Phil quickly test
// speed without a reflash per test. NOT persisted -- resets to the defaults
// below on every reboot/reflash, on purpose (this is a bench tuning knob,
// not a calibration value). panSpeed default bumped from the original 0x20
// to 50 (0x32) this same day, since 0x20 was confirmed to grind/near-stall
// under encoder jog -- 50 was confirmed clean. tiltSpeed stays at the
// original max (0x3F) -- never proven safe to run any lower.
uint8_t panSpeed = 50;
uint8_t tiltSpeed = 0x3F;

// Two-threshold hysteresis, both runtime-adjustable (TOL?/TOL STOP <deg>/
// TOL RESUME <deg>), NOT persisted. Added 2026-08-02 after raising jog speed
// fixed the grinding noise but exposed a "waggle" right before stopping --
// classic fixed-speed bang-bang overshoot: at higher speed the head covers
// more ground per POLL_INTERVAL_MS poll, so it blows past a single tight
// tolerance, corrects back, overshoots the other way, repeat. Fix: use the
// tight tolerance to decide when to STOP, but require missing by more than
// the looser resume tolerance before driving is allowed to START again once
// already stopped -- kills the ping-pong right at the edge without slowing
// back down.
float positionToleranceDeg = 1.0f;
float resumeToleranceDeg = 3.0f;

// This head runs a fixed ~95 second startup routine after power-up before
// it will respond to anything. Can't be skipped or shortened; baked into
// the head's own firmware.
static constexpr uint32_t HEAD_STARTUP_MS = 95000;

static constexpr uint32_t QUERY_TIMEOUT_MS = 250;
static constexpr uint32_t POLL_INTERVAL_MS = 300;

// Manual jog dial -- reworked 2026-08-02 to match the old W6PS SatRotor
// Nucleo firmware's approach after comparing the two: that firmware does NOT
// drive encoder jog through closed-loop position tracking at all. It just
// sends one raw "move this direction" Pelco command while you're actively
// turning, and a short dead-man timer (ENCODER_JOG_HOLD_MS, 140ms there)
// stops it shortly after you stop turning -- no target, no tolerance check,
// nothing to hunt. This sketch's original encoder jog instead nudged
// rotor.targetAz/targetEl and let the same tolerance-based
// driveTowardTarget() used for real TARGET commands chase it -- that bang-
// bang loop is exactly what was causing the "waggle" at the end of a jog.
// Switched to the same open-loop velocity-jog-plus-timeout model here.
static constexpr uint32_t ENCODER_JOG_HOLD_MS = 150;

// --- GPS (SparkFun GNSS NEO-F10N) ---
// Ships at 38400 baud NMEA by default (SparkFun hardware overview docs).
static constexpr uint32_t GPS_BAUD = 38400;
HardwareSerial &gpsSerial = Serial2;  // GIGA D18 (TX2) / D19 (RX2)

// --- Encoders (Bourns EM14R0B-M25-L064N via SN74LVC245A 5V->3.3V buffer) ---
static constexpr uint8_t AZ_ENC_A_PIN = 2;
static constexpr uint8_t AZ_ENC_B_PIN = 3;
static constexpr uint8_t EL_ENC_A_PIN = 4;
static constexpr uint8_t EL_ENC_B_PIN = 5;

// ---------------------------------------------------------------------
// State
// ---------------------------------------------------------------------

struct RotorState {
  float currentAz = 0.0f;
  float currentEl = 0.0f;
  float targetAz = 0.0f;
  float targetEl = 0.0f;
  bool hasTarget = false;
  bool hasPosition = false;
  bool isMoving = false;
};

struct GpsState {
  bool hasFix = false;
  float latDeg = 0.0f;
  float lonDeg = 0.0f;
  float altM = 0.0f;
  int fixQuality = 0;
  int numSatellites = 0;
};

RotorState rotor;
GpsState gpsState;
volatile long azEncoderCount = 0;  // running signed quadrature step count
volatile long elEncoderCount = 0;
volatile uint8_t azQuadState = 0;  // last 2-bit (A<<1|B) state per axis
volatile uint8_t elQuadState = 0;

uint32_t bootMs = 0;
uint32_t lastPollMs = 0;
long azAppliedCount = 0;  // how much of azEncoderCount we've already turned
long elAppliedCount = 0;  // into jog direction tracking
String lineBuffer;
String gpsLineBuffer;

// Encoder jog state -- open-loop velocity jog + dead-man timeout, matching
// the old W6PS SatRotor Nucleo firmware's applyJogMotion()/
// maybeStopEncoderJog() rather than nudging a closed-loop target (see the
// long comment above ENCODER_JOG_HOLD_MS for why).
int8_t azJogDir = 0;  // -1, 0, or +1 -- last direction turned on this axis
int8_t elJogDir = 0;
uint32_t jogHoldUntilMs = 0;  // 0 = no jog session active right now

// ---------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------

float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float normalizeAz(float az) {
  float wrapped = fmodf(az, 360.0f);
  if (wrapped < 0.0f) wrapped += 360.0f;
  return wrapped;
}

// Shortest signed delta from `from` to `to` on a 0-360 circle, in [-180, 180].
float shortestAzDelta(float from, float to) {
  float delta = fmodf(to - from, 360.0f);
  if (delta > 180.0f) delta -= 360.0f;
  if (delta < -180.0f) delta += 360.0f;
  return delta;
}

uint8_t pelcoChecksum(uint8_t address, uint8_t cmd1, uint8_t cmd2, uint8_t data1, uint8_t data2) {
  return static_cast<uint8_t>(address + cmd1 + cmd2 + data1 + data2);
}

// Standard quadrature decode transition table, indexed by
// (prevState<<2 | newState) where each state is (A<<1 | B). Returns +1, -1,
// or 0 for an invalid/bounced transition. This is the well-known 4-bit
// full-step quadrature table used across most Arduino rotary encoder code.
static const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void azEncoderIsr() {
  uint8_t newState = (digitalRead(AZ_ENC_A_PIN) << 1) | digitalRead(AZ_ENC_B_PIN);
  uint8_t index = (azQuadState << 2) | newState;
  azEncoderCount += QUAD_TABLE[index];
  azQuadState = newState;
}

void elEncoderIsr() {
  uint8_t newState = (digitalRead(EL_ENC_A_PIN) << 1) | digitalRead(EL_ENC_B_PIN);
  uint8_t index = (elQuadState << 2) | newState;
  elEncoderCount += QUAD_TABLE[index];
  elQuadState = newState;
}

// ---------------------------------------------------------------------
// Head-specific transforms -- the only place real-world az/el gets
// converted to/from this head's own raw Pelco-D numbers.
// ---------------------------------------------------------------------

// Real-world azimuth -> raw azimuth this head expects on the wire.
uint16_t encodePan(float realAzDeg) {
  float raw = normalizeAz(realAzDeg + azPhaseOffsetDeg);
  raw = clampFloat(raw, AZ_MIN_DEG, AZ_MAX_DEG);
  return static_cast<uint16_t>(lroundf(raw * 100.0f));
}

// Raw azimuth from this head -> real-world azimuth.
float decodePan(uint16_t rawValue) {
  float raw = fmodf(static_cast<float>(rawValue) / 100.0f, 360.0f);
  if (raw < 0.0f) raw += 360.0f;
  return normalizeAz(raw - azPhaseOffsetDeg);
}

// Real-world elevation -> raw Pelco tilt value this head expects on the wire.
// Encoding mirrors the proven W6PS SatRotor firmware: Pelco-D represents a
// negative tilt as (36000 - magnitude*100) in the 16-bit field.
uint16_t encodeTilt(float realElDeg) {
  float el = clampFloat(realElDeg - elOffsetDeg, EL_MIN_DEG, EL_MAX_DEG);
  float physicalTilt = INVERT_ELEVATION_USING_NEGATIVE_TILT ? -el : el;

  uint32_t raw;
  if (physicalTilt < 0.0f) {
    raw = static_cast<uint32_t>(lroundf(-physicalTilt * 100.0f));
  } else {
    raw = 36000UL - static_cast<uint32_t>(lroundf(physicalTilt * 100.0f));
  }
  raw %= 36000UL;
  return static_cast<uint16_t>(raw);
}

// Raw Pelco tilt value from this head -> real-world elevation.
float decodeTilt(uint16_t rawValue) {
  float physicalTilt;
  if (rawValue > 18000) {
    physicalTilt = (36000.0f - static_cast<float>(rawValue)) / 100.0f;
  } else {
    physicalTilt = -static_cast<float>(rawValue) / 100.0f;
  }
  float el = INVERT_ELEVATION_USING_NEGATIVE_TILT ? -physicalTilt : physicalTilt;
  return clampFloat(el + elOffsetDeg, EL_MIN_DEG, EL_MAX_DEG);
}

// ---------------------------------------------------------------------
// Pelco-D wire I/O
// ---------------------------------------------------------------------

void pelcoSend(uint8_t cmd1, uint8_t cmd2, uint8_t data1, uint8_t data2) {
  uint8_t frame[7] = {
    0xFF,
    PELCO_ADDRESS,
    cmd1,
    cmd2,
    data1,
    data2,
    pelcoChecksum(PELCO_ADDRESS, cmd1, cmd2, data1, data2)
  };
  pelcoSerial.write(frame, sizeof(frame));
  pelcoSerial.flush();
}

bool pelcoReadFrame(uint8_t frame[7], uint32_t timeoutMs) {
  uint32_t start = millis();
  size_t index = 0;
  while (millis() - start < timeoutMs) {
    while (pelcoSerial.available()) {
      uint8_t value = static_cast<uint8_t>(pelcoSerial.read());
      if (index == 0 && value != 0xFF) continue;
      frame[index++] = value;
      if (index == 7) {
        uint8_t expected = pelcoChecksum(frame[1], frame[2], frame[3], frame[4], frame[5]);
        return frame[6] == expected;
      }
    }
  }
  return false;
}

// NOTE: query opcode/response byte framing below matches the last known-good
// convention noted in the W6PS SatRotor project's calibration session log
// (pan: cmd2 0x51 / response 0x59, tilt: cmd2 0x53 / response 0x5B). Not
// yet verified against this specific physical head -- confirm once wired up
// and adjust if the head doesn't answer.
bool pelcoQuery(uint8_t queryCmd2, uint8_t responseCmd2, uint16_t &value) {
  while (pelcoSerial.available()) pelcoSerial.read();  // flush stale bytes
  pelcoSend(0x00, queryCmd2, 0x00, 0x00);

  uint8_t frame[7] = {};
  if (!pelcoReadFrame(frame, QUERY_TIMEOUT_MS)) return false;
  if (frame[1] != PELCO_ADDRESS || frame[2] != 0x00 || frame[3] != responseCmd2) return false;

  value = (static_cast<uint16_t>(frame[4]) << 8) | frame[5];
  return true;
}

bool pollPosition() {
  uint16_t rawPan = 0, rawTilt = 0;
  bool gotPan = pelcoQuery(0x51, 0x59, rawPan);
  bool gotTilt = pelcoQuery(0x53, 0x5B, rawTilt);

  if (gotPan) rotor.currentAz = decodePan(rawPan);
  if (gotTilt) rotor.currentEl = decodeTilt(rawTilt);
  rotor.hasPosition = rotor.hasPosition || gotPan || gotTilt;
  return gotPan && gotTilt;
}

void pelcoStop() {
  pelcoSend(0x00, 0x00, 0x00, 0x00);
}

// Staged speed reduction for PAN only, as azDelta shrinks -- ported from the
// old Nucleo firmware's targetDriveSpeed(), same distance breakpoints
// (35/14/5 degrees). This is what lets a TARGET move arrive already slowing
// down instead of hitting the tolerance band at full commanded speed and
// needing a hard correction (the other half of tonight's "waggle" -- the
// encoder-jog half is fixed separately in applyEncoderJog(), which no longer
// does position-based driving at all).
//
// Deliberately NOT applied to tilt: we've only ever confirmed tilt moves
// reliably at max speed (0x3F) fighting gravity -- an untested lower tilt
// speed stalling mid-move would be a worse bug than a waggle, so tilt stays
// flat at tiltSpeed the whole approach until that's actually verified safe.
//
// panMinSpeed is the deceleration floor, runtime-adjustable via
// SPEED PANMIN <n> (see handleLine), NOT persisted. Defaults to 63 -- higher
// than any valid panSpeed -- so by default this floor never actually kicks
// in and pan holds flat at panSpeed the whole approach (back to the
// original waggle-but-no-grinding behavior). A hardcoded floor of 20 tried
// on 2026-08-02 audibly ground/nearly stalled the gears on the final
// approach -- there is NO evidence yet that anything below the validated
// panSpeed (50 as of tonight) runs clean, so don't guess a lower floor
// again; only lower panMinSpeed after testing a candidate value briefly and
// confirming by ear/feel that it doesn't grind.
uint8_t panMinSpeed = 63;

uint8_t stagedPanSpeed(float errorDeg) {
  float error = fabsf(errorDeg);
  uint8_t floorSpeed = (panMinSpeed < panSpeed) ? panMinSpeed : panSpeed;
  if (error >= 35.0f) return panSpeed;
  if (error >= 14.0f) {
    uint8_t half = panSpeed / 2;
    return half > floorSpeed ? half : floorSpeed;
  }
  if (error >= 5.0f) {
    uint8_t third = panSpeed / 3;
    return third > floorSpeed ? third : floorSpeed;
  }
  return floorSpeed;
}

// Fixed-speed jog toward the current target, one axis at a time is fine --
// Pelco-D combines pan+tilt bits in one command2 byte, so do both at once.
void driveTowardTarget() {
  if (jogHoldUntilMs != 0) return;  // an encoder jog session owns the bus right now
  if (!rotor.hasTarget || !rotor.hasPosition) return;

  float azDelta = shortestAzDelta(rotor.currentAz, rotor.targetAz);
  float elDelta = rotor.targetEl - rotor.currentEl;

  // Hysteresis: while already stopped, require the looser resume tolerance
  // before considering it "not close" again -- otherwise small jitter/steps
  // right at the tight tolerance boundary cause endless restart/overshoot.
  float threshold = rotor.isMoving ? positionToleranceDeg : resumeToleranceDeg;
  bool azClose = fabsf(azDelta) <= threshold;
  bool elClose = fabsf(elDelta) <= threshold;

  if (azClose && elClose) {
    // Target reached -- stop and consider it DONE, don't keep tracking.
    // Root cause of the intermittent TARGET waggle (found by Codex, checked
    // against this file 2026-08-02): this used to leave rotor.hasTarget
    // true, so the very next poll would recompute azDelta/elDelta against
    // whatever the head coasted/backlashed to after the stop command. Land
    // inside resumeToleranceDeg after coasting -> stays stopped, looks
    // clean. Coast past it -> "oh, we're off target again" -> drive back
    // the other way -> waggle. Purely random per-attempt because coast
    // distance after a stop isn't perfectly repeatable -- explains why no
    // speed/tolerance/target-value combination ever fixed it consistently.
    // Clearing hasTarget here makes a TARGET command "drive there once,
    // stop, done" -- a fresh TARGET command (as continuous tracking would
    // send) still re-arms this normally.
    pelcoStop();
    rotor.isMoving = false;
    rotor.hasTarget = false;
    return;
  }

  uint8_t cmd2 = 0x00;
  uint8_t data1 = 0x00;
  uint8_t data2 = 0x00;

  if (!azClose) {
    if (azDelta > 0.0f) {
      cmd2 |= 0x02;  // pan right
    } else {
      cmd2 |= 0x04;  // pan left
    }
    data1 = stagedPanSpeed(azDelta);
  }

  if (!elClose) {
    // Jog direction flips along with the elevation encoding, same rule as
    // the proven SatRotor firmware's applyJogMotion().
    bool wantUp = elDelta > 0.0f;
    bool commandUp = INVERT_ELEVATION_USING_NEGATIVE_TILT ? !wantUp : wantUp;
    // Empirically this head's tilt-up/tilt-down bits are swapped relative to
    // the generic Pelco-D convention we assumed -- every "go down" request
    // was driving it up and pinning at the EL_MAX_DEG clamp instead (seen
    // live: asked for 75, then 0, both climbed to 90). Value encode/decode
    // (encodeTilt/decodeTilt) is confirmed correct via matching readback --
    // only the jog direction bits needed flipping, so this is the one line
    // that changed.
    cmd2 |= commandUp ? 0x10 : 0x08;
    data2 = tiltSpeed;
  }

  pelcoSend(0x00, cmd2, data1, data2);
  rotor.isMoving = true;
}

// ---------------------------------------------------------------------
// GPS (lightweight GGA-only parser)
// ---------------------------------------------------------------------

float nmeaToDecimalDegrees(const String &value, int degDigits) {
  if (value.length() < static_cast<unsigned int>(degDigits)) return 0.0f;
  float degrees = value.substring(0, degDigits).toFloat();
  float minutes = value.substring(degDigits).toFloat();
  return degrees + minutes / 60.0f;
}

// Parses one $--GGA sentence (checksum, if present, already excluded or
// included -- either way, trailing junk after '*' is stripped here).
bool parseGga(const String &rawLine) {
  int starIndex = rawLine.indexOf('*');
  String line = (starIndex >= 0) ? rawLine.substring(0, starIndex) : rawLine;
  line += ",";  // guarantees the final field terminates cleanly below

  String fields[15];
  int fieldCount = 0;
  int fieldStart = 0;
  for (int i = 0; i < static_cast<int>(line.length()) && fieldCount < 15; i++) {
    if (line[i] == ',') {
      fields[fieldCount++] = line.substring(fieldStart, i);
      fieldStart = i + 1;
    }
  }
  if (fieldCount < 10) return false;
  if (fields[0].indexOf("GGA") < 0) return false;

  int fixQuality = fields[6].toInt();
  int numSat = fields[7].toInt();
  bool hasFix = fixQuality > 0 && fields[2].length() > 0 && fields[4].length() > 0;

  gpsState.fixQuality = fixQuality;
  gpsState.numSatellites = numSat;
  gpsState.hasFix = hasFix;

  if (hasFix) {
    float lat = nmeaToDecimalDegrees(fields[2], 2);
    if (fields[3] == "S") lat = -lat;
    float lon = nmeaToDecimalDegrees(fields[4], 3);
    if (fields[5] == "W") lon = -lon;
    gpsState.latDeg = lat;
    gpsState.lonDeg = lon;
    gpsState.altM = fields[9].toFloat();
  }
  return true;
}

void pollGps() {
  while (gpsSerial.available()) {
    char c = static_cast<char>(gpsSerial.read());
    if (c == '\n') {
      gpsLineBuffer.trim();
      if (gpsLineBuffer.startsWith("$") && gpsLineBuffer.indexOf("GGA") >= 0) {
        parseGga(gpsLineBuffer);
      }
      gpsLineBuffer = "";
    } else if (c != '\r') {
      gpsLineBuffer += c;
      if (gpsLineBuffer.length() > 120) {
        gpsLineBuffer = "";  // malformed/overlong -- drop and resync
      }
    }
  }
}

// ---------------------------------------------------------------------
// Manual jog dial -- open-loop velocity jog + dead-man timeout, matching the
// old Nucleo firmware's applyJogMotion()/maybeStopEncoderJog() instead of
// nudging a closed-loop target (see the ENCODER_JOG_HOLD_MS comment above
// for why: that avoids the tolerance-band bang-bang overshoot entirely by
// never doing position-based stopping for jog in the first place).
// ---------------------------------------------------------------------

void applyEncoderJog() {
  long azNow, elNow;
  noInterrupts();
  azNow = azEncoderCount;
  elNow = elEncoderCount;
  interrupts();

  long azSteps = azNow - azAppliedCount;
  long elSteps = elNow - elAppliedCount;

  if (azSteps != 0) {
    azAppliedCount = azNow;
    azJogDir = (azSteps > 0) ? 1 : -1;
    jogHoldUntilMs = millis() + ENCODER_JOG_HOLD_MS;
    rotor.hasTarget = false;  // a fresh jog always overrides any pending TARGET
  }
  if (elSteps != 0) {
    elAppliedCount = elNow;
    elJogDir = (elSteps > 0) ? 1 : -1;
    jogHoldUntilMs = millis() + ENCODER_JOG_HOLD_MS;
    rotor.hasTarget = false;
  }

  if (jogHoldUntilMs == 0) return;  // no jog session active, nothing to do

  if (millis() >= jogHoldUntilMs) {
    // Dead-man timeout: no fresh encoder activity for ENCODER_JOG_HOLD_MS --
    // stop and end the jog session.
    pelcoStop();
    azJogDir = 0;
    elJogDir = 0;
    jogHoldUntilMs = 0;
    rotor.isMoving = false;
    return;
  }

  uint8_t cmd2 = 0x00;
  uint8_t data1 = 0x00;
  uint8_t data2 = 0x00;

  if (azJogDir != 0) {
    cmd2 |= (azJogDir > 0) ? 0x02 : 0x04;  // pan right : pan left
    data1 = panSpeed;
  }
  if (elJogDir != 0) {
    // Same invert convention as driveTowardTarget()'s tilt bits.
    bool wantUp = elJogDir > 0;
    bool commandUp = INVERT_ELEVATION_USING_NEGATIVE_TILT ? !wantUp : wantUp;
    cmd2 |= commandUp ? 0x10 : 0x08;
    data2 = tiltSpeed;
  }

  pelcoSend(0x00, cmd2, data1, data2);
  rotor.isMoving = true;
}

// ---------------------------------------------------------------------
// Serial command handling (UNO Q link)
// ---------------------------------------------------------------------

void sendPositionReply() {
  Serial.print("POS ");
  Serial.print(rotor.currentAz, 2);
  Serial.print(" ");
  Serial.print(rotor.currentEl, 2);
  Serial.print(" ");
  Serial.println(rotor.isMoving ? 1 : 0);
}

void sendGpsReply() {
  if (gpsState.hasFix) {
    Serial.print("GPS ");
    Serial.print(gpsState.latDeg, 6);
    Serial.print(" ");
    Serial.print(gpsState.lonDeg, 6);
    Serial.print(" ");
    Serial.print(gpsState.altM, 1);
    Serial.print(" ");
    Serial.print(gpsState.fixQuality);
    Serial.print(" ");
    Serial.println(gpsState.numSatellites);
  } else {
    Serial.print("GPS NOFIX ");
    Serial.print(gpsState.fixQuality);
    Serial.print(" ");
    Serial.println(gpsState.numSatellites);
  }
}

void sendEncoderReply() {
  noInterrupts();
  long az = azEncoderCount;
  long el = elEncoderCount;
  interrupts();
  Serial.print("ENC ");
  Serial.print(az);
  Serial.print(" ");
  Serial.println(el);
}

void handleLine(const String &line) {
  if (line == "POS?") {
    sendPositionReply();
    return;
  }
  if (line == "STOP") {
    rotor.hasTarget = false;
    azJogDir = 0;
    elJogDir = 0;
    jogHoldUntilMs = 0;
    pelcoStop();
    rotor.isMoving = false;
    sendPositionReply();
    return;
  }
  if (line == "GPS?") {
    sendGpsReply();
    return;
  }
  if (line == "ENC?") {
    sendEncoderReply();
    return;
  }
  if (line == "SKIP_STARTUP") {
    // Bench/development override ONLY: use this after reflashing the GIGA
    // while the head itself stayed powered on the whole time (so it's
    // already well past its own startup dance) -- reflashing alone
    // shouldn't re-impose a fake 95s wait. Do NOT send this after a real
    // power-up where the head and GIGA booted together -- it hasn't
    // actually finished its dance yet in that case.
    bootMs = millis() - HEAD_STARTUP_MS - 1;
    Serial.println("Startup wait skipped -- polling/driving active now.");
    return;
  }
  if (line == "HEAD_POWERED_ON") {
    // Re-arms the real 95s wait from right now. Not needed after a normal
    // shared power-up (setup() already arms it) -- this is only for the
    // rare case where you power-cycle just the head while the GIGA keeps
    // running (e.g. cycling the 12V head supply without touching the GIGA).
    bootMs = millis();
    Serial.println("Startup wait armed -- polling/driving paused for 95s.");
    return;
  }
  if (line == "SPEED?") {
    Serial.print("SPEED pan=");
    Serial.print(panSpeed);
    Serial.print(" (0x");
    Serial.print(panSpeed, HEX);
    Serial.print(") panMin=");
    Serial.print(panMinSpeed);
    Serial.print(" tilt=");
    Serial.print(tiltSpeed);
    Serial.print(" (0x");
    Serial.print(tiltSpeed, HEX);
    Serial.println(")");
    return;
  }
  if (line.startsWith("SPEED PAN ")) {
    int value = atoi(line.c_str() + 10);
    panSpeed = static_cast<uint8_t>(constrain(value, 0, 63));
    Serial.print("SPEED pan set to ");
    Serial.println(panSpeed);
    return;
  }
  if (line.startsWith("SPEED TILT ")) {
    int value = atoi(line.c_str() + 11);
    tiltSpeed = static_cast<uint8_t>(constrain(value, 0, 63));
    Serial.print("SPEED tilt set to ");
    Serial.println(tiltSpeed);
    return;
  }
  if (line.startsWith("SPEED PANMIN ")) {
    // Deceleration floor for TARGET moves' final approach -- test any new
    // value briefly and listen/watch for grinding before trusting it.
    int value = atoi(line.c_str() + 13);
    panMinSpeed = static_cast<uint8_t>(constrain(value, 0, 63));
    Serial.print("SPEED panMin set to ");
    Serial.println(panMinSpeed);
    return;
  }
  if (line.startsWith("AZSET ")) {
    // "Wherever the head is physically pointed right now, in reality it's
    // this azimuth" -- resyncs azPhaseOffsetDeg so decodePan() reports the
    // given value from now on, without needing to know or compute the
    // offset arithmetic by hand. Does NOT move anything or touch the head
    // at all -- purely corrects our one fixed assumption about the
    // relationship between the head's raw pan count and true north.
    float realAz = atof(line.c_str() + 6);
    azPhaseOffsetDeg = normalizeAz(azPhaseOffsetDeg + (rotor.currentAz - realAz));
    rotor.currentAz = normalizeAz(realAz);  // reflect it immediately, don't wait for next poll
    Serial.print("AZSET applied -- azPhaseOffsetDeg now ");
    Serial.print(azPhaseOffsetDeg, 2);
    Serial.print(", currentAz now ");
    Serial.println(rotor.currentAz, 2);
    return;
  }
  if (line.startsWith("ELSET ")) {
    // Same idea as AZSET, other axis: "wherever the head is physically
    // pointed right now, in reality it's this elevation." Note the sign is
    // opposite AZSET's because elOffsetDeg is ADDED in decodeTilt (vs.
    // SUBTRACTED in decodePan) -- verified against the math, don't copy
    // AZSET's formula here without re-deriving it.
    float realEl = atof(line.c_str() + 6);
    elOffsetDeg = elOffsetDeg + (realEl - rotor.currentEl);
    rotor.currentEl = clampFloat(realEl, EL_MIN_DEG, EL_MAX_DEG);
    Serial.print("ELSET applied -- elOffsetDeg now ");
    Serial.print(elOffsetDeg, 2);
    Serial.print(", currentEl now ");
    Serial.println(rotor.currentEl, 2);
    return;
  }
  if (line == "TOL?") {
    Serial.print("TOL stop=");
    Serial.print(positionToleranceDeg, 2);
    Serial.print(" resume=");
    Serial.println(resumeToleranceDeg, 2);
    return;
  }
  if (line.startsWith("TOL STOP ")) {
    positionToleranceDeg = atof(line.c_str() + 9);
    Serial.print("TOL stop set to ");
    Serial.println(positionToleranceDeg, 2);
    return;
  }
  if (line.startsWith("TOL RESUME ")) {
    resumeToleranceDeg = atof(line.c_str() + 11);
    Serial.print("TOL resume set to ");
    Serial.println(resumeToleranceDeg, 2);
    return;
  }
  if (line == "JOGUP" || line == "JOGDOWN") {
    // Bounded, one-shot manual nudge -- sends exactly one direction for
    // 300ms then auto-stops. NOT the closed-loop TARGET system -- this is
    // purely so you can watch, with your own eyes, which physical direction
    // each raw bit actually produces on THIS head, instead of guessing.
    // 0x08 is the generic-Pelco-D "tilt up" bit, 0x10 is "tilt down" -- but
    // don't assume that holds for this head; that's exactly what this test
    // is for.
    uint8_t bit = (line == "JOGUP") ? 0x08 : 0x10;
    pelcoSend(0x00, bit, 0x00, tiltSpeed);
    delay(300);
    pelcoStop();
    Serial.print(line);
    Serial.print(" sent (cmd2=0x");
    Serial.print(bit, HEX);
    Serial.println(") for 300ms, then stopped.");
    return;
  }
  if (line.startsWith("PRESET ")) {
    // Diagnostic only: sends a raw Pelco-D "call preset" command (cmd2=0x07,
    // preset number in data2), matching the extended-command convention from
    // the old W6PS SatRotor project's PelcoD.cpp callPreset(). This is how
    // preset 125 -- documented in this head's own manual as "restore default
    // parameters, open self test function of the pan tilt" -- gets tested.
    // CAUTION: per the manual, preset 125 likely kicks off the same startup/
    // self-test dance this head runs after power-up, including relying on
    // the spoof board's fixed-timing limit clicks. Be ready to babysit it
    // through the whole sequence and hit the physical limit switch by hand
    // if it blows past the stop point again, same as a cold power-up.
    int presetNum = atoi(line.c_str() + 7);
    pelcoSend(0x00, 0x07, 0x00, static_cast<uint8_t>(presetNum));
    Serial.print("PRESET sent: ");
    Serial.println(presetNum);
    return;
  }
  if (line == "QUERY") {
    // On-demand diagnostic: forces a REAL Pelco-D query out on D1 right now,
    // regardless of the 95s HEAD_STARTUP_MS gate. Unlike POS?, this does not
    // read cached rotor state -- it actually calls pelcoQuery() and reports
    // exactly what came back (or didn't). Use this with a logic analyzer on
    // D0/D1 for a deterministic, guaranteed-to-transmit test.
    uint16_t rawPan = 0, rawTilt = 0;
    bool gotPan = pelcoQuery(0x51, 0x59, rawPan);
    bool gotTilt = pelcoQuery(0x53, 0x5B, rawTilt);
    Serial.print("QUERY pan=");
    Serial.print(gotPan ? "OK" : "FAIL");
    Serial.print(" raw=");
    Serial.print(rawPan);
    Serial.print(" tilt=");
    Serial.print(gotTilt ? "OK" : "FAIL");
    Serial.print(" raw=");
    Serial.println(rawTilt);
    return;
  }
  if (line.startsWith("TARGET ")) {
    float az = 0.0f, el = 0.0f;
    int parsed = sscanf(line.c_str() + 7, "%f %f", &az, &el);
    if (parsed == 2) {
      rotor.targetAz = normalizeAz(az);
      rotor.targetEl = clampFloat(el, EL_MIN_DEG, EL_MAX_DEG);
      rotor.hasTarget = true;
    }
    sendPositionReply();
    return;
  }
  // Unrecognized line -- ignore rather than wedge the parser.
}

// ---------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  pelcoSerial.begin(PELCO_BAUD, SERIAL_8N1);
  gpsSerial.begin(GPS_BAUD);

  pinMode(AZ_ENC_A_PIN, INPUT);
  pinMode(AZ_ENC_B_PIN, INPUT);
  pinMode(EL_ENC_A_PIN, INPUT);
  pinMode(EL_ENC_B_PIN, INPUT);

  // Let the 5V->3.3V encoder buffer board's outputs settle before trusting
  // any transitions -- a noisy/undefined burst right at power-up was seen
  // driving elevation all the way to its hard limit stop the instant the AZ
  // encoder was first touched after a fresh boot, only ever on that first
  // touch. Reading the real starting pin state here (instead of assuming
  // both start at 0) also avoids misattributing whatever the very first
  // genuine transition turns out to be.
  delay(100);
  azQuadState = (digitalRead(AZ_ENC_A_PIN) << 1) | digitalRead(AZ_ENC_B_PIN);
  elQuadState = (digitalRead(EL_ENC_A_PIN) << 1) | digitalRead(EL_ENC_B_PIN);

  // Real quadrature decode needs both channels watched, not just A.
  attachInterrupt(digitalPinToInterrupt(AZ_ENC_A_PIN), azEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(AZ_ENC_B_PIN), azEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EL_ENC_A_PIN), elEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EL_ENC_B_PIN), elEncoderIsr, CHANGE);

  // Discard anything that piled up as pins were configured/settling above --
  // none of it was a real user-driven encoder turn.
  noInterrupts();
  azEncoderCount = 0;
  elEncoderCount = 0;
  interrupts();
  azAppliedCount = 0;
  elAppliedCount = 0;

  // Default assumption: this is a real power-up, head and GIGA together
  // from a full shutdown (the normal, deployed case) -- so arm the real
  // 95s wait from boot, same as always. If you're bench-testing and only
  // reflashing the GIGA while the head stays powered on, send
  // "SKIP_STARTUP" after each reflash to bypass the fake re-wait.
  bootMs = millis();
}

void loop() {
  // GPS, encoder jog input, and the UNO Q command link all run regardless
  // of the head's startup wait below -- only the Pelco-D polling/driving
  // needs to hold off. Turning the encoders during that wait just stages up
  // a target; driveTowardTarget() picks it up once the wait clears.
  pollGps();
  applyEncoderJog();

  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      lineBuffer.trim();
      if (lineBuffer.length() > 0) handleLine(lineBuffer);
      lineBuffer = "";
    } else if (c != '\r') {
      lineBuffer += c;
    }
  }

  // This head ignores everything for ~95s after power-up. Don't poll or
  // drive during that window -- just wait it out.
  if (millis() - bootMs < HEAD_STARTUP_MS) {
    return;
  }

  uint32_t now = millis();
  if (now - lastPollMs >= POLL_INTERVAL_MS) {
    lastPollMs = now;
    // pollPosition()'s return value matters: rotor.hasPosition latches true
    // forever after the first good read, so a later missed/partial query
    // would otherwise silently leave driveTowardTarget() computing az/el
    // error against stale currentAz/currentEl -- a second contributor to
    // the intermittent TARGET waggle (also flagged by Codex, 2026-08-02).
    // On a bad poll, stop rather than drive blind on old data; the next
    // good poll will resume driving normally since hasTarget is untouched.
    bool freshPosition = pollPosition();
    if (rotor.hasTarget) {
      if (freshPosition) {
        driveTowardTarget();
      } else {
        pelcoStop();
        rotor.isMoving = false;
      }
    }
  }
}
