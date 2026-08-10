/*
  PTZ azimuth limit spoof board

  Hardware:
  - Pro Mini / ATmega328PB 5V board
  - 2-channel 5V relay board
  - Relay board set to HIGH-level trigger
  - D2 -> IN1 (first azimuth trigger)
  - D3 -> IN2 (second azimuth trigger)

  Behavior:
  - Keep both relays OFF at boot so both NC switch loops remain closed.
  - Wait for the head to begin startup motion.
  - Pulse relay 1 (Spoof A) to briefly open loop 1.
  - Pulse relay 2 (Spoof B) for the early reverse pan leg.
  - Wait through the elevation dance.
  - Pulse relay 2 (Spoof B) again at 01:36.30 after power-up.
  - Stay idle forever after that.
*/

const uint8_t RELAY1_PIN = 2;
const uint8_t RELAY2_PIN = 3;
const uint8_t LED_PIN = LED_BUILTIN;

const bool RELAY_ACTIVE_HIGH = true;

const unsigned long RELAY_PULSE_MS = 200;
const unsigned long FIRST_SPOOF_A_MS = 3000;
const unsigned long SECOND_SPOOF_B_MS = 5200;
const unsigned long THIRD_SPOOF_B_MS = 96300;

void setRelay(uint8_t pin, bool active) {
  digitalWrite(pin, (active == RELAY_ACTIVE_HIGH) ? HIGH : LOW);
}

void waitUntilMs(unsigned long targetMs) {
  while (millis() < targetMs) {
    delay(10);
  }
}

void pulseRelay(uint8_t pin, unsigned long pulseMs) {
  setRelay(pin, true);
  digitalWrite(LED_PIN, HIGH);
  delay(pulseMs);
  setRelay(pin, false);
  digitalWrite(LED_PIN, LOW);
}

void setup() {
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Default safe state: relays off, NC loops remain closed.
  setRelay(RELAY1_PIN, false);
  setRelay(RELAY2_PIN, false);
  digitalWrite(LED_PIN, LOW);

  waitUntilMs(FIRST_SPOOF_A_MS);
  pulseRelay(RELAY1_PIN, RELAY_PULSE_MS);

  waitUntilMs(SECOND_SPOOF_B_MS);
  pulseRelay(RELAY2_PIN, RELAY_PULSE_MS);

  waitUntilMs(THIRD_SPOOF_B_MS);
  pulseRelay(RELAY2_PIN, RELAY_PULSE_MS);
}

void loop() {
  // Stay idle. Keep both relays off forever after the startup sequence.
  setRelay(RELAY1_PIN, false);
  setRelay(RELAY2_PIN, false);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}
