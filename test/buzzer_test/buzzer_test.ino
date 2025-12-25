#include <Arduino.h>

constexpr uint8_t BUZZER_PIN = 4;

// Set to 1 for active buzzer, 0 for passive buzzer.
#ifndef BUZZER_ACTIVE
#define BUZZER_ACTIVE 1
#endif

constexpr uint16_t kToneHz = 2000;
constexpr uint32_t kOnMs = 200;
constexpr uint32_t kOffMs = 200;

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
#if BUZZER_ACTIVE
  digitalWrite(BUZZER_PIN, HIGH);
  delay(kOnMs);
  digitalWrite(BUZZER_PIN, LOW);
  delay(kOffMs);
#else
  tone(BUZZER_PIN, kToneHz, kOnMs);
  delay(kOnMs + kOffMs);
#endif
}
