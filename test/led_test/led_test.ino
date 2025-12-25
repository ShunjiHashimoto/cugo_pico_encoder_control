#include <Arduino.h>

constexpr uint8_t LED1_PIN = 21;
constexpr uint8_t LED2_PIN = 22;

// Change LED_TEST_PIN to LED2_PIN to verify LED2 instead of LED1.
#ifndef LED_TEST_PIN
#define LED_TEST_PIN LED2_PIN
#endif

constexpr uint32_t kBlinkMs = 300;

void setup() {
  pinMode(LED_TEST_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_TEST_PIN, HIGH);
  delay(kBlinkMs);
  digitalWrite(LED_TEST_PIN, LOW);
  delay(kBlinkMs);
}
