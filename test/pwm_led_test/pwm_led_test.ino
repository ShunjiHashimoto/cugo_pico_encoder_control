#include <Arduino.h>

constexpr uint8_t LED_PWM_PIN = 20;

// Set LED_PWM_DUTY (0-255) at build time, e.g. -DLED_PWM_DUTY=64.
#ifndef LED_PWM_DUTY
#define LED_PWM_DUTY 128
#endif

constexpr uint16_t kPwmRange = 255;

constexpr uint32_t kBaud = 115200;

void setup() {
  Serial.begin(kBaud);
  pinMode(LED_PWM_PIN, OUTPUT);
  analogWriteRange(kPwmRange);
  int duty = static_cast<int>(LED_PWM_DUTY);
  if (duty < 0) {
    duty = 0;
  } else if (duty > kPwmRange) {
    duty = kPwmRange;
  }
  analogWrite(LED_PWM_PIN, duty);
  Serial.print("LED PWM duty: ");
  Serial.println(duty);
}

void loop() {
  delay(1000);
}
