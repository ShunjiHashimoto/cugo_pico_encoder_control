#include <Arduino.h>

// Set to 1 for left motor, 0 for right motor.
#ifndef USE_LEFT_MOTOR
#define USE_LEFT_MOTOR 0
#endif

constexpr uint8_t PIN_MOTOR_L_PWM = 17;
constexpr uint8_t PIN_MOTOR_L_DIR = 16;
constexpr uint8_t PIN_MOTOR_R_PWM = 19;
constexpr uint8_t PIN_MOTOR_R_DIR = 18;
constexpr uint8_t PIN_SW1 = 14;

// Duty ratio (0.0 to 1.0). Keep it low for testing (e.g., 0.1 or 0.2).
constexpr float kDutyRatio = 0.1f;
constexpr bool kForward = false;
constexpr uint32_t kDebounceMs = 20;

constexpr uint16_t kPwmRange = 255;
constexpr uint32_t kBaud = 115200;

uint8_t pwm_pin = 0;
uint8_t dir_pin = 0;
bool motor_on = false;
unsigned long last_toggle_ms = 0;

void setup() {
  Serial.begin(kBaud);

#if USE_LEFT_MOTOR
  pwm_pin = PIN_MOTOR_L_PWM;
  dir_pin = PIN_MOTOR_L_DIR;
  Serial.println("Motor: LEFT");
#else
  pwm_pin = PIN_MOTOR_R_PWM;
  dir_pin = PIN_MOTOR_R_DIR;
  Serial.println("Motor: RIGHT");
#endif

  pinMode(pwm_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(PIN_SW1, INPUT_PULLUP);
  analogWriteRange(kPwmRange);

  int duty = static_cast<int>(kDutyRatio * kPwmRange + 0.5f);
  if (duty < 0) duty = 0;
  if (duty > kPwmRange) duty = kPwmRange;

  digitalWrite(dir_pin, kForward ? HIGH : LOW);
  analogWrite(pwm_pin, 0);

  Serial.print("Duty ratio: ");
  Serial.print(kDutyRatio, 2);
  Serial.print(" (");
  Serial.print(duty);
  Serial.println(" / 255)");
  Serial.println("Press SW1 to toggle motor.");
}

void loop() {
  static int last_state = HIGH;
  int state = digitalRead(PIN_SW1);
  unsigned long now = millis();
  if (state == LOW && last_state == HIGH && (now - last_toggle_ms) > kDebounceMs) {
    motor_on = !motor_on;
    last_toggle_ms = now;
    int duty = static_cast<int>(kDutyRatio * kPwmRange + 0.5f);
    if (duty < 0) duty = 0;
    if (duty > kPwmRange) duty = kPwmRange;
    analogWrite(pwm_pin, motor_on ? duty : 0);
    Serial.print("Motor ");
    Serial.println(motor_on ? "on" : "off");
  }
  last_state = state;
  delay(5);
}
