#include <Arduino.h>

// Set to 1 for left motor, 0 for right motor.
#ifndef USE_LEFT_MOTOR
#define USE_LEFT_MOTOR 0
#endif

// 0: DC motor driver (cugov3 style)
// 1: BLDC driver HM-5100J (FWD/REV)
// 2: BLDC driver HP-5097 (FWD-only direction select)
#ifndef DRIVER_TYPE
#define DRIVER_TYPE 2
#endif

#define DRIVER_DC 0
#define DRIVER_BLDC_HM5100J 1
#define DRIVER_BLDC_HP5097 2

constexpr uint8_t PIN_MOTOR_L_PWM = 17;
constexpr uint8_t PIN_MOTOR_R_PWM = 19;
constexpr uint8_t PIN_MOTOR_L_DIR_DC = 16;
constexpr uint8_t PIN_MOTOR_R_DIR_DC = 18;
constexpr uint8_t PIN_MOTOR_L_FWD = 2;
constexpr uint8_t PIN_MOTOR_L_REV = 3;
constexpr uint8_t PIN_MOTOR_R_FWD = 8;
constexpr uint8_t PIN_MOTOR_R_REV = 9;
constexpr uint8_t PIN_SW1 = 15;

// Duty ratio (0.0 to 1.0). Keep it low for testing (e.g., 0.1 or 0.2).
constexpr float kDutyRatio = 0.1f;
constexpr bool kForward = false;
constexpr uint32_t kDebounceMs = 20;

constexpr uint16_t kPwmRange = 255;
constexpr uint32_t kBaud = 115200;

uint8_t pwm_pin = 0;
uint8_t dir_fwd_pin = 0;
uint8_t dir_rev_pin = 255;  // 255 means unused.
bool motor_on = false;
unsigned long last_toggle_ms = 0;

int calcDuty() {
  int duty = static_cast<int>(kDutyRatio * kPwmRange + 0.5f);
  if (duty < 0) duty = 0;
  if (duty > kPwmRange) duty = kPwmRange;
  return duty;
}

void setDirectionPins(bool forward) {
#if DRIVER_TYPE == DRIVER_DC
  digitalWrite(dir_fwd_pin, forward ? HIGH : LOW);
#elif DRIVER_TYPE == DRIVER_BLDC_HM5100J
  digitalWrite(dir_fwd_pin, forward ? HIGH : LOW);
  digitalWrite(dir_rev_pin, forward ? LOW : HIGH);
#elif DRIVER_TYPE == DRIVER_BLDC_HP5097
  digitalWrite(dir_fwd_pin, forward ? HIGH : LOW);
  digitalWrite(dir_rev_pin, LOW);
#else
#error "Unsupported DRIVER_TYPE. Use 0, 1, or 2."
#endif
}

void stopMotorOutput() {
  analogWrite(pwm_pin, 0);
  digitalWrite(dir_fwd_pin, LOW);
  if (dir_rev_pin != 255) {
    digitalWrite(dir_rev_pin, LOW);
  }
}

void setup() {
  Serial.begin(kBaud);

#if USE_LEFT_MOTOR
  pwm_pin = PIN_MOTOR_L_PWM;
  Serial.println("Motor: LEFT");
#if DRIVER_TYPE == DRIVER_DC
  dir_fwd_pin = PIN_MOTOR_L_DIR_DC;
#else
  dir_fwd_pin = PIN_MOTOR_L_FWD;
  dir_rev_pin = PIN_MOTOR_L_REV;
#endif
#else
  pwm_pin = PIN_MOTOR_R_PWM;
  Serial.println("Motor: RIGHT");
#if DRIVER_TYPE == DRIVER_DC
  dir_fwd_pin = PIN_MOTOR_R_DIR_DC;
#else
  dir_fwd_pin = PIN_MOTOR_R_FWD;
  dir_rev_pin = PIN_MOTOR_R_REV;
#endif
#endif

#if DRIVER_TYPE == DRIVER_DC
  Serial.println("Driver: DC");
#elif DRIVER_TYPE == DRIVER_BLDC_HM5100J
  Serial.println("Driver: BLDC HM-5100J");
#elif DRIVER_TYPE == DRIVER_BLDC_HP5097
  Serial.println("Driver: BLDC HP-5097");
#endif

  pinMode(pwm_pin, OUTPUT);
  pinMode(dir_fwd_pin, OUTPUT);
  if (dir_rev_pin != 255) {
    pinMode(dir_rev_pin, OUTPUT);
  }
  pinMode(PIN_SW1, INPUT_PULLUP);
  analogWriteRange(kPwmRange);
  stopMotorOutput();

  Serial.print("Duty ratio: ");
  Serial.print(kDutyRatio, 2);
  Serial.print(" (");
  Serial.print(calcDuty());
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
    if (motor_on) {
      setDirectionPins(kForward);
      analogWrite(pwm_pin, calcDuty());
    } else {
      stopMotorOutput();
    }
    Serial.print("Motor ");
    Serial.println(motor_on ? "on" : "off");
  }
  last_state = state;
  delay(5);
}
