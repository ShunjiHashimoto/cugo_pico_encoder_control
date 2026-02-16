#include <Arduino.h>

constexpr uint8_t PIN_RC_L = 10;  // GP10: RC receiver signal for left motor
constexpr uint8_t PIN_RC_R = 11;  // GP11: RC receiver signal for right motor

constexpr uint8_t PIN_MOTOR_L_PWM = 17;
constexpr uint8_t PIN_MOTOR_L_DIR = 16;
constexpr uint8_t PIN_MOTOR_R_PWM = 19;
constexpr uint8_t PIN_MOTOR_R_DIR = 18;

constexpr uint16_t kPwmRange = 255;
constexpr uint32_t kBaud = 115200;

constexpr uint32_t kPulseTimeoutUs = 25000;
constexpr int kPulseMinUs = 1000;
constexpr int kPulseMaxUs = 2000;
constexpr int kNeutralMinUs = 1480;
constexpr int kNeutralMaxUs = 1520;

struct MotorPins {
  uint8_t pwm;
  uint8_t dir;
};

MotorPins left_motor{PIN_MOTOR_L_PWM, PIN_MOTOR_L_DIR};
MotorPins right_motor{PIN_MOTOR_R_PWM, PIN_MOTOR_R_DIR};

int readPulseUs(uint8_t pin) {
  return static_cast<int>(pulseIn(pin, HIGH, kPulseTimeoutUs));
}

void applyPulseToMotor(int pulse_us, const MotorPins &motor) {
  if (pulse_us < kPulseMinUs || pulse_us > kPulseMaxUs) {
    analogWrite(motor.pwm, 0);
    digitalWrite(motor.dir, LOW);
    return;
  }

  if (pulse_us >= kNeutralMinUs && pulse_us <= kNeutralMaxUs) {
    analogWrite(motor.pwm, 0);
    digitalWrite(motor.dir, LOW);
    return;
  }

  if (pulse_us > kNeutralMaxUs) {
    int speed = static_cast<int>(map(pulse_us, kNeutralMaxUs, kPulseMaxUs, 0, kPwmRange));
    speed = constrain(speed, 0, kPwmRange);
    digitalWrite(motor.dir, HIGH);
    analogWrite(motor.pwm, speed);
  } else {
    int speed = static_cast<int>(map(pulse_us, kNeutralMinUs, kPulseMinUs, 0, kPwmRange));
    speed = constrain(speed, 0, kPwmRange);
    digitalWrite(motor.dir, LOW);
    analogWrite(motor.pwm, speed);
  }
}

void setup() {
  Serial.begin(kBaud);

  pinMode(PIN_RC_L, INPUT);
  pinMode(PIN_RC_R, INPUT);

  pinMode(PIN_MOTOR_L_PWM, OUTPUT);
  pinMode(PIN_MOTOR_L_DIR, OUTPUT);
  pinMode(PIN_MOTOR_R_PWM, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR, OUTPUT);

  analogWriteRange(kPwmRange);

  analogWrite(PIN_MOTOR_L_PWM, 0);
  analogWrite(PIN_MOTOR_R_PWM, 0);
  digitalWrite(PIN_MOTOR_L_DIR, LOW);
  digitalWrite(PIN_MOTOR_R_DIR, LOW);

  Serial.println("RC motor test: GP10->left, GP11->right");
}

void loop() {
  int pulse_l = readPulseUs(PIN_RC_L);
  int pulse_r = readPulseUs(PIN_RC_R);

  bool l_ok = pulse_l >= kPulseMinUs && pulse_l <= kPulseMaxUs;
  bool r_ok = pulse_r >= kPulseMinUs && pulse_r <= kPulseMaxUs;

  Serial.print("L:");
  Serial.print(pulse_l);
  Serial.print(l_ok ? "us" : "us (no signal)");
  Serial.print(" R:");
  Serial.print(pulse_r);
  Serial.print(r_ok ? "us" : "us (no signal)");
  Serial.println();

  delay(50);
}
