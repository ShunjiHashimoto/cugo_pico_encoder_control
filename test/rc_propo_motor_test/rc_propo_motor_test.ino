#include <Arduino.h>

// RC receiver inputs (left/right command channels)
constexpr uint8_t PIN_RC_L = 10;  // GP10
constexpr uint8_t PIN_RC_R = 11;  // GP11

// CuGoV4 motor outputs (BLVD10KM driver: PWM + FWD/REV)
constexpr uint8_t PIN_MOTOR_L_PWM = 17;      // PWM1_L
constexpr uint8_t PIN_MOTOR_L_DIR_FWD = 2;   // DIR1_L_FWD
constexpr uint8_t PIN_MOTOR_L_DIR_REV = 3;   // DIR1_L_REV
constexpr uint8_t PIN_MOTOR_R_PWM = 19;      // PWM1_R
constexpr uint8_t PIN_MOTOR_R_DIR_FWD = 8;   // DIR2_R_FWD
constexpr uint8_t PIN_MOTOR_R_DIR_REV = 9;   // DIR2_R_REV

constexpr uint16_t kPwmRange = 600;
constexpr uint32_t kPwmFreqHz = 20000;
constexpr uint32_t kBaud = 115200;

constexpr uint32_t kPulseTimeoutUs = 25000;
constexpr int kPulseMinUs = 1000;
constexpr int kPulseCenterUs = 1500;
constexpr int kPulseMaxUs = 2000;
constexpr int kNeutralDeadbandUs = 20;
constexpr uint32_t kSignalTimeoutMs = 100;
constexpr uint32_t kPrintIntervalMs = 100;

// Match cugov4 main sketch polarity default:
// left motor command is inverted, right is normal.
constexpr bool kInvertLeft = true;
constexpr bool kInvertRight = false;

struct MotorPins {
  uint8_t pwm;
  uint8_t dir_fwd;
  uint8_t dir_rev;
};

MotorPins left_motor{PIN_MOTOR_L_PWM, PIN_MOTOR_L_DIR_FWD, PIN_MOTOR_L_DIR_REV};
MotorPins right_motor{PIN_MOTOR_R_PWM, PIN_MOTOR_R_DIR_FWD, PIN_MOTOR_R_DIR_REV};

uint32_t last_valid_left_ms = 0;
uint32_t last_valid_right_ms = 0;
uint32_t last_print_ms = 0;

int readPulseUs(uint8_t pin) {
  return static_cast<int>(pulseIn(pin, HIGH, kPulseTimeoutUs));
}

void stopMotor(const MotorPins &motor) {
  analogWrite(motor.pwm, 0);
  digitalWrite(motor.dir_fwd, LOW);
  digitalWrite(motor.dir_rev, LOW);
}

void applySignedPwmToMotor(int signed_pwm, const MotorPins &motor) {
  int pwm = abs(signed_pwm);
  if (pwm > static_cast<int>(kPwmRange)) {
    pwm = static_cast<int>(kPwmRange);
  }
  if (pwm == 0) {
    stopMotor(motor);
    return;
  }

  if (signed_pwm > 0) {
    digitalWrite(motor.dir_fwd, HIGH);
    digitalWrite(motor.dir_rev, LOW);
  } else {
    digitalWrite(motor.dir_fwd, LOW);
    digitalWrite(motor.dir_rev, HIGH);
  }
  analogWrite(motor.pwm, pwm);
}

bool isPulseValid(int pulse_us) {
  return pulse_us >= kPulseMinUs && pulse_us <= kPulseMaxUs;
}

int pulseToSignedPwm(int pulse_us, bool invert) {
  if (pulse_us < kPulseMinUs || pulse_us > kPulseMaxUs) {
    return 0;
  }

  int delta = pulse_us - kPulseCenterUs;
  if (abs(delta) <= kNeutralDeadbandUs) {
    return 0;
  }

  int min_cmd = kPulseCenterUs + kNeutralDeadbandUs;
  int max_cmd = kPulseMaxUs;
  int min_rev = kPulseCenterUs - kNeutralDeadbandUs;
  int max_rev = kPulseMinUs;

  int signed_pwm = 0;
  if (delta > 0) {
    signed_pwm = map(pulse_us, min_cmd, max_cmd, 0, static_cast<int>(kPwmRange));
  } else {
    signed_pwm = -map(pulse_us, min_rev, max_rev, 0, static_cast<int>(kPwmRange));
  }

  signed_pwm = constrain(signed_pwm, -static_cast<int>(kPwmRange), static_cast<int>(kPwmRange));
  if (invert) {
    signed_pwm = -signed_pwm;
  }
  return signed_pwm;
}

void setup() {
  Serial.begin(kBaud);

  pinMode(PIN_RC_L, INPUT);
  pinMode(PIN_RC_R, INPUT);

  pinMode(PIN_MOTOR_L_PWM, OUTPUT);
  pinMode(PIN_MOTOR_L_DIR_FWD, OUTPUT);
  pinMode(PIN_MOTOR_L_DIR_REV, OUTPUT);
  pinMode(PIN_MOTOR_R_PWM, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR_FWD, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR_REV, OUTPUT);

  analogWriteRange(kPwmRange);
  analogWriteFreq(kPwmFreqHz);

  stopMotor(left_motor);
  stopMotor(right_motor);

  Serial.println("RC motor test for CuGoV4");
  Serial.println("CH_L: GP10 -> Left motor, CH_R: GP11 -> Right motor");
  Serial.println("No valid pulse => motor stop");
  uint32_t now_ms = millis();
  last_valid_left_ms = now_ms;
  last_valid_right_ms = now_ms;
  last_print_ms = now_ms;
}

void loop() {
  int pulse_l = readPulseUs(PIN_RC_L);
  int pulse_r = readPulseUs(PIN_RC_R);
  uint32_t now_ms = millis();

  bool l_ok = isPulseValid(pulse_l);
  bool r_ok = isPulseValid(pulse_r);
  if (l_ok) {
    last_valid_left_ms = now_ms;
  }
  if (r_ok) {
    last_valid_right_ms = now_ms;
  }

  bool l_timeout = (now_ms - last_valid_left_ms) > kSignalTimeoutMs;
  bool r_timeout = (now_ms - last_valid_right_ms) > kSignalTimeoutMs;
  int cmd_l = (l_ok && !l_timeout) ? pulseToSignedPwm(pulse_l, kInvertLeft) : 0;
  int cmd_r = (r_ok && !r_timeout) ? pulseToSignedPwm(pulse_r, kInvertRight) : 0;
  applySignedPwmToMotor(cmd_l, left_motor);
  applySignedPwmToMotor(cmd_r, right_motor);

  if (now_ms - last_print_ms >= kPrintIntervalMs) {
    last_print_ms = now_ms;
    Serial.print("L:");
    Serial.print(pulse_l);
    Serial.print(l_ok ? "us" : "us(no)");
    Serial.print(" cmd=");
    Serial.print(cmd_l);
    Serial.print(" R:");
    Serial.print(pulse_r);
    Serial.print(r_ok ? "us" : "us(no)");
    Serial.print(" cmd=");
    Serial.print(cmd_r);
    if (l_timeout || r_timeout) {
      Serial.print(" timeout");
    }
    Serial.println();
  }
}
