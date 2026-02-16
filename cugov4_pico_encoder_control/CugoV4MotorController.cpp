#include "Arduino.h"
#include "CugoV4MotorController.h"

MotorController::MotorController()
    : enc_pin_(-1),
      pwm_pin_(-1),
      dir_pin_fwd_(-1),
      dir_pin_rev_(-1),
      max_pwm_(0),
      control_hz_(0),
      pulse_per_round_(1),
      lpf_rate_(0.95f),
      speed_(0),
      enc_(0),
      prev_enc_(0),
      rpm_(0.0f),
      prev_rpm_(0.0f),
      target_rpm_(0.0f),
      kp_(0.0f),
      ki_(0.0f),
      kd_(0.0f),
      disp_p_(0.0f),
      disp_i_(0.0f),
      disp_d_(0.0f),
      prev_p_(0.0f),
      prev_i_(0.0f),
      reverse_encoder_(false),
      reverse_motor_(false),
      stop_cnt_(0),
      initialized_(false),
      last_cmd_forward_(true) {}

MotorController::MotorController(int enc_pin, int pwm_pin, int dir_pin_fwd, int pulse_per_round,
                                 int max_pwm, int control_hz, float lpf_rate, float kp,
                                 float ki, float kd, bool reverse_encoder, bool reverse_motor)
    : MotorController(enc_pin, pwm_pin, dir_pin_fwd, -1, pulse_per_round, max_pwm, control_hz,
                      lpf_rate, kp, ki, kd, reverse_encoder, reverse_motor) {}

MotorController::MotorController(int enc_pin, int pwm_pin, int dir_pin_fwd, int dir_pin_rev,
                                 int pulse_per_round, int max_pwm, int control_hz,
                                 float lpf_rate, float kp, float ki, float kd,
                                 bool reverse_encoder, bool reverse_motor)
    : enc_pin_(enc_pin),
      pwm_pin_(pwm_pin),
      dir_pin_fwd_(dir_pin_fwd),
      dir_pin_rev_(dir_pin_rev),
      max_pwm_(max_pwm),
      control_hz_(control_hz),
      pulse_per_round_(pulse_per_round),
      lpf_rate_(lpf_rate),
      speed_(0),
      enc_(0),
      prev_enc_(0),
      rpm_(0.0f),
      prev_rpm_(0.0f),
      target_rpm_(0.0f),
      kp_(kp),
      ki_(ki),
      kd_(kd),
      disp_p_(0.0f),
      disp_i_(0.0f),
      disp_d_(0.0f),
      prev_p_(0.0f),
      prev_i_(0.0f),
      reverse_encoder_(reverse_encoder),
      reverse_motor_(reverse_motor),
      stop_cnt_(0),
      initialized_(false),
      last_cmd_forward_(true) {
  pinMode(enc_pin_, INPUT_PULLUP);
  pinMode(pwm_pin_, OUTPUT);
  pinMode(dir_pin_fwd_, OUTPUT);
  if (dir_pin_rev_ >= 0) {
    pinMode(dir_pin_rev_, OUTPUT);
  }

  analogWrite(pwm_pin_, 0);
  digitalWrite(dir_pin_fwd_, LOW);
  if (dir_pin_rev_ >= 0) {
    digitalWrite(dir_pin_rev_, LOW);
  }

  initialized_ = true;
}

void MotorController::driveMotor() {
  if (!initialized_) {
    return;
  }

  calcRpm();
  pidControl();
  applyPwmOutput();
}

void MotorController::reset_PID_param() {
  speed_ = 0;
  rpm_ = 0.0f;
  prev_rpm_ = 0.0f;
  target_rpm_ = 0.0f;
  prev_p_ = 0.0f;
  prev_i_ = 0.0f;
  stop_cnt_ = 0;
}

void MotorController::updateEnc() {
  if (!initialized_) {
    return;
  }

  int8_t delta = last_cmd_forward_ ? 1 : -1;
  if (reverse_encoder_) {
    delta = -delta;
  }
  enc_ += delta;
}

void MotorController::setTargetRpm(float target_rpm) {
  if (!initialized_) {
    return;
  }
  target_rpm_ = target_rpm;
}

float MotorController::getTargetRpm() {
  if (!initialized_) {
    return 0.0f;
  }
  return target_rpm_;
}

long MotorController::getCount() {
  if (!initialized_) {
    return 0;
  }
  return enc_;
}

float MotorController::getRpm() {
  if (!initialized_) {
    return 0.0f;
  }
  return rpm_;
}

float MotorController::getSpeed() {
  if (!initialized_) {
    return 0.0f;
  }
  return speed_;
}

float MotorController::getPID_P() {
  if (!initialized_) {
    return 0.0f;
  }
  return disp_p_;
}

float MotorController::getPID_I() {
  if (!initialized_) {
    return 0.0f;
  }
  return disp_i_;
}

float MotorController::getPID_D() {
  if (!initialized_) {
    return 0.0f;
  }
  return disp_d_;
}

void MotorController::calcRpm() {
  if (!initialized_) {
    return;
  }

  int diff = static_cast<int>(enc_ - prev_enc_);
  float rpm_nolpf = 0.0f;

  if (diff != 0) {
    float rps = static_cast<float>(diff) * static_cast<float>(control_hz_) /
                static_cast<float>(pulse_per_round_);
    rpm_nolpf = rps * 60.0f;
  }

  rpm_ = prev_rpm_ * lpf_rate_ + rpm_nolpf * (1.0f - lpf_rate_);
  prev_rpm_ = rpm_;
  prev_enc_ = enc_;
}

void MotorController::pidControl() {
  if (!initialized_) {
    return;
  }

  float p = target_rpm_ - rpm_;
  float i = prev_i_ + p;
  float d = p - prev_p_;

  speed_ = p * kp_ + i * ki_ + d * kd_;

  prev_p_ = p;
  prev_i_ = i;

  disp_p_ = p;
  disp_i_ = i;
  disp_d_ = d;

  float surplus = limitSpeed();
  prev_i_ = prev_i_ - surplus * kp_;

  if (target_rpm_ == 0.0f || p == 0.0f) {
    stop_cnt_++;
  }
  if (stop_cnt_ > 100) {
    prev_i_ = 0.0f;
    stop_cnt_ = 0;
  }
}

float MotorController::limitSpeed() {
  if (!initialized_) {
    return -1.0f;
  }

  float surplus = static_cast<float>(speed_) - static_cast<float>(max_pwm_);
  if (speed_ > max_pwm_) {
    speed_ = max_pwm_;
    return surplus;
  }
  if (speed_ < -max_pwm_) {
    speed_ = -max_pwm_;
    return surplus;
  }
  return 0.0f;
}

void MotorController::applyPwmOutput() {
  if (!initialized_) {
    return;
  }

  int pwm_value = abs(speed_);
  if (pwm_value > max_pwm_) {
    pwm_value = max_pwm_;
  }

  bool cmd_forward = (speed_ >= 0);
  if (pwm_value > 0) {
    last_cmd_forward_ = cmd_forward;
  }

  if (pwm_value == 0) {
    analogWrite(pwm_pin_, 0);
    digitalWrite(dir_pin_fwd_, LOW);
    if (dir_pin_rev_ >= 0) {
      digitalWrite(dir_pin_rev_, LOW);
    }
    return;
  }

  bool hw_forward = cmd_forward;
  if (reverse_motor_) {
    hw_forward = !hw_forward;
  }

  if (dir_pin_rev_ >= 0) {
    digitalWrite(dir_pin_fwd_, hw_forward ? HIGH : LOW);
    digitalWrite(dir_pin_rev_, hw_forward ? LOW : HIGH);
  } else {
    digitalWrite(dir_pin_fwd_, hw_forward ? HIGH : LOW);
  }
  analogWrite(pwm_pin_, pwm_value);
}

void MotorController::stopOutput() {
  if (!initialized_) {
    return;
  }

  analogWrite(pwm_pin_, 0);
  digitalWrite(dir_pin_fwd_, LOW);
  if (dir_pin_rev_ >= 0) {
    digitalWrite(dir_pin_rev_, LOW);
  }
}
