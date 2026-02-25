#ifndef CUGOV4_MOTOR_CONTROLLER_H
#define CUGOV4_MOTOR_CONTROLLER_H

#include <stdint.h>

class MotorController {
 public:
  MotorController();
  MotorController(int enc_pin, int pwm_pin, int dir_pin_fwd, int pulse_per_round, int max_pwm,
                  int control_hz, float lpf_rate, float kp, float ki, float kd,
                  bool reverse_encoder, bool reverse_motor);
  MotorController(int enc_pin, int pwm_pin, int dir_pin_fwd, int dir_pin_rev,
                  int pulse_per_round, int max_pwm, int control_hz, float lpf_rate,
                  float kp, float ki, float kd, bool reverse_encoder, bool reverse_motor);

  void driveMotor();
  void updateEnc();
  void setTargetRpm(float target_rpm);
  float getRpm();
  float getSpeed();
  long getCount();
  float getTargetRpm();
  void reset_PID_param();
  float getPID_P();
  float getPID_I();
  float getPID_D();
  void stopOutput();

 private:
  void calcRpm();
  void pidControl();
  float limitSpeed();
  void applyPwmOutput();

  int enc_pin_;
  int pwm_pin_;
  int dir_pin_fwd_;
  int dir_pin_rev_;  // -1 when the driver supports only one direction input.
  int max_pwm_;
  int control_hz_;
  int pulse_per_round_;
  float lpf_rate_;

  int16_t speed_;
  long enc_;
  long prev_enc_;
  float rpm_;
  float prev_rpm_;
  float target_rpm_;
  float kp_;
  float ki_;
  float kd_;
  float disp_p_;
  float disp_i_;
  float disp_d_;
  float prev_p_;
  float prev_i_;
  bool reverse_encoder_;
  bool reverse_motor_;

  int stop_cnt_;
  bool initialized_;
  int8_t commanded_direction_;  // 1: forward, -1: reverse, 0: no direction command
};

#endif
