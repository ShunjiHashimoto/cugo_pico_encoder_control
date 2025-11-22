#include "Arduino.h"
#include "MotorController.h"

MotorController::MotorController(){
  initialized_ = false;
}

MotorController::MotorController(int enc_pin_a, int enc_pin_b, int pwm_pin, int dir_pin, int pulse_per_round, int max_speed, int control_hz, float lpf_rate, float kp, float ki, float kd, bool reverse){

  // 引数で変数を初期化
  enc_pin_a_ = enc_pin_a;
  enc_pin_b_ = enc_pin_b;
  pwm_pin_ = pwm_pin;
  dir_pin_ = dir_pin;
  pulse_per_round_ = pulse_per_round;
  max_speed_ = max_speed;
  control_hz_ = control_hz;
  lpf_rate_ = lpf_rate;
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  reverse_ = reverse;

  // 変数の初期化
  speed_ = 0;
  enc_ = 0;
//  enc_rotation_ = 0;    //22/8/2追加
  prev_enc_ = 0;
  rpm_ = 0.0;
  prev_rpm_ = 0.0;
  target_rpm_ = 0.0;
  prev_p_ = 0.0;
  prev_i_ = 0.0;
  stop_cnt = 0;

  pinMode(pwm_pin_, OUTPUT);
  pinMode(dir_pin_, OUTPUT);
  analogWrite(pwm_pin_, 0);
  digitalWrite(dir_pin_, LOW);

  initialized_ = true;
}

void MotorController::driveMotor(){
  if(!initialized_){
    return;
  }

  // rpm_の更新
  calcRpm();
  
  pidControl();
 
  applyPwmOutput();
}

//static void MotorController::ChangedEncPin(){
//}

void MotorController::reset_PID_param()
{
  speed_ = 0;
  // 2023/6/2
  // TODO: びくつき修正で問答無用にカウンタをゼロにしてしまう。
  // 今までの積算したカウンタ->0に変化するため、オドメトリに数十mのベクトルが生成されてしまう。
  // びくつきが軽微であるため、コメントアウトするが、対策が必要
  //enc_ = 0; //Cugoびくつき修正:23/02/12
  //prev_enc_ = 0;  //Cugoびくつき修正:23/02/12
  rpm_ = 0.0;
  prev_rpm_ = 0.0;
  target_rpm_ = 0.0;
  prev_p_ = 0.0;
  prev_i_ = 0.0;
  stop_cnt = 0;
}

void MotorController::updateEnc(){
  if(!initialized_){
    return;
  }

  if(LOW == digitalRead(enc_pin_b_)){
    if(!reverse_){
      enc_--;  // PINがLOW & 正転 -> デクリメント
    }else{
      enc_++;  // PINがLOW & 逆転 -> インクリメント
    }
  }else{
    if(!reverse_){
      enc_++;  // PINがHIGH & 正転 -> インクリメント
    }else{
      enc_--;  // PINがHIGH & 逆転 -> デクリメント
    }
  }
}

void MotorController::setTargetRpm(float target_rpm){
  if(!initialized_){
    return;
  }
  target_rpm_ = target_rpm;
}

float MotorController::getTargetRpm(){
  if(!initialized_){
    return 0.0;
  }
  return target_rpm_;
}


long int MotorController::getCount(){
  if(!initialized_){
    return 0;
  }
  return enc_;
}

/*
int MotorController::getCounterRotation(){
  if(!initialized_){
    return 0;
  }
//  return enc_rotation;
}
*/

float MotorController::getRpm(){
  if(!initialized_){
    return 0.0;
  }
  
  return rpm_;
}

float MotorController::getSpeed(){
  if(!initialized_){
    return 0.0;
  }
  
  return speed_;
}

float MotorController::getPID_P(){
  if(!initialized_){
    return 0.0;
  }
  return disp_p;
}

float MotorController::getPID_I(){
  if(!initialized_){
    return 0.0;
  }
  return disp_i;
}

float MotorController::getPID_D(){
  if(!initialized_){
    return 0.0;
  }
  return disp_d;
}

void MotorController::calcRpm(){
  if(!initialized_){
    return;
  }
  
  float rps;  // RPS
  int diff; // エンコーダカウントの差分
  float rpm_nolpf;  // RPM(LPF前)

  // エンコーダカウントの差分を生成
  diff = enc_ - prev_enc_;
  // ノイズで車体が動いていない時もカウントアップしてしまうため
  if (-3 < diff && diff < 3 ){
    diff = 0;
    enc_ = prev_enc_;
  }
  //Serial.print(diff);
  //Serial.print(",");

  if(diff == 0){ // 0割り回避
    rpm_nolpf = 0.0;
  }else{
    // RPS 計算
    rps = (float)diff * (float)control_hz_ / (float)pulse_per_round_;
    
    // RPS -> RPM
    rpm_nolpf = rps * 60.0;
  }
  
  // RPM 計算
  rpm_ = prev_rpm_ * lpf_rate_ + rpm_nolpf * (1.0 - lpf_rate_);
  //Serial.println(rpm_);

  // prev_ 更新
  prev_rpm_ = rpm_;
  prev_enc_ = enc_;

}

void MotorController::pidControl(){
  if(!initialized_){
    return;
  }
  
  float p;  // P制御値
  float i;  // I制御値
  float d;  // D制御値

  // 各制御値の計算
  p = target_rpm_ - rpm_;
  i = prev_i_ + p;
  d = p - prev_p_;
//  Serial.println("");
//  Serial.println("i: " + String(i));

  // PID制御
  speed_ = p * kp_ + i * ki_ + d * kd_;
  
  // prev_ 更新
  prev_p_ = p;
  prev_i_ = i;

  // 画面表示用
  disp_p = p;
  disp_i = i;
  disp_d = d;

  // iゲインの発散補償（自動整合）
  float surplus = limitSpeed();
  prev_i_ = prev_i_ - surplus * kp_;

  // 停止中のiゲインの定常偏差をリセット（モータの不感地帯での制御がかかってしまうため）
  if(target_rpm_ == 0.0 || p == 0.0){
    stop_cnt++;
  }
  if(stop_cnt > 100){
    prev_i_ = 0.0;
    stop_cnt = 0;
  }

}

float MotorController::limitSpeed(){
  if(!initialized_){
    return -1.0;
  }
  float surplus; //Iゲイン自動整合用の余剰制御量を算出
  surplus = speed_ - max_speed_;
  
  // speed_ を範囲内に調整
  if(speed_ > max_speed_){
    speed_ = max_speed_;
    return surplus;
  }else if(speed_ < -max_speed_) {
    speed_ = -max_speed_;
    return surplus;
  } else {
    return 0.0;
  }

}

void MotorController::applyPwmOutput(){
  if(!initialized_){
    return;
  }
  int pwm_value = abs(speed_);
  if(pwm_value > max_speed_) {
    pwm_value = max_speed_;
  }
  bool forward = (speed_ >= 0);
  if(reverse_) {
    forward = !forward;
  }
  digitalWrite(dir_pin_, forward ? HIGH : LOW);
  analogWrite(pwm_pin_, pwm_value);
}

void MotorController::stopOutput(){
  if(!initialized_){
    return;
  }
  analogWrite(pwm_pin_, 0);
}
