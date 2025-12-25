#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#define PULSE_CENTRAL 1500  // legacy value (PWM mode does not use)
#define COUNT_INT_MAX 65535 // Arduino Uno/Nanoの場合はintが16bit

class MotorController {

  public:
    MotorController();
    MotorController(int enc_pin_a, int enc_pin_b, int pwm_pin, int dir_pin, int pulse_per_round, int max_speed, int control_hz, float lpf_rate, float kp, float ki, float kd, bool reverse_encoder, bool reverse_motor);
    void driveMotor();
    //static void ChangedEncPin();
    void updateEnc();
    void setTargetRpm(float target_rpm);
    float getRpm();
    float getSpeed();
    long int getCount();
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

    int enc_pin_a_;       // エンコーダカウントピン
    int enc_pin_b_;       // エンコーダカウントピン
    int pwm_pin_;         // PWMピン
    int dir_pin_;         // 回転方向ピン
    int max_speed_;       // 最大スピードを指定（絶対値）
    int control_hz_;      // 制御する周期
    int pulse_per_round_; //
    float lpf_rate_;      // ローパスフィルタの効き具合


    int16_t speed_;       // モータ出力速度
    long int enc_;             // エンコーダカウント
    long int prev_enc_;        // 前ステップのエンコーダカウント
    float rpm_;           // RPM
    float prev_rpm_;      // 前ステップのRPM
    float target_rpm_;    // 目標RPM
    float kp_;            // Pゲイン
    float ki_;            // Iゲイン
    float kd_;            // Dゲイン
    float disp_p;         // 表示用
    float disp_i;         // 表示用
    float disp_d;         // 表示用
    float prev_p_;        // 前ステップのP
    float prev_i_;        // 前ステップのI
    bool reverse_encoder_;
    bool reverse_motor_;

    int stop_cnt;         // 速度ゼロで不感地帯の制御値をいったんリセット

    bool initialized_;
    uint8_t last_enc_state_;

};


#endif
