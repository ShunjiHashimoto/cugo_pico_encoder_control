#include <Arduino.h>
#include <Servo.h>
#include <PacketSerial.h>
#include "MotorController.h"

// Test stage selector
// 1: Encoder-only logging
// 2: Manual RPM control (no serial protocol)
// 3: Fixed v/w control (no serial protocol)
// 4: Full PacketSerial velocity control
#ifndef TEST_STAGE
#define TEST_STAGE 4
#endif

namespace {
constexpr int kMotorCount = 2;
constexpr int MOTOR_LEFT = 0;
constexpr int MOTOR_RIGHT = 1;
constexpr int kPulsePerRound = 8192; // 2048
constexpr int kMaxServoPwm = 600;
constexpr int kControlHz = 100;
constexpr float kLpf = 0.95f;
constexpr float kLeftKp = 1.5f;
constexpr float kLeftKi = 0.02f;
constexpr float kLeftKd = 0.1f;
constexpr float kRightKp = 1.5f;
constexpr float kRightKi = 0.02f;
constexpr float kRightKd = 0.1f;
constexpr bool kLeftEncoderReverse = false;
constexpr bool kRightEncoderReverse = true;
constexpr bool kLeftMotorReverse = true;
constexpr bool kRightMotorReverse = true;

// Robot physical parameters
constexpr float kWheelRadiusL = 0.0825f;   // [m]
constexpr float kWheelRadiusR = 0.0825f;   // [m]
constexpr float kTread = 0.394f;           // [m]
constexpr float kReductionRatio = 30.0f;   // motor -> wheel gear ratio

// Pico pin assignment (GPIO numbers)
constexpr uint8_t PIN_MOTOR_L_PWM = 17;  // GP17 (PWM capable)
constexpr uint8_t PIN_MOTOR_L_DIR = 16;
constexpr uint8_t PIN_MOTOR_R_PWM = 19;  // GP19 (PWM capable)
constexpr uint8_t PIN_MOTOR_R_DIR = 18;
constexpr uint8_t PIN_ENCODER_L_A = 2;  // left A -> GP2
constexpr uint8_t PIN_ENCODER_L_B = 3;  // left B -> GP3
constexpr uint8_t PIN_ENCODER_R_A = 8;  // right A -> GP8
constexpr uint8_t PIN_ENCODER_R_B = 9;  // right B -> GP9
constexpr uint8_t PIN_USER_BUTTON = 15;  // SW5 active-low (connect to GND)
constexpr uint8_t PIN_ADC_BAT = 26;  // GP26 (ADC0)
#if defined(LED_BUILTIN)
constexpr uint8_t PIN_STATUS_LED = LED_BUILTIN;
#else
constexpr uint8_t PIN_STATUS_LED = 25;  // Onboard LED for RP2040
#endif

constexpr uint32_t kFailSafeThreshold = 5;  // 100ms * 5 = 0.5s
constexpr uint32_t kMicro10ms = 10000;
constexpr uint32_t kMicro100ms = 100000;
constexpr uint32_t kMicro1000ms = 1000000;

constexpr size_t SERIAL_BIN_BUFF_SIZE = 64;
constexpr size_t SERIAL_HEADER_SIZE = 8;
constexpr uint16_t kLocalPort = 8888;  // dummy metadata for compatibility
constexpr uint16_t kAdcMax = 4095;
constexpr float kAdcVref = 3.3f;
constexpr float kBatteryDividerRatio = 10.85f;  // Adjust to match the actual divider.

// Header pointers
constexpr int RECV_HEADER_PRODUCT_ID_PTR = 0;
constexpr int RECV_HEADER_CHECKSUM_PTR = 6;

// Body pointers (incoming: v [m/s], w [rad/s])
constexpr int TARGET_V_PTR = 0;
constexpr int TARGET_W_PTR = 4;
constexpr int SEND_ENCODER_L_PTR = 0;
constexpr int SEND_ENCODER_R_PTR = 4;
constexpr int SEND_BATTERY_VOLT_PTR = 8;
constexpr int SEND_V_PTR = 12;
constexpr int SEND_W_PTR = 16;

constexpr float kDefaultMaxRpm = 600.0f;
constexpr float kTwoPi = 6.28318530718f;
constexpr float kTestTargetV = 0.01f;   // [m/s]
constexpr float kTestTargetW = 0.0f;   // [rad/s]
constexpr uint32_t kTestUpdateMs = 1000;

PacketSerial packetSerial;
MotorController motor_controllers[kMotorCount];

volatile unsigned long current_time = 0;
volatile unsigned long prev_time_10ms = 0;
volatile unsigned long prev_time_100ms = 0;
volatile unsigned long prev_time_1000ms = 0;

volatile int com_fail_count = 0;

long last_reported_count_l = 0;
long last_reported_count_r = 0;
bool status_led_state = false;
bool failsafe_active = false;
bool motor_enabled = false;

struct EncoderEstimate {
  long last_count;
  unsigned long last_time_us;
  float last_motor_rpm;
};

EncoderEstimate encoder_estimates[kMotorCount];

}  // namespace

// Forward declarations for helpers used before definition
void stop_motor_immediately();

uint16_t calculate_checksum(const void* data, size_t size, size_t start = 0) {
  uint32_t checksum = 0;
  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  for (size_t i = start; i < size; i += 2) {
    uint16_t word = bytes[i] << 8;
    if (i + 1 < size) {
      word |= bytes[i + 1];
    }
    checksum += word;
  }
  checksum = (checksum & 0xFFFF) + (checksum >> 16);
  checksum = (checksum & 0xFFFF) + (checksum >> 16);
  return static_cast<uint16_t>(~checksum);
}

void create_serial_packet(uint8_t* packet, const uint16_t* header, const uint8_t* body) {
  memcpy(packet, header, sizeof(uint16_t) * 4);
  memcpy(packet + SERIAL_HEADER_SIZE, body, SERIAL_BIN_BUFF_SIZE);
}

float read_float_from_buf(const uint8_t* buf, int offset) {
  float val = *reinterpret_cast<const float*>(buf + SERIAL_HEADER_SIZE + offset);
  return val;
}

void write_int_to_buf(uint8_t* buf, int offset, long val) {
  memcpy(buf + offset, &val, sizeof(long));
}

void write_float_to_buf(uint8_t* buf, int offset, float val) {
  memcpy(buf + offset, &val, sizeof(float));
}

uint16_t read_uint16_from_header(const uint8_t* buf, int offset) {
  if (offset >= SERIAL_HEADER_SIZE - 1) {
    return 0;
  }
  uint16_t val = *reinterpret_cast<const uint16_t*>(buf + offset);
  return val;
}

float read_battery_voltage() {
  int raw = analogRead(PIN_ADC_BAT);
  float v_adc = (static_cast<float>(raw) / kAdcMax) * kAdcVref;
  return v_adc * kBatteryDividerRatio;
}

float clamp_rpm(float rpm, float max_rpm) {
  if (rpm > max_rpm) return max_rpm;
  if (rpm < -max_rpm) return -max_rpm;
  return rpm;
}

float resolve_max_rpm(uint16_t product_id) {
  switch (product_id) {
    case 0x0002:
      return 400.0f;  // example for CuGo V4
    case 0x0003:
      return 300.0f;  // example for CuGo V3i
    default:
      return kDefaultMaxRpm;
  }
}

float linear_to_motor_rpm(float linear_velocity, float wheel_radius) {
  float wheel_ang = linear_velocity / wheel_radius;  // [rad/s]
  float motor_ang = wheel_ang * kReductionRatio;
  return motor_ang * 60.0f / kTwoPi;
}

void apply_velocity_targets(float v, float w, float max_rpm) {
  float v_l = v - (w * kTread * 0.5f);
  float v_r = v + (w * kTread * 0.5f);
  float rpm_l = linear_to_motor_rpm(v_l, kWheelRadiusL);
  float rpm_r = linear_to_motor_rpm(v_r, kWheelRadiusR);
  rpm_l = clamp_rpm(rpm_l, max_rpm);
  rpm_r = clamp_rpm(rpm_r, max_rpm);
  motor_controllers[MOTOR_LEFT].setTargetRpm(rpm_l);
  motor_controllers[MOTOR_RIGHT].setTargetRpm(rpm_r);
}

float motor_rpm_to_linear_velocity(int motor_index, float motor_rpm) {
  float wheel_radius = (motor_index == MOTOR_LEFT) ? kWheelRadiusL : kWheelRadiusR;
  float wheel_rps = (motor_rpm / 60.0f) / kReductionRatio;
  return wheel_rps * kTwoPi * wheel_radius;
}

float update_motor_rpm_estimate(int motor_index, unsigned long now_us) {
  EncoderEstimate& estimate = encoder_estimates[motor_index];
  long current_count = motor_controllers[motor_index].getCount();
  long diff = current_count - estimate.last_count;
  unsigned long elapsed = now_us - estimate.last_time_us;
  estimate.last_count = current_count;
  estimate.last_time_us = now_us;
  if (elapsed == 0) {
    return estimate.last_motor_rpm;
  }
  float dt = static_cast<float>(elapsed) * 1e-6f;
  float motor_rev = static_cast<float>(diff) / static_cast<float>(kPulsePerRound);
  float motor_rps = motor_rev / dt;
  estimate.last_motor_rpm = motor_rps * 60.0f;
  return estimate.last_motor_rpm;
}

void init_encoder_estimates() {
  unsigned long now = micros();
  for (int i = 0; i < kMotorCount; ++i) {
    encoder_estimates[i].last_count = motor_controllers[i].getCount();
    encoder_estimates[i].last_time_us = now;
    encoder_estimates[i].last_motor_rpm = 0.0f;
  }
}

void set_motor_cmd_binary(const uint8_t* packet, size_t size) {
  (void)size;
  uint16_t product_id = read_uint16_from_header(packet, RECV_HEADER_PRODUCT_ID_PTR);
  float max_rpm = resolve_max_rpm(product_id);
  uint16_t recv_checksum = read_uint16_from_header(packet, RECV_HEADER_CHECKSUM_PTR);
  uint16_t calc_checksum = calculate_checksum(packet + SERIAL_HEADER_SIZE, SERIAL_BIN_BUFF_SIZE);
  if (recv_checksum != calc_checksum) {
    return;
  }

  float target_v = read_float_from_buf(packet, TARGET_V_PTR);
  float target_w = read_float_from_buf(packet, TARGET_W_PTR);
  apply_velocity_targets(target_v, target_w, max_rpm);
  com_fail_count = 0;
  if (failsafe_active) {
    failsafe_active = false;
    set_status_led(false);
  }
}

void send_encoder_feedback() {
  uint8_t body[SERIAL_BIN_BUFF_SIZE];
  memset(body, 0, sizeof(body));
  long count_l = motor_controllers[MOTOR_LEFT].getCount();
  long count_r = motor_controllers[MOTOR_RIGHT].getCount();
  write_int_to_buf(body, SEND_ENCODER_L_PTR, count_l);
  write_int_to_buf(body, SEND_ENCODER_R_PTR, count_r);
  float v_bat = read_battery_voltage();
  write_float_to_buf(body, SEND_BATTERY_VOLT_PTR, v_bat);
  unsigned long now_us = micros();
  float rpm_l = update_motor_rpm_estimate(MOTOR_LEFT, now_us);
  float rpm_r = update_motor_rpm_estimate(MOTOR_RIGHT, now_us);
  float vel_l = motor_rpm_to_linear_velocity(MOTOR_LEFT, rpm_l);
  float vel_r = motor_rpm_to_linear_velocity(MOTOR_RIGHT, rpm_r);
  float v = 0.5f * (vel_l + vel_r);
  float w = 0.0f;
  if (kTread > 0.0f) {
    w = (vel_r - vel_l) / kTread;
  }
  write_float_to_buf(body, SEND_V_PTR, v);
  write_float_to_buf(body, SEND_W_PTR, w);
  uint16_t checksum = calculate_checksum(body, SERIAL_BIN_BUFF_SIZE);
  uint16_t header[4] = {kLocalPort, kLocalPort, static_cast<uint16_t>(SERIAL_HEADER_SIZE + SERIAL_BIN_BUFF_SIZE), checksum};
  uint8_t packet[SERIAL_HEADER_SIZE + SERIAL_BIN_BUFF_SIZE];
  create_serial_packet(packet, header, body);
  packetSerial.send(packet, sizeof(packet));
  last_reported_count_l = count_l;
  last_reported_count_r = count_r;
}

void onSerialPacketReceived(const uint8_t* buffer, size_t size) {
  uint8_t temp[size];
  memcpy(temp, buffer, size);
#if TEST_STAGE >= 4
  set_motor_cmd_binary(temp, size);
  send_encoder_feedback();
#endif
}

void leftEncHandler() {
  motor_controllers[MOTOR_LEFT].updateEnc();
}

void rightEncHandler() {
  motor_controllers[MOTOR_RIGHT].updateEnc();
}

void set_status_led(bool on) {
  status_led_state = on;
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

void toggle_status_led() {
  set_status_led(!status_led_state);
}

void init_status_led() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  failsafe_active = false;
  set_status_led(false);
}

void init_user_button() {
  pinMode(PIN_USER_BUTTON, INPUT_PULLUP);
}

void update_motor_enable_from_button() {
  bool pressed = digitalRead(PIN_USER_BUTTON) == LOW;
  if (pressed != motor_enabled) {
    motor_enabled = pressed;
    if (!motor_enabled) {
      stop_motor_immediately();
    }
  }
}

void init_motor_controllers() {
  pinMode(PIN_ENCODER_L_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_L_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_R_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_R_B, INPUT_PULLUP);

  motor_controllers[MOTOR_LEFT] = MotorController(PIN_ENCODER_L_A, PIN_ENCODER_L_B, PIN_MOTOR_L_PWM, PIN_MOTOR_L_DIR,
                                                 kPulsePerRound, kMaxServoPwm, kControlHz,
                                                 kLpf, kLeftKp, kLeftKi, kLeftKd, kLeftEncoderReverse, kLeftMotorReverse);
  motor_controllers[MOTOR_RIGHT] = MotorController(PIN_ENCODER_R_A, PIN_ENCODER_R_B, PIN_MOTOR_R_PWM, PIN_MOTOR_R_DIR,
                                                  kPulsePerRound, kMaxServoPwm, kControlHz,
                                                  kLpf, kRightKp, kRightKi, kRightKd, kRightEncoderReverse, kRightMotorReverse);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_A), leftEncHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_B), leftEncHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_A), rightEncHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_B), rightEncHandler, CHANGE);
}

void stop_motor_immediately() {
  motor_controllers[MOTOR_LEFT].setTargetRpm(0.0f);
  motor_controllers[MOTOR_RIGHT].setTargetRpm(0.0f);
  motor_controllers[MOTOR_LEFT].stopOutput();
  motor_controllers[MOTOR_RIGHT].stopOutput();
}

void job_10ms() {
#if TEST_STAGE >= 2
  if (motor_enabled) {
    for (int i = 0; i < kMotorCount; ++i) {
      motor_controllers[i].driveMotor();
    }
  } else {
    stop_motor_immediately();
  }
#endif
}

void job_100ms() {
#if TEST_STAGE >= 4
  com_fail_count++;
  if (com_fail_count > static_cast<int>(kFailSafeThreshold)) {
    if (!failsafe_active) {
      failsafe_active = true;
      set_status_led(true);
    }
    stop_motor_immediately();
  }
#endif
}

void job_1000ms() {
  if (!failsafe_active) {
    toggle_status_led();
  }
}

void setup() {
  Serial.begin(115200);
  analogWriteFreq(20000);
  analogReadResolution(12);
  init_status_led();
  init_user_button();
  init_motor_controllers();
  init_encoder_estimates();
#if TEST_STAGE >= 4
  packetSerial.begin(115200);
  packetSerial.setStream(&Serial);
  packetSerial.setPacketHandler(&onSerialPacketReceived);
  delay(100);
  while (Serial.available() > 0) {
    Serial.read();
  }
#endif
}

void loop() {
  update_motor_enable_from_button();
  current_time = micros();
  if (current_time - prev_time_10ms > kMicro10ms) {
    job_10ms();
    prev_time_10ms = current_time;
  }
  if (current_time - prev_time_100ms > kMicro100ms) {
    job_100ms();
    prev_time_100ms = current_time;
  }
  if (current_time - prev_time_1000ms > kMicro1000ms) {
    job_1000ms();
    prev_time_1000ms = current_time;
  }
#if TEST_STAGE >= 4
  packetSerial.update();
  if (packetSerial.overflow()) {
    stop_motor_immediately();
  }
#elif TEST_STAGE == 1
  static unsigned long last_print = 0;
  if (millis() - last_print > 200) {
    last_print = millis();
    long enc_l = motor_controllers[MOTOR_LEFT].getCount();
    long enc_r = motor_controllers[MOTOR_RIGHT].getCount();
    float rpm_l = update_motor_rpm_estimate(MOTOR_LEFT, micros());
    float rpm_r = update_motor_rpm_estimate(MOTOR_RIGHT, micros());
    float vel_l = motor_rpm_to_linear_velocity(MOTOR_LEFT, rpm_l);
    float vel_r = motor_rpm_to_linear_velocity(MOTOR_RIGHT, rpm_r);
    Serial.print("enc_l:");
    Serial.print(enc_l);
    Serial.print(", enc_r:");
    Serial.print(enc_r);
    Serial.print(", rpm_l:");
    Serial.print(rpm_l, 2);
    Serial.print(", rpm_r:");
    Serial.print(rpm_r, 2);
    Serial.print(", v_l:");
    Serial.print(vel_l, 3);
    Serial.print("[m/s]");
    Serial.print(", v_r:");
    Serial.print(vel_r, 3);
    Serial.println("[m/s]");
  }
#elif TEST_STAGE == 2
  static unsigned long last_update = 0;
  if (millis() - last_update > 1000) {
    last_update = millis();
    float test_rpm = motor_enabled ? 20.0f : 0.0f;
    motor_controllers[MOTOR_LEFT].setTargetRpm(test_rpm);
    motor_controllers[MOTOR_RIGHT].setTargetRpm(test_rpm);
    float rpm_l = update_motor_rpm_estimate(MOTOR_LEFT, micros());
    float rpm_r = update_motor_rpm_estimate(MOTOR_RIGHT, micros());
    Serial.print("[TEST_STAGE2] Cmd RPM:");
    Serial.print(test_rpm, 1);
    Serial.print(", meas_l:");
    Serial.print(rpm_l, 1);
    Serial.print(", meas_r:");
    Serial.print(rpm_r, 1);
    Serial.print(", v_l:");
    Serial.print(motor_rpm_to_linear_velocity(MOTOR_LEFT, rpm_l), 3);
    Serial.print(", v_r:");
    Serial.println(motor_rpm_to_linear_velocity(MOTOR_RIGHT, rpm_r), 3);
  }
#elif TEST_STAGE == 3
  static unsigned long last_update = 0;
  if (millis() - last_update > kTestUpdateMs) {
    last_update = millis();
    float target_v = motor_enabled ? kTestTargetV : 0.0f;
    float target_w = motor_enabled ? kTestTargetW : 0.0f;
    apply_velocity_targets(target_v, target_w, kDefaultMaxRpm);
    float rpm_l = update_motor_rpm_estimate(MOTOR_LEFT, micros());
    float rpm_r = update_motor_rpm_estimate(MOTOR_RIGHT, micros());
    float vel_l = motor_rpm_to_linear_velocity(MOTOR_LEFT, rpm_l);
    float vel_r = motor_rpm_to_linear_velocity(MOTOR_RIGHT, rpm_r);
    Serial.print("[TEST_STAGE3] v:");
    Serial.print(target_v, 3);
    Serial.print(", w:");
    Serial.print(target_w, 3);
    Serial.print(", rpm_l:");
    Serial.print(rpm_l, 1);
    Serial.print(", rpm_r:");
    Serial.print(rpm_r, 1);
    Serial.print(", v_l:");
    Serial.print(vel_l, 3);
    Serial.print(", v_r:");
    Serial.println(vel_r, 3);
  }
#endif
}
