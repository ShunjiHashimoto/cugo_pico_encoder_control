#include <Arduino.h>
#include <Servo.h>
#include <PacketSerial.h>
#include "MotorController.h"

// Test stage selector
// 1: Encoder-only logging
// 2: Manual RPM control (no serial protocol)
// 3: Full PacketSerial velocity control
#ifndef TEST_STAGE
#define TEST_STAGE 3
#endif

namespace {
constexpr int kMotorCount = 2;
constexpr int MOTOR_LEFT = 0;
constexpr int MOTOR_RIGHT = 1;
constexpr int kPulsePerRound = 2048;
constexpr int kMaxServoPwm = 600;
constexpr int kControlHz = 100;
constexpr float kLpf = 0.95f;
constexpr float kLeftKp = 1.5f;
constexpr float kLeftKi = 0.02f;
constexpr float kLeftKd = 0.1f;
constexpr float kRightKp = 1.5f;
constexpr float kRightKi = 0.02f;
constexpr float kRightKd = 0.1f;
constexpr bool kLeftReverse = false;
constexpr bool kRightReverse = true;

// Robot physical parameters
constexpr float kWheelRadiusL = 0.0825f;   // [m]
constexpr float kWheelRadiusR = 0.0825f;   // [m]
constexpr float kTread = 0.394f;           // [m]
constexpr float kReductionRatio = 30.0f;   // motor -> wheel gear ratio

// Pico pin assignment (GPIO numbers)
constexpr uint8_t PIN_MOTOR_L_PWM = 26;  // GP26 (PWM capable)
constexpr uint8_t PIN_MOTOR_L_DIR = 16;
constexpr uint8_t PIN_MOTOR_R_PWM = 27;  // GP27 (PWM capable)
constexpr uint8_t PIN_MOTOR_R_DIR = 17;
constexpr uint8_t PIN_ENCODER_L_A = 2;
constexpr uint8_t PIN_ENCODER_L_B = 8;
constexpr uint8_t PIN_ENCODER_R_A = 3;
constexpr uint8_t PIN_ENCODER_R_B = 9;

constexpr uint32_t kFailSafeThreshold = 5;  // 100ms * 5 = 0.5s
constexpr uint32_t kMicro10ms = 10000;
constexpr uint32_t kMicro100ms = 100000;
constexpr uint32_t kMicro1000ms = 1000000;

constexpr size_t SERIAL_BIN_BUFF_SIZE = 64;
constexpr size_t SERIAL_HEADER_SIZE = 8;
constexpr uint16_t kLocalPort = 8888;  // dummy metadata for compatibility

// Header pointers
constexpr int RECV_HEADER_PRODUCT_ID_PTR = 0;
constexpr int RECV_HEADER_CHECKSUM_PTR = 6;

// Body pointers (incoming: v [m/s], w [rad/s])
constexpr int TARGET_V_PTR = 0;
constexpr int TARGET_W_PTR = 4;
constexpr int SEND_ENCODER_L_PTR = 0;
constexpr int SEND_ENCODER_R_PTR = 4;

constexpr float kDefaultMaxRpm = 600.0f;
constexpr float kTwoPi = 6.28318530718f;

PacketSerial packetSerial;
MotorController motor_controllers[kMotorCount];

volatile unsigned long current_time = 0;
volatile unsigned long prev_time_10ms = 0;
volatile unsigned long prev_time_100ms = 0;
volatile unsigned long prev_time_1000ms = 0;

volatile int com_fail_count = 0;

long last_reported_count_l = 0;
long last_reported_count_r = 0;

}  // namespace

uint16_t calculate_checksum(const void* data, size_t size, size_t start = 0) {
  uint16_t checksum = 0;
  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  for (size_t i = start; i < size; i += 2) {
    uint16_t word = bytes[i] << 8;
    if (i + 1 < size) {
      word |= bytes[i + 1];
    }
    checksum += word;
  }
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

uint16_t read_uint16_from_header(const uint8_t* buf, int offset) {
  if (offset >= SERIAL_HEADER_SIZE - 1) {
    return 0;
  }
  uint16_t val = *reinterpret_cast<const uint16_t*>(buf + offset);
  return val;
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
}

void send_encoder_feedback() {
  uint8_t body[SERIAL_BIN_BUFF_SIZE];
  memset(body, 0, sizeof(body));
  long count_l = motor_controllers[MOTOR_LEFT].getCount();
  long count_r = motor_controllers[MOTOR_RIGHT].getCount();
  write_int_to_buf(body, SEND_ENCODER_L_PTR, count_l);
  write_int_to_buf(body, SEND_ENCODER_R_PTR, count_r);
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
#if TEST_STAGE >= 3
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

void init_motor_controllers() {
  pinMode(PIN_ENCODER_L_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_L_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_R_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_R_B, INPUT_PULLUP);

  motor_controllers[MOTOR_LEFT] = MotorController(PIN_ENCODER_L_A, PIN_ENCODER_L_B, PIN_MOTOR_L_PWM, PIN_MOTOR_L_DIR,
                                                 kPulsePerRound, kMaxServoPwm, kControlHz,
                                                 kLpf, kLeftKp, kLeftKi, kLeftKd, kLeftReverse);
  motor_controllers[MOTOR_RIGHT] = MotorController(PIN_ENCODER_R_A, PIN_ENCODER_R_B, PIN_MOTOR_R_PWM, PIN_MOTOR_R_DIR,
                                                  kPulsePerRound, kMaxServoPwm, kControlHz,
                                                  kLpf, kRightKp, kRightKi, kRightKd, kRightReverse);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L_A), leftEncHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R_A), rightEncHandler, RISING);
}

void stop_motor_immediately() {
  motor_controllers[MOTOR_LEFT].setTargetRpm(0.0f);
  motor_controllers[MOTOR_RIGHT].setTargetRpm(0.0f);
  motor_controllers[MOTOR_LEFT].stopOutput();
  motor_controllers[MOTOR_RIGHT].stopOutput();
}

void job_10ms() {
#if TEST_STAGE >= 2
  for (int i = 0; i < kMotorCount; ++i) {
    motor_controllers[i].driveMotor();
  }
#endif
}

void job_100ms() {
#if TEST_STAGE >= 3
  com_fail_count++;
  if (com_fail_count > static_cast<int>(kFailSafeThreshold)) {
    stop_motor_immediately();
  }
#endif
}

void job_1000ms() {
  // reserved for diagnostics
}

void setup() {
  Serial.begin(115200);
  init_motor_controllers();
#if TEST_STAGE >= 3
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
#if TEST_STAGE >= 3
  packetSerial.update();
  if (packetSerial.overflow()) {
    stop_motor_immediately();
  }
#elif TEST_STAGE == 1
  static unsigned long last_print = 0;
  if (millis() - last_print > 200) {
    last_print = millis();
    Serial.print("enc_l:");
    Serial.print(motor_controllers[MOTOR_LEFT].getCount());
    Serial.print(", enc_r:");
    Serial.println(motor_controllers[MOTOR_RIGHT].getCount());
  }
#elif TEST_STAGE == 2
  static unsigned long last_update = 0;
  if (millis() - last_update > 1000) {
    last_update = millis();
    float test_rpm = 100.0f;
    motor_controllers[MOTOR_LEFT].setTargetRpm(test_rpm);
    motor_controllers[MOTOR_RIGHT].setTargetRpm(test_rpm);
    Serial.println("[TEST_STAGE2] Commanded RPM:" + String(test_rpm));
  }
#endif
}
