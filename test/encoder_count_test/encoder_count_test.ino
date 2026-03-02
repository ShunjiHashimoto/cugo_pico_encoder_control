#include <Arduino.h>

// DC mode only: set to 1 for left encoder, 0 for right encoder.
#ifndef USE_LEFT_ENCODER
#define USE_LEFT_ENCODER 0
#endif

// 0: DC motor driver (quadrature encoder)
// 1: BLDC driver HM-5100J (SPEED-OUT)
// 2: BLDC driver HP-5097 (SPEED-OUT)
#ifndef DRIVER_TYPE
#define DRIVER_TYPE 1
#endif

#define DRIVER_DC 0
#define DRIVER_BLDC_HM5100J 1
#define DRIVER_BLDC_HP5097 2

constexpr uint8_t PIN_ENCODER_L_A = 2;
constexpr uint8_t PIN_ENCODER_L_B = 3;
constexpr uint8_t PIN_ENCODER_R_A = 8;
constexpr uint8_t PIN_ENCODER_R_B = 9;
constexpr uint8_t PIN_SPEED_OUT_L = 5;
constexpr uint8_t PIN_SPEED_OUT_R = 7;
constexpr uint8_t PIN_DIRECTION_L = 12;
constexpr uint8_t PIN_DIRECTION_R = 13;
constexpr bool kDirectionHighIsForwardLeft = false;
constexpr bool kDirectionHighIsForwardRight = true;

constexpr bool kLeftReverse = false;
constexpr bool kRightReverse = true;
constexpr int kSpeedOutPulsesPerRev = 30;

constexpr uint32_t kBaud = 115200;
constexpr uint32_t kReportMs = 200;

struct EncoderState {
  volatile long count;
  volatile uint8_t last_state;  // used for quadrature mode only
  uint8_t pin_a;
  uint8_t pin_b;
  bool reverse;
  bool use_quadrature;
};

EncoderState encoder;
volatile long bldc_count_left = 0;
volatile long bldc_count_right = 0;

int8_t readDirectionSign(uint8_t pin_direction, bool high_is_forward) {
  bool high = (digitalRead(pin_direction) == HIGH);
  bool forward = high_is_forward ? high : !high;
  return forward ? 1 : -1;
}

void updateQuadratureEncoder() {
  bool a_state = digitalRead(encoder.pin_a);
  bool b_state = digitalRead(encoder.pin_b);
  uint8_t state = static_cast<uint8_t>((a_state << 1) | (b_state ? 1 : 0));
  if (state == encoder.last_state) {
    return;
  }

  static const int8_t transition_table[4][4] = {
      { 0, -1,  1,  0},
      { 1,  0,  0, -1},
      {-1,  0,  0,  1},
      { 0,  1, -1,  0},
  };

  int8_t delta = transition_table[encoder.last_state][state];
  if (delta != 0) {
    if (encoder.reverse) {
      delta = -delta;
    }
    encoder.count += delta;
  }
  encoder.last_state = state;
}

void encoderHandler() {
  updateQuadratureEncoder();
}

void speedOutLeftHandler() {
  bldc_count_left += readDirectionSign(PIN_DIRECTION_L, kDirectionHighIsForwardLeft);
}

void speedOutRightHandler() {
  bldc_count_right += readDirectionSign(PIN_DIRECTION_R, kDirectionHighIsForwardRight);
}

void setup() {
  Serial.begin(kBaud);

  encoder.use_quadrature = (DRIVER_TYPE == DRIVER_DC);

#if DRIVER_TYPE == DRIVER_DC
  Serial.println("Driver: DC (quadrature)");
  #if USE_LEFT_ENCODER
    Serial.println("Encoder: LEFT");
    encoder.pin_a = PIN_ENCODER_L_A;
    encoder.pin_b = PIN_ENCODER_L_B;
    encoder.reverse = kLeftReverse;
  #else
    Serial.println("Encoder: RIGHT");
    encoder.pin_a = PIN_ENCODER_R_A;
    encoder.pin_b = PIN_ENCODER_R_B;
    encoder.reverse = kRightReverse;
  #endif
#elif DRIVER_TYPE == DRIVER_BLDC_HM5100J
  Serial.println("Driver: BLDC HM-5100J (SPEED-OUT)");
#elif DRIVER_TYPE == DRIVER_BLDC_HP5097
  Serial.println("Driver: BLDC HP-5097 (SPEED-OUT)");
#else
#error "Unsupported DRIVER_TYPE. Use 0, 1, or 2."
#endif

  if (encoder.use_quadrature) {
    pinMode(encoder.pin_a, INPUT_PULLUP);
    pinMode(encoder.pin_b, INPUT_PULLUP);
    uint8_t initial_a = digitalRead(encoder.pin_a);
    uint8_t initial_b = digitalRead(encoder.pin_b);
    encoder.last_state = static_cast<uint8_t>((initial_a << 1) | (initial_b & 0x01));
    attachInterrupt(digitalPinToInterrupt(encoder.pin_a), encoderHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder.pin_b), encoderHandler, CHANGE);
    encoder.count = 0;
  } else {
    pinMode(PIN_SPEED_OUT_L, INPUT_PULLUP);
    pinMode(PIN_SPEED_OUT_R, INPUT_PULLUP);
    pinMode(PIN_DIRECTION_L, INPUT_PULLUP);
    pinMode(PIN_DIRECTION_R, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_SPEED_OUT_L), speedOutLeftHandler, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_SPEED_OUT_R), speedOutRightHandler, RISING);
    Serial.print("SPEED-OUT pulses/rev: ");
    Serial.println(kSpeedOutPulsesPerRev);
    Serial.println("Direction pins: LEFT=GPIO12, RIGHT=GPIO13");
    bldc_count_left = 0;
    bldc_count_right = 0;
  }
}

void loop() {
  static unsigned long last_report = 0;
  static long last_count_left = 0;
  static long last_count_right = 0;
  if (millis() - last_report >= kReportMs) {
    last_report = millis();
    if (encoder.use_quadrature) {
      long count_snapshot;
      noInterrupts();
      count_snapshot = encoder.count;
      interrupts();
      Serial.print("count=");
      Serial.println(count_snapshot);
    } else {
      long count_left_snapshot;
      long count_right_snapshot;
      noInterrupts();
      count_left_snapshot = bldc_count_left;
      count_right_snapshot = bldc_count_right;
      interrupts();

      long diff_left = count_left_snapshot - last_count_left;
      long diff_right = count_right_snapshot - last_count_right;
      float rev_left = static_cast<float>(diff_left) / static_cast<float>(kSpeedOutPulsesPerRev);
      float rev_right = static_cast<float>(diff_right) / static_cast<float>(kSpeedOutPulsesPerRev);
      float rpm_left = rev_left * (60000.0f / static_cast<float>(kReportMs));
      float rpm_right = rev_right * (60000.0f / static_cast<float>(kReportMs));
      bool forward_left = (readDirectionSign(PIN_DIRECTION_L, kDirectionHighIsForwardLeft) > 0);
      bool forward_right = (readDirectionSign(PIN_DIRECTION_R, kDirectionHighIsForwardRight) > 0);

      Serial.print("L:count=");
      Serial.print(count_left_snapshot);
      Serial.print(",diff=");
      Serial.print(diff_left);
      Serial.print(",rpm_est=");
      Serial.print(rpm_left, 2);
      Serial.print(",dir=");
      Serial.print(forward_left ? "FWD" : "REV");
      Serial.print(" | R:count=");
      Serial.print(count_right_snapshot);
      Serial.print(",diff=");
      Serial.print(diff_right);
      Serial.print(",rpm_est=");
      Serial.print(rpm_right, 2);
      Serial.print(",dir=");
      Serial.println(forward_right ? "FWD" : "REV");

      last_count_left = count_left_snapshot;
      last_count_right = count_right_snapshot;
    }
  }
}
