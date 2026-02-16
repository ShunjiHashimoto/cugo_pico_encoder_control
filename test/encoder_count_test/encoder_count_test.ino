#include <Arduino.h>

// Set to 1 for left encoder, 0 for right encoder.
#ifndef USE_LEFT_ENCODER
#define USE_LEFT_ENCODER 0
#endif

// 0: DC motor driver (quadrature encoder)
// 1: BLDC driver HM-5100J (SPEED-OUT)
// 2: BLDC driver HP-5097 (SPEED-OUT)
#ifndef DRIVER_TYPE
#define DRIVER_TYPE 0
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
  if (encoder.use_quadrature) {
    updateQuadratureEncoder();
  } else {
    encoder.count++;
  }
}

void setup() {
  Serial.begin(kBaud);

#if USE_LEFT_ENCODER
  Serial.println("Encoder: LEFT");
#else
  Serial.println("Encoder: RIGHT");
#endif

  encoder.use_quadrature = (DRIVER_TYPE == DRIVER_DC);

#if USE_LEFT_ENCODER
#if DRIVER_TYPE == DRIVER_DC
  encoder.pin_a = PIN_ENCODER_L_A;
  encoder.pin_b = PIN_ENCODER_L_B;
  encoder.reverse = kLeftReverse;
#else
  encoder.pin_a = PIN_SPEED_OUT_L;
  encoder.pin_b = 255;
  encoder.reverse = false;
#endif
#else
#if DRIVER_TYPE == DRIVER_DC
  encoder.pin_a = PIN_ENCODER_R_A;
  encoder.pin_b = PIN_ENCODER_R_B;
  encoder.reverse = kRightReverse;
#else
  encoder.pin_a = PIN_SPEED_OUT_R;
  encoder.pin_b = 255;
  encoder.reverse = false;
#endif
#endif

#if DRIVER_TYPE == DRIVER_DC
  Serial.println("Driver: DC (quadrature)");
#elif DRIVER_TYPE == DRIVER_BLDC_HM5100J
  Serial.println("Driver: BLDC HM-5100J (SPEED-OUT)");
#elif DRIVER_TYPE == DRIVER_BLDC_HP5097
  Serial.println("Driver: BLDC HP-5097 (SPEED-OUT)");
#else
#error "Unsupported DRIVER_TYPE. Use 0, 1, or 2."
#endif

  pinMode(encoder.pin_a, INPUT_PULLUP);
  if (encoder.use_quadrature) {
    pinMode(encoder.pin_b, INPUT_PULLUP);
    uint8_t initial_a = digitalRead(encoder.pin_a);
    uint8_t initial_b = digitalRead(encoder.pin_b);
    encoder.last_state = static_cast<uint8_t>((initial_a << 1) | (initial_b & 0x01));
    attachInterrupt(digitalPinToInterrupt(encoder.pin_a), encoderHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder.pin_b), encoderHandler, CHANGE);
  } else {
    encoder.last_state = 0;
    attachInterrupt(digitalPinToInterrupt(encoder.pin_a), encoderHandler, RISING);
    Serial.print("SPEED-OUT pulses/rev: ");
    Serial.println(kSpeedOutPulsesPerRev);
  }
  encoder.count = 0;
}

void loop() {
  static unsigned long last_report = 0;
  static long last_count = 0;
  if (millis() - last_report >= kReportMs) {
    last_report = millis();
    long count_snapshot;
    noInterrupts();
    count_snapshot = encoder.count;
    interrupts();
    Serial.print("count=");
    Serial.print(count_snapshot);
    if (!encoder.use_quadrature) {
      long diff = count_snapshot - last_count;
      float rev = static_cast<float>(diff) / static_cast<float>(kSpeedOutPulsesPerRev);
      float rpm = rev * (60000.0f / static_cast<float>(kReportMs));
      Serial.print(", diff=");
      Serial.print(diff);
      Serial.print(", rpm_est=");
      Serial.print(rpm, 2);
      last_count = count_snapshot;
    }
    Serial.println();
  }
}
