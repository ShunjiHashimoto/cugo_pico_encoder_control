#include <Arduino.h>

// Set to 1 for left encoder, 0 for right encoder.
#ifndef USE_LEFT_ENCODER
#define USE_LEFT_ENCODER 0
#endif

constexpr uint8_t PIN_ENCODER_L_A = 2;
constexpr uint8_t PIN_ENCODER_L_B = 3;
constexpr uint8_t PIN_ENCODER_R_A = 8;
constexpr uint8_t PIN_ENCODER_R_B = 9;

constexpr bool kLeftReverse = false;
constexpr bool kRightReverse = true;

constexpr uint32_t kBaud = 115200;
constexpr uint32_t kReportMs = 200;

struct EncoderState {
  volatile long count;
  volatile uint8_t last_state;
  uint8_t pin_a;
  uint8_t pin_b;
  bool reverse;
};

EncoderState encoder;

void updateEncoder() {
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
  updateEncoder();
}

void setup() {
  Serial.begin(kBaud);

#if USE_LEFT_ENCODER
  encoder.pin_a = PIN_ENCODER_L_A;
  encoder.pin_b = PIN_ENCODER_L_B;
  encoder.reverse = kLeftReverse;
  Serial.println("Encoder: LEFT");
#else
  encoder.pin_a = PIN_ENCODER_R_A;
  encoder.pin_b = PIN_ENCODER_R_B;
  encoder.reverse = kRightReverse;
  Serial.println("Encoder: RIGHT");
#endif

  pinMode(encoder.pin_a, INPUT_PULLUP);
  pinMode(encoder.pin_b, INPUT_PULLUP);

  uint8_t initial_a = digitalRead(encoder.pin_a);
  uint8_t initial_b = digitalRead(encoder.pin_b);
  encoder.last_state = static_cast<uint8_t>((initial_a << 1) | (initial_b & 0x01));
  encoder.count = 0;

  attachInterrupt(digitalPinToInterrupt(encoder.pin_a), encoderHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder.pin_b), encoderHandler, CHANGE);
}

void loop() {
  static unsigned long last_report = 0;
  if (millis() - last_report >= kReportMs) {
    last_report = millis();
    long count_snapshot;
    noInterrupts();
    count_snapshot = encoder.count;
    interrupts();
    Serial.print("count=");
    Serial.println(count_snapshot);
  }
}
