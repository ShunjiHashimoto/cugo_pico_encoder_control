#include <Arduino.h>

constexpr uint8_t ADC_BAT_PIN = 26;   // ADC_BAT (GPIO26)
constexpr uint8_t LED_PWM_PIN = 20;   // BAT_LED_PWM (GPIO20)

constexpr uint32_t kBaud = 115200;
constexpr uint32_t kSampleMs = 200;

constexpr uint16_t kAdcMax = 4095;
constexpr float kAdcVref = 3.3f;      // Calibrate if needed.
constexpr float kDividerRatio = 11.0f; // 100k / 10k divider -> Vbat = Vadc * 11

constexpr float kBatMinV = 22.0f;     // Duty is 0 at ~22V and below.
constexpr float kBatMaxV = 25.2f;     // Adjust to your battery pack.

constexpr uint16_t kPwmRange = 255;
constexpr uint16_t kPwmMin = 0;
constexpr uint16_t kPwmMax = 255;

void setup() {
  Serial.begin(kBaud);
  pinMode(LED_PWM_PIN, OUTPUT);
  analogReadResolution(12);
  analogWriteRange(kPwmRange);
}

void loop() {
  int raw = analogRead(ADC_BAT_PIN);
  float v_adc = (static_cast<float>(raw) / kAdcMax) * kAdcVref;
  float v_bat = v_adc * kDividerRatio;

  float clamped = constrain(v_bat, kBatMinV, kBatMaxV);
  float ratio = (clamped - kBatMinV) / (kBatMaxV - kBatMinV);
  int duty = static_cast<int>(ratio * (kPwmMax - kPwmMin) + kPwmMin + 0.5f);
  if (duty < kPwmMin) duty = kPwmMin;
  if (duty > kPwmMax) duty = kPwmMax;

  analogWrite(LED_PWM_PIN, duty);

  Serial.print("ADC_BAT raw=");
  Serial.print(raw);
  Serial.print(", Vadc=");
  Serial.print(v_adc, 3);
  Serial.print(" V, Vbat=");
  Serial.print(v_bat, 3);
  Serial.print(" V, duty=");
  Serial.println(duty);

  delay(kSampleMs);
}
