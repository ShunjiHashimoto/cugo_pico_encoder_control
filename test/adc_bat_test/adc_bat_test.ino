#include <Arduino.h>

constexpr uint8_t ADC_BAT_PIN = 26;
constexpr uint32_t kBaud = 115200;
constexpr uint32_t kSampleMs = 200;
constexpr uint16_t kAdcMax = 4095;
constexpr float kAdcVref = 3.3f;
constexpr float kDividerRatio = 10.85f;  // Adjust if a resistor divider is used.

void setup() {
  Serial.begin(kBaud);
  analogReadResolution(12);
}

void loop() {
  int raw = analogRead(ADC_BAT_PIN);
  float v_adc = (static_cast<float>(raw) / kAdcMax) * kAdcVref;
  float v_bat = v_adc * kDividerRatio;
  Serial.print("ADC_BAT raw=");
  Serial.print(raw);
  Serial.print(", Vadc=");
  Serial.print(v_adc, 3);
  Serial.print(" V, Vbat=");
  Serial.print(v_bat, 3);
  Serial.println(" V");
  delay(kSampleMs);
}
