#include <Arduino.h>

constexpr uint8_t SW1_PIN = 15; //14 or 15(=mode switch)
constexpr uint8_t LED1_PIN = 21;
constexpr uint32_t kBaud = 115200;
constexpr uint32_t kPollMs = 10;

void setup() {
  Serial.begin(kBaud);
  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(LED1_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
}

void loop() {
  static int last_state = HIGH;
  static bool led_on = false;
  int state = digitalRead(SW1_PIN);
  if (state == LOW && last_state == HIGH) {
    led_on = !led_on;
    digitalWrite(LED1_PIN, led_on ? HIGH : LOW);
    Serial.print("LED1: ");
    Serial.println(led_on ? "on" : "off");
  }
  if (state != last_state) {
    last_state = state;
    Serial.print("SW1: ");
    Serial.println(state == LOW ? "pressed" : "released");
  }
  delay(kPollMs);
}
