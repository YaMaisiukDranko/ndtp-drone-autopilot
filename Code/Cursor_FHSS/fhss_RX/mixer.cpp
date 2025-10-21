#include "mixer.h"

void mixerInit() {
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  pinMode(MOTOR3_PIN, OUTPUT);
  pinMode(MOTOR4_PIN, OUTPUT);
}

void mixerWrite(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4) {
  analogWrite(MOTOR1_PIN, m1);
  analogWrite(MOTOR2_PIN, m2);
  analogWrite(MOTOR3_PIN, m3);
  analogWrite(MOTOR4_PIN, m4);
}
