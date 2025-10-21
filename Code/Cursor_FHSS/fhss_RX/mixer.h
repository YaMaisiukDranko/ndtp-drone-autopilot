#ifndef MIXER_H
#define MIXER_H

#include <Arduino.h>

#ifndef MOTOR1_PIN
#define MOTOR1_PIN 1
#endif
#ifndef MOTOR2_PIN
#define MOTOR2_PIN 2
#endif
#ifndef MOTOR3_PIN
#define MOTOR3_PIN 3
#endif
#ifndef MOTOR4_PIN
#define MOTOR4_PIN 4
#endif

void mixerInit();
void mixerWrite(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4);

#endif // MIXER_H
