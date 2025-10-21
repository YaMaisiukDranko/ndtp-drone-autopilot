#ifndef STABILIZER_H
#define STABILIZER_H

#include <Arduino.h>
#include "pid.h"
#include "attitude.h"
#include "telemetry.h"
#include "joystick.h" // full definition of JoystickData

void stabilizerInit();

void stabilizeMix(const JoystickData& js, const TelemetryData& sens, float dt,
                  uint8_t* m1, uint8_t* m2, uint8_t* m3, uint8_t* m4);

#endif // STABILIZER_H
