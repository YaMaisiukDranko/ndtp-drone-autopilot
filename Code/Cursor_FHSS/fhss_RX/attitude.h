#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <Arduino.h>
#include "telemetry.h"

struct Attitude {
  float roll;   // deg
  float pitch;  // deg
  float yaw;    // deg
  float gx;     // deg/s
  float gy;     // deg/s
  float gz;     // deg/s
};

void attitudeInit(float alpha = 0.98f);
void attitudeUpdate(const TelemetryData& sens, float dt, Attitude* out);

#endif // ATTITUDE_H
