#include <Arduino.h>
#include <math.h>
#include "attitude.h"

static float s_alpha = 0.98f;
static bool  s_first = true;
static float s_roll = 0.0f;
static float s_pitch = 0.0f;
static float s_yaw = 0.0f;

void attitudeInit(float alpha) {
  s_alpha = constrain(alpha, 0.0f, 1.0f);
  s_first = true;
  s_roll = s_pitch = s_yaw = 0.0f;
}

static inline float rad2deg(float r) { return r * 57.2957795f; }

void attitudeUpdate(const TelemetryData& sens, float dt, Attitude* out) {
  float gx = sens.gyro_x;
  float gy = sens.gyro_y;
  float gz = sens.gyro_z;

  float roll_g  = s_roll  + gx * dt;
  float pitch_g = s_pitch + gy * dt;
  float yaw_g   = s_yaw   + gz * dt;

  float ax = sens.accel_x;
  float ay = sens.accel_y;
  float az = sens.accel_z;
  float roll_acc  = rad2deg(atan2f(ay, az));
  float pitch_acc = rad2deg(atan2f(-ax, sqrtf(ay*ay + az*az)));

  if (s_first) {
    s_roll = roll_acc;
    s_pitch = pitch_acc;
    s_yaw = 0.0f;
    s_first = false;
  } else {
    s_roll  = s_alpha * roll_g  + (1.0f - s_alpha) * roll_acc;
    s_pitch = s_alpha * pitch_g + (1.0f - s_alpha) * pitch_acc;
    s_yaw   = yaw_g;
  }

  out->roll  = s_roll;
  out->pitch = s_pitch;
  out->yaw   = s_yaw;
  out->gx = gx; out->gy = gy; out->gz = gz;
}
