#ifndef PID_H
#define PID_H

#include <Arduino.h>

struct PID {
  float kp, ki, kd;
  float i_min, i_max;
  float out_min, out_max;
  float integrator;
  float prev_err;
  bool  first;
};

inline void pidInit(PID &p, float kp, float ki, float kd,
                    float i_min, float i_max, float out_min, float out_max) {
  p.kp = kp; p.ki = ki; p.kd = kd;
  p.i_min = i_min; p.i_max = i_max;
  p.out_min = out_min; p.out_max = out_max;
  p.integrator = 0.0f;
  p.prev_err = 0.0f;
  p.first = true;
}

inline float pidStep(PID &p, float setpoint, float measurement, float dt, float d_meas = NAN) {
  float err = setpoint - measurement;
  float P = p.kp * err;
  p.integrator += err * dt * p.ki;
  if (p.integrator > p.i_max) p.integrator = p.i_max;
  if (p.integrator < p.i_min) p.integrator = p.i_min;
  float I = p.integrator;
  float deriv;
  if (!isnan(d_meas)) {
    deriv = -d_meas;
  } else {
    deriv = p.first ? 0.0f : (err - p.prev_err) / max(dt, 1e-6f);
  }
  float D = p.kd * deriv;
  p.prev_err = err;
  p.first = false;

  float out = P + I + D;
  if (out > p.out_max) out = p.out_max;
  if (out < p.out_min) out = p.out_min;
  return out;
}

#endif // PID_H
