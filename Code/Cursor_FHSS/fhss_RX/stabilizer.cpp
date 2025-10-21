
#include "stabilizer.h"
#include "mixer.h"

static PID pid_roll;
static PID pid_pitch;
static PID pid_yaw_rate;

// Arming / safety
static bool s_armed = false;
static uint32_t s_holdStartMs = 0;
static const int16_t STICK_ARM_HOLD = 800;   // мс удержания
static const int16_t STICK_MIN = -900;       // низкий газ
static const int16_t STICK_EDGE =  900;      // край по рысканию
static const uint8_t IDLE_PWM = 0;           // 0 => моторы стоят до реального газа

// Helper
static inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Deadband
static inline int16_t deadband(int16_t v, int16_t db=50) {
  return (abs(v) <= db) ? 0 : v;
}

// Throttle mapping: -1000..+1000 -> 0..255 (верх = больше)
static inline uint8_t throttleToPwm(int16_t y_left) {
  if (y_left < -1000) y_left = -1000;
  if (y_left >  1000) y_left =  1000;
  long shifted = (long)(y_left + 1000); // 0..2000
  long pwm = (shifted * 255L) / 2000L;  // 0..255
  if (pwm < 0)   pwm = 0;
  if (pwm > 255) pwm = 255;
  return (uint8_t)pwm;
}

void stabilizerInit() {
  attitudeInit(0.98f);
  pidInit(pid_roll,  3.0f, 0.0f, 0.08f, -50.0f, 50.0f, -200.0f, 200.0f);
  pidInit(pid_pitch, 3.0f, 0.0f, 0.08f, -50.0f, 50.0f, -200.0f, 200.0f);
  pidInit(pid_yaw_rate, 1.2f, 0.0f, 0.02f, -50.0f, 50.0f, -200.0f, 200.0f);
  s_armed = false;
  s_holdStartMs = 0;
}

// Handle arm/disarm stick combos (hold STICK_ARM_HOLD ms)
static void updateArming(const JoystickData& js) {
  uint32_t now = millis();
  bool throttleLow = (js.y_left <= STICK_MIN);
  bool armCombo   = throttleLow && (js.x_left <= -STICK_EDGE); // газ низко + рыскание влево
  bool disarmCombo= throttleLow && (js.x_left >=  STICK_EDGE); // газ низко + рыскание вправо

  static bool lastArmState=false;
  static bool lastDisarmState=false;

  if (armCombo) {
    if (!lastArmState) { s_holdStartMs = now; }
    if (now - s_holdStartMs >= (uint32_t)STICK_ARM_HOLD) s_armed = true;
    lastArmState = true;
  } else {
    lastArmState = false;
  }

  if (disarmCombo) {
    if (!lastDisarmState) { s_holdStartMs = now; }
    if (now - s_holdStartMs >= (uint32_t)STICK_ARM_HOLD) s_armed = false;
    lastDisarmState = true;
  } else {
    lastDisarmState = false;
  }
}

void stabilizeMix(const JoystickData& js_raw, const TelemetryData& sens, float dt,
                  uint8_t* m1, uint8_t* m2, uint8_t* m3, uint8_t* m4)
{
  // 0) RC preprocessing
  JoystickData js = js_raw;
  js.x_left  = deadband(js.x_left);
  js.x_right = deadband(js.x_right);
  js.y_right = deadband(js.y_right);
  // y_left (throttle) без мёртвой зоны, но используем порог в логике ниже

  // 1) Update arming state
  updateArming(js);

  // 2) Attitude estimation
  Attitude att;
  attitudeUpdate(sens, dt, &att);

  // Throttle
  uint8_t throttle_pwm = throttleToPwm(js.y_left);

  // Safety: if not armed OR throttle stick is near bottom => motors off, reset PIDs
  if (!s_armed || js.y_left <= (STICK_MIN + 50)) {
    pidInit(pid_roll,  pid_roll.kp,  pid_roll.ki,  pid_roll.kd,  pid_roll.i_min,  pid_roll.i_max,  pid_roll.out_min,  pid_roll.out_max);
    pidInit(pid_pitch, pid_pitch.kp, pid_pitch.ki, pid_pitch.kd, pid_pitch.i_min, pid_pitch.i_max, pid_pitch.out_min, pid_pitch.out_max);
    pidInit(pid_yaw_rate, pid_yaw_rate.kp, pid_yaw_rate.ki, pid_yaw_rate.kd, pid_yaw_rate.i_min, pid_yaw_rate.i_max, pid_yaw_rate.out_min, pid_yaw_rate.out_max);

    *m1 = *m2 = *m3 = *m4 = 0; // полностью остановить
    return;
  }

  // 3) Setpoints (deg / deg/s)
  float sp_roll      = mapf((float)js.x_right, -1000.0f, 1000.0f, -25.0f, 25.0f);
  float sp_pitch     = mapf((float)js.y_right, -1000.0f, 1000.0f, -25.0f, 25.0f);
  float sp_yaw_rate  = mapf((float)js.x_left,  -1000.0f, 1000.0f, -150.0f, 150.0f);

  // 4) Controllers (scale with throttle to calm near-idle)
  float throttle_scale = throttle_pwm / 255.0f; // 0..1
  float u_roll  = throttle_scale * pidStep(pid_roll,  sp_roll,  att.roll,  dt, att.gx);
  float u_pitch = throttle_scale * pidStep(pid_pitch, sp_pitch, att.pitch, dt, att.gy);
  float u_yaw   = throttle_scale * pidStep(pid_yaw_rate, sp_yaw_rate, att.gz, dt);

  // 5) Mixer (X)
  float base = (float)max<uint8_t>(throttle_pwm, IDLE_PWM);
  float m1f = base + u_roll + u_pitch - u_yaw; // Front Left
  float m2f = base - u_roll + u_pitch + u_yaw; // Front Right
  float m3f = base - u_roll - u_pitch - u_yaw; // Rear Right
  float m4f = base + u_roll - u_pitch + u_yaw; // Rear Left

  auto sat = [](float v){ if (v < 0) v = 0; if (v > 255) v = 255; return (uint8_t)(v + 0.5f); };
  *m1 = sat(m1f);
  *m2 = sat(m2f);
  *m3 = sat(m3f);
  *m4 = sat(m4f);
}
