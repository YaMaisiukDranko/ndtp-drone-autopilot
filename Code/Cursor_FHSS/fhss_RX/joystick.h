#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>

// Shared joystick structure used across the project
struct JoystickData {
    int16_t x_left, y_left;   // Left stick: throttle/yaw
    int16_t x_right, y_right; // Right stick: pitch/roll
};

#endif // JOYSTICK_H
