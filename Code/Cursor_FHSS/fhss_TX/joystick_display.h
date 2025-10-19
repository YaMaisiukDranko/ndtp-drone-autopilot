#ifndef JOYSTICK_DISPLAY_H
#define JOYSTICK_DISPLAY_H

#include <Arduino.h>

// Forward-declare display type to avoid hard dependency here.
// Include the specific display header in the .ino implementation.

// Normalized joystick values are expected in range [-1.0, 1.0]
// where (0,0) is centered. Values outside the range will be clamped.

// Initialize display subsystem. Returns true if successful.
bool joystickDisplayBegin();

// Show a short calibration message at startup.
void joystickDisplayShowCalibration(const char* messageLine1, const char* messageLine2, uint16_t ms);

// Render two joystick circles with points at given normalized positions.
// leftX, leftY: left stick; rightX, rightY: right stick.
void joystickDisplayRender(float leftX, float leftY, float rightX, float rightY);

#endif // JOYSTICK_DISPLAY_H



