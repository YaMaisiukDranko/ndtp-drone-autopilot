#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_VL53L0X.h>

// ====== Pin configuration ======
#define SDA_PIN 8
#define SCL_PIN 9

// ====== Sensor objects ======
extern Adafruit_MPU6050 mpu;
extern Adafruit_BMP280 bmp;
extern Adafruit_VL53L0X lox;

// ====== Telemetry data structure ======
struct TelemetryData {
    // MPU6050 Accelerometer (X, Y, Z)
    float accel_x;
    float accel_y;
    float accel_z;
    
    // MPU6050 Gyroscope (X, Y, Z)
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    // BMP280 Pressure (in hPa)
    float pressure;
    
    // VL53L0X Distance (in mm)
    float distance;
};

// ====== Function declarations ======
bool initializeTelemetrySensors();
void readTelemetryData(TelemetryData* data);
void formatTelemetryString(const TelemetryData* data, char* output, size_t maxLen);
bool isTelemetryValid(const TelemetryData* data);

#endif // TELEMETRY_H
