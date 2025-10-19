#include "telemetry.h"

// ====== Global sensor objects ======
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;

// ====== Sensor initialization ======
bool initializeTelemetrySensors() {
    // Initialize I2C with custom pins
    Wire.begin(SDA_PIN, SCL_PIN);
    
    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("TELEMETRY: Failed to find MPU6050!");
        return false;
    }
    Serial.println("TELEMETRY: MPU6050 found!");
    
    // Initialize BMP280 (default address 0x76, if different specify 0x77)
    if (!bmp.begin(0x76)) {
        Serial.println("TELEMETRY: Failed to find BMP280!");
        return false;
    }
    Serial.println("TELEMETRY: BMP280 found!");
    
    
    Serial.println("TELEMETRY: All sensors initialized successfully!");
    return true;
}

// ====== Read data from all sensors ======
void readTelemetryData(TelemetryData* data) {
    if (data == nullptr) return;
    
    // Initialize with invalid values
    data->accel_x = data->accel_y = data->accel_z = 0.0f;
    data->gyro_x = data->gyro_y = data->gyro_z = 0.0f;
    data->pressure = 0.0f;
    
    // Read MPU6050 data (accelerometer, gyroscope)
    sensors_event_t a, g, temp_mpu;
    if (mpu.getEvent(&a, &g, &temp_mpu)) {
        data->accel_x = a.acceleration.x;
        data->accel_y = a.acceleration.y;
        data->accel_z = a.acceleration.z;
        
        data->gyro_x = g.gyro.x;
        data->gyro_y = g.gyro.y;
        data->gyro_z = g.gyro.z;
    }
    
    // Read BMP280 data (pressure)
    float pressure_pa = bmp.readPressure();
    if (pressure_pa > 0) {
        data->pressure = pressure_pa / 100.0F; // Convert to hPa
    }
    
}

// ====== Format telemetry data as string ======
void formatTelemetryString(const TelemetryData* data, char* output, size_t maxLen) {
    if (data == nullptr || output == nullptr || maxLen == 0) return;
    
    // Ultra-compact format to fit in 24 bytes
    // Format: aX:aY:aZ.gX:gY:gZ.p
    // Example: 1.2:4.5:7.8.0.1:0.3:0.5.1013
    
    int len = snprintf(output, maxLen, "%.1f:%.1f:%.1f.%.1f:%.1f:%.1f.%.0f",
        data->accel_x, data->accel_y, data->accel_z,
        data->gyro_x, data->gyro_y, data->gyro_z,
        data->pressure);
    
    // If still too long, use even more compact format
    if (len >= (int)maxLen - 1) {
        len = snprintf(output, maxLen, "%.0f:%.0f:%.0f.%.0f:%.0f:%.0f.%.0f",
            data->accel_x, data->accel_y, data->accel_z,
            data->gyro_x, data->gyro_y, data->gyro_z,
            data->pressure);
    }
    
    // Ensure null termination
    if (len >= (int)maxLen) {
        output[maxLen - 1] = '\0';
    }
}

// ====== Validate telemetry data ======
bool isTelemetryValid(const TelemetryData* data) {
    if (data == nullptr) return false;
    
    // Check for reasonable ranges
    // Accelerometer: typically -20 to +20 m/s²
    if (data->accel_x < -50.0f || data->accel_x > 50.0f) return false;
    if (data->accel_y < -50.0f || data->accel_y > 50.0f) return false;
    if (data->accel_z < -50.0f || data->accel_z > 50.0f) return false;
    
    // Gyroscope: typically -2000 to +2000 dps
    if (data->gyro_x < -5000.0f || data->gyro_x > 5000.0f) return false;
    if (data->gyro_y < -5000.0f || data->gyro_y > 5000.0f) return false;
    if (data->gyro_z < -5000.0f || data->gyro_z > 5000.0f) return false;
    
    // Pressure: typically 800 to 1200 hPa
    if (data->pressure < 500.0f || data->pressure > 1500.0f) return false;
    
    
    return true;
}
