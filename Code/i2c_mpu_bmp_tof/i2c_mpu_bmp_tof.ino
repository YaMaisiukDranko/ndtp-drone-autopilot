#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_VL53L0X.h>

// Определяем пины для I2C
#define SDA_PIN 8
#define SCL_PIN 9

// Создаем объекты сенсоров
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  // Инициализация серийного порта для вывода данных
  Serial.begin(9600);
  while (!Serial) delay(10); // Ждем подключения серийного монитора

  // Инициализация I2C с кастомными пинами
  Wire.begin(SDA_PIN, SCL_PIN);

  // Инициализация MPU6050
  if (!mpu.begin()) {
    Serial.println("Не удалось найти MPU6050!");
    while (1) delay(10);
  }
  Serial.println("MPU6050 найден!");

  // Инициализация BMP280 (адрес по умолчанию 0x76, если другой - укажите 0x77)
  if (!bmp.begin(0x76)) {
    Serial.println("Не удалось найти BMP280!");
    while (1) delay(10);
  }
  Serial.println("BMP280 найден!");

  // Инициализация VL53L0X (TOF200C)
  if (!lox.begin()) {
    Serial.println("Не удалось найти VL53L0X!");
    while (1) delay(10);
  }
  Serial.println("VL53L0X найден!");

  Serial.println("Все сенсоры инициализированы. Начинаем чтение данных...");
  Serial.println("---");
}

void loop() {
  // Чтение данных с MPU6050 (акселерометр, гироскоп, температура)
  sensors_event_t a, g, temp_mpu;
  mpu.getEvent(&a, &g, &temp_mpu);

  Serial.print("Акцелерометр X: "); Serial.print(a.acceleration.x);
  Serial.print(" Y: "); Serial.print(a.acceleration.y);
  Serial.print(" Z: "); Serial.println(a.acceleration.z);
  
  Serial.print("Гироскоп X: "); Serial.print(g.gyro.x);
  Serial.print(" Y: "); Serial.print(g.gyro.y);
  Serial.print(" Z: "); Serial.println(g.gyro.z);
  
  Serial.print("Температура MPU: "); Serial.println(temp_mpu.temperature);

  // Чтение данных с BMP280 (температура, давление)
  float temperature_bmp = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0F; // в гПа

  Serial.print("Температура BMP: "); Serial.print(temperature_bmp);
  Serial.println(" °C");
  
  Serial.print("Давление: "); Serial.print(pressure);
  Serial.println(" гПа");

  // Чтение данных с VL53L0X (расстояние)
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // false - однократное измерение

  if (measure.RangeStatus != 4) { // 4 значит "Out of range"
    Serial.print("Расстояние: "); Serial.print(measure.RangeMilliMeter);
    Serial.println(" мм");
  } else {
    Serial.println("Расстояние: Out of range");
  }

  Serial.println("---");
  delay(1000); // Пауза 1 секунда между чтениями
}