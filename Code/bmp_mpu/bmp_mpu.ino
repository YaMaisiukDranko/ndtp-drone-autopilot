#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>

// Определяем пины для I2C
#define SDA_PIN 4
#define SCL_PIN 5

// Создаем объекты датчиков
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;

// Функция для инициализации I2C с нестандартными пинами
void setupI2C() {
  Wire.begin(SDA_PIN, SCL_PIN);
}

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10); // Ждем открытия Serial Monitor

  // Инициализируем I2C
  setupI2C();

  // Инициализируем MPU6050
  if (!mpu.begin()) {
    Serial.println("Не удалось найти MPU6050!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 найден!");

  // Настройки MPU6050 (опционально)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Инициализируем BMP280 (адрес по умолчанию 0x76, если 0x77 - используйте & 0x77)
  if (!bmp.begin(0x76)) {
    Serial.println("Не удалось найти BMP280!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BMP280 найден!");

  Serial.println("Датчики инициализированы. Начинаем чтение данных...");
}

void loop() {
  // Читаем данные с MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Читаем данные с BMP280
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0F; // в гПа

  // Выводим данные в Serial Monitor
  Serial.println("--- Данные с датчиков ---");
  
  Serial.print("MPU6050 - Температура: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");
  
  Serial.print("MPU6050 - Акселерометр X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.println(a.acceleration.z);
  Serial.println(" м/с²");
  
  Serial.print("MPU6050 - Гироскоп X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.println(g.gyro.z);
  Serial.println(" рад/с");
  
  Serial.print("BMP280 - Температура: ");
  Serial.print(temperature);
  Serial.println(" °C");
  
  Serial.print("BMP280 - Давление: ");
  Serial.print(pressure);
  Serial.println(" гПа");
  
  Serial.println("-------------------------");
  Serial.println();

  delay(1000); // Пауза 1 секунда между чтениями
}