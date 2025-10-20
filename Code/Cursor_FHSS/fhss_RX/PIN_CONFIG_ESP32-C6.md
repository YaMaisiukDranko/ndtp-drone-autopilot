# Конфигурация пинов для ESP32-C6 Supermini

## Подключение nRF24L01 (SPI)
```
ESP32-C6 Supermini  →  nRF24L01
────────────────────────────────
GPIO 7 (CE)         →  CE
GPIO 8 (CSN)        →  CSN  
GPIO 9 (SCK)        →  SCK
GPIO 14 (MOSI)      →  MOSI
GPIO 15 (MISO)      →  MISO
3.3V                →  VCC
GND                 →  GND
```

## Подключение датчиков (I2C)
```
ESP32-C6 Supermini  →  Датчики
────────────────────────────────
GPIO 18 (SDA)       →  SDA (MPU6050, BMP280, VL53L0X)
GPIO 19 (SCL)       →  SCL (MPU6050, BMP280, VL53L0X)
3.3V                →  VCC (все датчики)
GND                 →  GND (все датчики)
```

## Дополнительные подключения датчиков

### MPU6050
- VCC → 3.3V
- GND → GND
- SCL → GPIO 19
- SDA → GPIO 18

### BMP280
- VCC → 3.3V
- GND → GND
- SCK → GPIO 19 (SCL)
- SDI → GPIO 18 (SDA)

### VL53L0X (TOF200C)
- VIN → 3.3V
- GND → GND
- SCL → GPIO 19
- SDA → GPIO 18
- XSHUT → не подключен (или к любому GPIO для сброса)

## Важные замечания
- Все датчики работают от 3.3V
- I2C шина общая для всех датчиков
- Убедитесь в правильности подключения питания
- При проблемах с I2C проверьте pull-up резисторы (обычно встроены в датчики)

