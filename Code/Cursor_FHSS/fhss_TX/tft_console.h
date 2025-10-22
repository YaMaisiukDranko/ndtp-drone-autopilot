#ifndef TFT_CONSOLE_H
#define TFT_CONSOLE_H

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// ====== Пины дисплея (5-проводное SPI: CS, DC, RST, MOSI, SCLK) ======
#ifndef TFT_CS_PIN
#define TFT_CS_PIN    10
#endif
#ifndef TFT_DC_PIN
#define TFT_DC_PIN    9
#endif
#ifndef TFT_RST_PIN
#define TFT_RST_PIN   8
#endif
#ifndef TFT_MOSI_PIN
#define TFT_MOSI_PIN  11   // DIN / SDA
#endif
#ifndef TFT_SCLK_PIN
#define TFT_SCLK_PIN  13   // CLK / SCL
#endif

// ====== Тумблер ARM/DISARM ======
#ifndef ARM_SWITCH_PIN
#define ARM_SWITCH_PIN 19
#endif
#ifndef ARM_ACTIVE_LEVEL
#define ARM_ACTIVE_LEVEL LOW   // поменяй на LOW, если тумблер замыкает на GND
#endif

// ====== Цвета ======
#define COL_BG        ST77XX_BLACK
#define COL_TEXT      ST77XX_WHITE
#define COL_ARMED_BG  ST77XX_GREEN
#define COL_ARMED_TXT ST77XX_BLACK
#define COL_DIS_BG    ST77XX_RED
#define COL_DIS_TXT   ST77XX_WHITE

// Высота баннера сверху
#ifndef BANNER_H
#define BANNER_H 14
#endif

class TftConsole {
public:
  // Инициализация (использует 5-пиновый конструктор Adafruit_ST7735)
  void begin();

  // Обновить баннер Armed/Disarmed по состоянию пина ARM_SWITCH_PIN
  void updateArmingBanner();

  // Считать байты из Serial и вывести в консоль
  void updateFromSerial(HardwareSerial &ser = Serial);

  // Напечатать строку программно (без Serial)
  void println(const String& s);

  // Очистить консольную область (без баннера)
  void clearConsole();

  // Доступ к драйверу (если нужно)
  Adafruit_ST7735& tft() { return *_tft; }

private:
  // Вариант с динамическим конструктором "на 5 пинов"
  Adafruit_ST7735* _tft = nullptr;

  // Для 128x160 при шрифте 5x7 и межстрочном ~8px
  static const uint8_t MAX_LINES = 18;
  String _lines[MAX_LINES];
  String _current;
  uint8_t _lineCount = 0;
  bool _inited = false;

  void _drawBanner(bool armed);
  void _redrawAll();
  void _appendChar(char c);
};

#endif // TFT_CONSOLE_H
