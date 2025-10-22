#include "tft_console.h"

void TftConsole::begin() {
  if (_inited) return;

  // Конструктор на 5 пинов: CS, DC, MOSI, SCLK, RST
  _tft = new Adafruit_ST7735(TFT_CS_PIN, TFT_DC_PIN, TFT_MOSI_PIN, TFT_SCLK_PIN, TFT_RST_PIN);

  _tft->initR(INITR_BLACKTAB);  // ST7735S (BLACKTAB — самая частая ревизия)
  _tft->setRotation(1);         // повернуть при необходимости: 0/1/2/3
  _tft->fillScreen(COL_BG);
  _tft->setTextWrap(false);
  _tft->setTextSize(1);         // базовый шрифт 5x7; высота строки ~8 px
  _tft->setTextColor(COL_TEXT, COL_BG);

  pinMode(ARM_SWITCH_PIN, INPUT_PULLUP);
  _drawBanner(false);
  clearConsole();

  _inited = true;
}

void TftConsole::_drawBanner(bool armed) {
  _tft->fillRect(0, 0, _tft->width(), BANNER_H, armed ? COL_ARMED_BG : COL_DIS_BG);
  _tft->setCursor(2, 3);
  _tft->setTextColor(armed ? COL_ARMED_TXT : COL_DIS_TXT, armed ? COL_ARMED_BG : COL_DIS_BG);
  _tft->print(armed ? F("ARMED") : F("DISARMED"));
  _tft->setTextColor(COL_TEXT, COL_BG);
}

void TftConsole::updateArmingBanner() {
  bool pinState = digitalRead(ARM_SWITCH_PIN);
  bool armed = (pinState == ARM_ACTIVE_LEVEL);
  _drawBanner(armed);
}

void TftConsole::clearConsole() {
  // Очистить область ниже баннера
  _tft->fillRect(0, BANNER_H, _tft->width(), _tft->height() - BANNER_H, COL_BG);
  for (uint8_t i=0; i<MAX_LINES; ++i) _lines[i] = "";
  _current = "";
  _lineCount = 0;
  _redrawAll();
}

void TftConsole::_redrawAll() {
  int16_t y = BANNER_H + 1;
  _tft->setTextColor(COL_TEXT, COL_BG);

  for (uint8_t i=0; i<_lineCount; i++) {
    _tft->setCursor(0, y);
    _tft->print(_lines[i]);
    y += 8;
  }
  if (_current.length()) {
    _tft->setCursor(0, y);
    _tft->print(_current);
  }
}

void TftConsole::_appendChar(char c) {
  if (c == '\r') return;
  if (c == '\n') {
    // перенос строки
    if (_lineCount < MAX_LINES) {
      _lines[_lineCount++] = _current;
    } else {
      // прокрутка вверх
      for (uint8_t i=1; i<MAX_LINES; i++) _lines[i-1] = _lines[i];
      _lines[MAX_LINES-1] = _current;
    }
    _current = "";
  } else {
    _current += c;
  }
}

void TftConsole::println(const String& s) {
  for (size_t i=0; i<s.length(); ++i) _appendChar(s[i]);
  _appendChar('\n');
  _redrawAll();
}

void TftConsole::updateFromSerial(HardwareSerial &ser) {
  bool changed = false;
  while (ser.available()) {
    char c = (char)ser.read();
    _appendChar(c);
    changed = true;
  }
  if (changed) _redrawAll();
}
