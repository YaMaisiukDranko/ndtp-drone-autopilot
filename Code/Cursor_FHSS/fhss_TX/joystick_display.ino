#include <Arduino.h>
#include <LovyanGFX.hpp>
#include "joystick_display.h"

// Pins for Ucglib software SPI based on your line:

#ifndef TFT_SCLK_PIN
#define TFT_SCLK_PIN 13
#endif
#ifndef TFT_MOSI_PIN
#define TFT_MOSI_PIN 11
#endif
#ifndef TFT_DC_PIN
#define TFT_DC_PIN 9
#endif
#ifndef TFT_CS_PIN
#define TFT_CS_PIN 10
#endif
#ifndef TFT_RST_PIN
#define TFT_RST_PIN 8
#endif

class LGFX_ST7735 final : public lgfx::LGFX_Device {
	lgfx::Panel_ST7735 _panel;
	lgfx::Bus_SPI _bus;
	//lgfx::Light_NULL _light;
	//lgfx::Touch_NULL _touch;
public:
	LGFX_ST7735() {
		{
			auto cfg = _bus.config();
			cfg.spi_host = SPI3_HOST;
			cfg.spi_mode = 0;
			cfg.freq_write = 40000000;
			cfg.freq_read  = 16000000;
			cfg.spi_3wire = false;
			cfg.use_lock = true;
			cfg.dma_channel = 1;
			cfg.pin_sclk = TFT_SCLK_PIN;
			cfg.pin_mosi = TFT_MOSI_PIN;
			cfg.pin_miso = -1;
			cfg.pin_dc   = TFT_DC_PIN;
			_bus.config(cfg);
			_panel.setBus(&_bus);
		}
		{
			auto cfg = _panel.config();
			cfg.pin_cs = TFT_CS_PIN;
			cfg.pin_rst = TFT_RST_PIN;
			cfg.pin_busy = -1;
			cfg.panel_width  = 128;
			cfg.panel_height = 160;
			cfg.offset_x = 0;
			cfg.offset_y = 0;
			cfg.offset_rotation = 0;
			cfg.dummy_read_pixel = 8;
			cfg.dummy_read_bits  = 1;
			cfg.readable   = false;
			cfg.invert     = false;
			cfg.rgb_order  = false;
			cfg.memory_width  = 128;
			cfg.memory_height = 160;
			_panel.config(cfg);
		}
		setPanel(&_panel);
	}
};

static LGFX_ST7735 gfx;
static lgfx::LGFX_Sprite canvas(&gfx);
static bool tftReady = false;

// Colors RGB565
#ifndef TFT_BLACK
#define TFT_BLACK 0x0000
#endif
#ifndef TFT_WHITE
#define TFT_WHITE 0xFFFF
#endif
#ifndef TFT_CYAN
#define TFT_CYAN 0x07FF
#endif
#ifndef TFT_YELLOW
#define TFT_YELLOW 0xFFE0
#endif

static inline float clamp01(float v) {
	if (v < -1.0f) return -1.0f;
	if (v > 1.0f) return 1.0f;
	return v;
}

bool joystickDisplayBegin() {
	if (!gfx.init()) return false;
	gfx.setColorDepth(16);
	gfx.setRotation(1);
	gfx.fillScreen(TFT_BLACK);
	canvas.setColorDepth(16);
	canvas.createSprite(160, 128);
	canvas.fillScreen(TFT_BLACK);
	tftReady = true;
	return true;
}

void joystickDisplayShowCalibration(const char* messageLine1, const char* messageLine2, uint16_t ms) {
	if (!tftReady) return;
	canvas.fillScreen(TFT_BLACK);
	canvas.setTextColor(TFT_WHITE, TFT_BLACK);
	canvas.setFont(&fonts::Font4);
	canvas.setCursor(8, 22);
	canvas.print("Калибровка");
	canvas.setFont(&fonts::Font2);
	if (messageLine1 && *messageLine1) {
		canvas.setCursor(8, 56);
		canvas.print(messageLine1);
	}
	if (messageLine2 && *messageLine2) {
		canvas.setCursor(8, 72);
		canvas.print(messageLine2);
	}
	canvas.pushSprite(0, 0);
	if (ms > 0) delay(ms);
}

void joystickDisplayRender(float leftX, float leftY, float rightX, float rightY) {
	if (!tftReady) return;

	leftX = clamp01(leftX);
	leftY = clamp01(leftY);
	rightX = clamp01(rightX);
	rightY = clamp01(rightY);

	const int16_t screenW = 160;
	const int16_t screenH = 128;
	const int16_t padding = 6;
	const int16_t radius = (screenH / 2) - padding;
	const int16_t centerY = screenH / 2;
	const int16_t centerLeftX = (screenW / 4);
	const int16_t centerRightX = (3 * screenW) / 4;

	canvas.fillScreen(TFT_BLACK);
	canvas.drawCircle(centerLeftX, centerY, radius, TFT_WHITE);
	canvas.drawCircle(centerRightX, centerY, radius, TFT_WHITE);
	canvas.drawLine(centerLeftX - radius, centerY, centerLeftX + radius, centerY, TFT_CYAN);
	canvas.drawLine(centerLeftX, centerY - radius, centerLeftX, centerY + radius, TFT_CYAN);
	canvas.drawLine(centerRightX - radius, centerY, centerRightX + radius, centerY, TFT_CYAN);
	canvas.drawLine(centerRightX, centerY - radius, centerRightX, centerY + radius, TFT_CYAN);

	auto mapToCircle = [radius](float nx, float ny) {
		int16_t dx = (int16_t)roundf(nx * radius);
		int16_t dy = (int16_t)roundf(-ny * radius);
		return std::pair<int16_t,int16_t>(dx, dy);
	};

	auto l = mapToCircle(leftX, leftY);
	canvas.fillCircle(centerLeftX + l.first, centerY + l.second, 3, TFT_YELLOW);

	auto r = mapToCircle(rightX, rightY);
	canvas.fillCircle(centerRightX + r.first, centerY + r.second, 3, TFT_YELLOW);

	canvas.setTextColor(TFT_WHITE, TFT_BLACK);
	canvas.setFont(&fonts::Font2);
	canvas.setCursor(8, 6);
	canvas.print("LEFT");
	canvas.setCursor(screenW - 40, 6);
	canvas.print("RIGHT");

	canvas.pushSprite(0, 0);
}


