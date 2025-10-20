#include <BleGamepad.h>

// Пины для джойстиков (аналоговые входы)
#define JOYSTICK1_X_PIN 14  // Левый джойстик X
#define JOYSTICK1_Y_PIN 3 // Левый джойстик Y
#define JOYSTICK2_X_PIN 16  // Правый джойстик X
#define JOYSTICK2_Y_PIN 7 // Правый джойстик Y

// Дополнительные кнопки (опционально)
#define BUTTON1_PIN 8
#define BUTTON2_PIN 9
#define BUTTON3_PIN 10
#define BUTTON4_PIN 11

// Создание объекта геймпада
BleGamepad bleGamepad("ESP32 Flight Gamepad", "ESP32Labs", 100);

// Калибровочные значения для джойстиков
struct JoystickCalibration {
    int minVal = 0;
    int maxVal = 4095;
    int centerVal = 2048;
    int deadzone = 300;
};

JoystickCalibration joy1X, joy1Y, joy2X, joy2Y;

// Функция для применения мертвой зоны и калибровки
int16_t calibrateAxis(int rawValue, JoystickCalibration& cal) {
    // Применяем мертвую зону относительно центра
    int centered = rawValue - cal.centerVal;
    if (abs(centered) < cal.deadzone) {
        return 0; // Центральное положение
    }
    
    // Масштабируем в диапазон -32767 до 32767
    if (centered > 0) {
        return map(rawValue, cal.centerVal + cal.deadzone, cal.maxVal, 0, 32767);
    } else {
        return map(rawValue, cal.minVal, cal.centerVal - cal.deadzone, -32767, 0);
    }
}

// Функция чтения кнопок
void readButtons() {
    static bool lastButton1 = false;
    static bool lastButton2 = false;
    static bool lastButton3 = false;
    static bool lastButton4 = false;
    
    bool button1 = (digitalRead(BUTTON1_PIN) == LOW);
    bool button2 = (digitalRead(BUTTON2_PIN) == LOW);
    bool button3 = (digitalRead(BUTTON3_PIN) == LOW);
    bool button4 = (digitalRead(BUTTON4_PIN) == LOW);
    
    // Отправляем изменения состояния кнопок
    if (button1 != lastButton1) {
        if (button1) bleGamepad.press(BUTTON_1);
        else bleGamepad.release(BUTTON_1);
        lastButton1 = button1;
    }
    
    if (button2 != lastButton2) {
        if (button2) bleGamepad.press(BUTTON_2);
        else bleGamepad.release(BUTTON_2);
        lastButton2 = button2;
    }
    
    if (button3 != lastButton3) {
        if (button3) bleGamepad.press(BUTTON_3);
        else bleGamepad.release(BUTTON_3);
        lastButton3 = button3;
    }
    
    if (button4 != lastButton4) {
        if (button4) bleGamepad.press(BUTTON_4);
        else bleGamepad.release(BUTTON_4);
        lastButton4 = button4;
    }
}

// Автокалибровка джойстиков
void calibrateJoysticks() {
    Serial.println("Начинаем калибровку джойстиков...");
    Serial.println("Оставьте джойстики в центральном положении на 3 секунды");
    
    delay(3000);
    
    // Читаем центральные значения
    joy1X.centerVal = analogRead(JOYSTICK1_X_PIN);
    joy1Y.centerVal = analogRead(JOYSTICK1_Y_PIN);
    joy2X.centerVal = analogRead(JOYSTICK2_X_PIN);
    joy2Y.centerVal = analogRead(JOYSTICK2_Y_PIN);
    
    Serial.printf("Центральные значения: J1X=%d, J1Y=%d, J2X=%d, J2Y=%d\n", 
                  joy1X.centerVal, joy1Y.centerVal, joy2X.centerVal, joy2Y.centerVal);
    
    Serial.println("Теперь двигайте джойстики во все стороны в течение 5 секунд");
    
    // Инициализируем min/max центральными значениями
    joy1X.minVal = joy1X.maxVal = joy1X.centerVal;
    joy1Y.minVal = joy1Y.maxVal = joy1Y.centerVal;
    joy2X.minVal = joy2X.maxVal = joy2X.centerVal;
    joy2Y.minVal = joy2Y.maxVal = joy2Y.centerVal;
    
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
        int val;
        
        val = analogRead(JOYSTICK1_X_PIN);
        joy1X.minVal = min(joy1X.minVal, val);
        joy1X.maxVal = max(joy1X.maxVal, val);
        
        val = analogRead(JOYSTICK1_Y_PIN);
        joy1Y.minVal = min(joy1Y.minVal, val);
        joy1Y.maxVal = max(joy1Y.maxVal, val);
        
        val = analogRead(JOYSTICK2_X_PIN);
        joy2X.minVal = min(joy2X.minVal, val);
        joy2X.maxVal = max(joy2X.maxVal, val);
        
        val = analogRead(JOYSTICK2_Y_PIN);
        joy2Y.minVal = min(joy2Y.minVal, val);
        joy2Y.maxVal = max(joy2Y.maxVal, val);
        
        delay(10);
    }
    
    Serial.println("Калибровка завершена!");
    Serial.printf("J1X: min=%d, center=%d, max=%d\n", joy1X.minVal, joy1X.centerVal, joy1X.maxVal);
    Serial.printf("J1Y: min=%d, center=%d, max=%d\n", joy1Y.minVal, joy1Y.centerVal, joy1Y.maxVal);
    Serial.printf("J2X: min=%d, center=%d, max=%d\n", joy2X.minVal, joy2X.centerVal, joy2X.maxVal);
    Serial.printf("J2Y: min=%d, center=%d, max=%d\n", joy2Y.minVal, joy2Y.centerVal, joy2Y.maxVal);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Запуск ESP32-S3 Геймпада для авиасимуляторов");

    // Настройка пинов кнопок
    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);
    pinMode(BUTTON3_PIN, INPUT_PULLUP);
    pinMode(BUTTON4_PIN, INPUT_PULLUP);

    // Калибровка джойстиков
    calibrateJoysticks();

    // Настройка конфигурации геймпада
    BleGamepadConfiguration bleGamepadConfig;
    bleGamepadConfig.setAutoReport(false);  // Отключаем автоотправку для лучшего контроля
    bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD);
    bleGamepadConfig.setVid(0x1234);  // Vendor ID
    bleGamepadConfig.setPid(0x5678);  // Product ID
    bleGamepadConfig.setButtonCount(16);
    bleGamepadConfig.setHatSwitchCount(0);
    bleGamepadConfig.setIncludeXAxis(true);
    bleGamepadConfig.setIncludeYAxis(true);
    bleGamepadConfig.setIncludeZAxis(true);
    bleGamepadConfig.setIncludeRzAxis(true);
    bleGamepadConfig.setIncludeRxAxis(false);
    bleGamepadConfig.setIncludeRyAxis(false);
    bleGamepadConfig.setIncludeSlider1(false);
    bleGamepadConfig.setIncludeSlider2(false);
    bleGamepadConfig.setIncludeRudder(false);
    bleGamepadConfig.setIncludeThrottle(false);
    bleGamepadConfig.setIncludeAccelerator(false);
    bleGamepadConfig.setIncludeBrake(false);
    bleGamepadConfig.setIncludeSteering(false);

    // Инициализация геймпада с конфигурацией
    bleGamepad.begin(&bleGamepadConfig);
    
    Serial.println("Геймпад готов к подключению!");
    Serial.println("Ищите устройство 'ESP32 Flight Gamepad' в настройках Bluetooth");
}

void loop() {
    if (bleGamepad.isConnected()) {
        // Чтение и отправка данных джойстиков
        int raw1X = analogRead(JOYSTICK1_X_PIN);
        int raw1Y = analogRead(JOYSTICK1_Y_PIN);
        int raw2X = analogRead(JOYSTICK2_X_PIN);
        int raw2Y = analogRead(JOYSTICK2_Y_PIN);

        // Калибровка и установка значений осей
        int16_t leftX = calibrateAxis(raw1X, joy1X);
        int16_t leftY = calibrateAxis(raw1Y, joy1Y);
        int16_t rightX = calibrateAxis(raw2X, joy2X);
        int16_t rightY = calibrateAxis(raw2Y, joy2Y);

        // Установка значений осей
        bleGamepad.setX(leftX);
        bleGamepad.setY(leftY);
        bleGamepad.setZ(rightX);
        bleGamepad.setRZ(rightY);

        // Чтение кнопок
        readButtons();

        // Отправка всех изменений одним пакетом
        bleGamepad.sendReport();

        // Вывод отладочной информации каждые 500мс
        static unsigned long lastDebug = 0;
        if (millis() - lastDebug > 500) {
            Serial.printf("Подключен | J1: X=%6d Y=%6d | J2: X=%6d Y=%6d\n", 
                         leftX, leftY, rightX, rightY);
            lastDebug = millis();
        }
    } else {
        // Если не подключен, показываем статус каждые 2 секунды
        static unsigned long lastStatus = 0;
        if (millis() - lastStatus > 2000) {
            Serial.println("Ожидание подключения...");
            lastStatus = millis();
        }
    }

    delay(10); // Обновление ~100Hz
}

// Функция для ручной установки калибровки (если нужно)
void setManualCalibration() {
    // Пример ручной калибровки, если автокалибровка не подходит
    joy1X.minVal = 100; joy1X.centerVal = 2048; joy1X.maxVal = 4000;
    joy1Y.minVal = 100; joy1Y.centerVal = 2048; joy1Y.maxVal = 4000;
    joy2X.minVal = 100; joy2X.centerVal = 2048; joy2X.maxVal = 4000;
    joy2Y.minVal = 100; joy2Y.centerVal = 2048; joy2Y.maxVal = 4000;
}