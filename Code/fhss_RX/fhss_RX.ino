#include <SPI.h>
#include <RF24.h>
#include <string.h>  // Для strcmp

// Пины для приёмника
#define RX_CE_PIN 43
#define RX_CSN_PIN 44


// Инициализация радио
RF24 radio(RX_CE_PIN, RX_CSN_PIN);

// Адреса труб
const uint64_t txPipe = 0xF0F0F0F0E1LL;  // Адрес для приема от передатчика
const uint64_t rxPipe = 0xF0F0F0F0D2LL;  // Адрес для передачи ответов

// Сообщения
const char syncMsg[] = "SYNC";
const char customAck[] = "OK";  // Простое ответное сообщение

bool synced = false;
unsigned long startTime;

uint8_t channels[] = {80, 81, 82, 83, 84, 85, 86, 87, 88, 89};
const int CH_COUNT = sizeof(channels) / sizeof(channels[0]);

uint8_t getFHSSChannel(uint32_t seed, int step) {
  randomSeed(seed);
  for (int i = 0; i < step; i++) {
    random();
  }
  uint8_t idx = random(CH_COUNT);
  return channels[idx];
}

void setup() {
  Serial.begin(115200);
  
  // Инициализация SPI с кастомными пинами
  //SPI.begin(RX_SCK_PIN, RX_MISO_PIN, RX_MOSI_PIN, RX_CSN_PIN);
  
  if (!radio.begin()) {
    Serial.println("radio hardware is not responding!");
    while (1);
  }
  
  // Отключаем встроенный ACK
  radio.setAutoAck(false);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openReadingPipe(1, txPipe);
  radio.openWritingPipe(rxPipe);
  radio.startListening();
  
  Serial.println("Receiver started, waiting for SYNC...");
}

void loop() {
  if (!synced) {
    radio.setChannel(80);  // Фиксированный канал для синхронизации
    if (radio.available()) {
      char buf[10] = {0};
      radio.read(&buf, sizeof(syncMsg));
      if (strcmp(buf, syncMsg) == 0) {
        Serial.println("SYNC received, sending custom ACK ('OK')");
        radio.stopListening();  // Переключаемся в режим передачи
        radio.write(&customAck, sizeof(customAck));
        radio.startListening();  // Возвращаемся в режим приема
        synced = true;
        startTime = millis();
        Serial.println("Custom ACK sent, starting FHSS reception");
      }
    }
    delay(10);
  } else {
    // FHSS-прием
    unsigned long currentTime = millis();
    int step = (currentTime - startTime) / 500;
    uint8_t ch = getFHSSChannel(987654321, step);
    radio.setChannel(ch);
    
    if (radio.available()) {
      uint32_t receivedMessage = 0;
      radio.read(&receivedMessage, sizeof(receivedMessage));
      Serial.print("Received message: ");
      Serial.println(receivedMessage);
    }
    delay(10);
  }
}
