#include <SPI.h>
#include <RF24.h>
#include <string.h>  // Для strcmp

// Пины для передатчика
#define TX_CE_PIN 4
#define TX_CSN_PIN 5
#define TX_MOSI_PIN 15
#define TX_MISO_PIN 12
#define TX_SCK_PIN 18

// Инициализация радио
RF24 radio(TX_CE_PIN, TX_CSN_PIN);

// Адреса труб (для двунаправленного обмена)
const uint64_t txPipe = 0xF0F0F0F0E1LL;  // Адрес для передачи от передатчика
const uint64_t rxPipe = 0xF0F0F0F0D2LL;  // Адрес для приема ответов от приемника

// Сообщения
const char syncMsg[] = "SYNC";
const char customAck[] = "OK";  // Простое ответное сообщение
const uint32_t message = 102030;

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
  SPI.begin(TX_SCK_PIN, TX_MISO_PIN, TX_MOSI_PIN, TX_CSN_PIN);
  
  if (!radio.begin()) {
    Serial.println("radio hardware is not responding!");
    while (1);
  }
  
  // Отключаем встроенный ACK
  radio.setAutoAck(false);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openWritingPipe(txPipe);
  radio.openReadingPipe(1, rxPipe);
  radio.stopListening();
  
  Serial.println("Transmitter started, waiting for sync...");
}

void loop() {
  if (!synced) {
    radio.setChannel(80);  // Фиксированный канал для синхронизации
    radio.stopListening();  // Режим передачи
    bool sent = radio.write(&syncMsg, sizeof(syncMsg));
    Serial.println(sent ? "Sync message sent" : "Sync send failed");
    
    radio.startListening();  // Переключаемся в режим приема для ожидания ответа
    unsigned long timeoutStart = millis();
    bool ackReceived = false;
    while (millis() - timeoutStart < 1000) {
      if (radio.available()) {
        char buf[10] = {0};
        radio.read(&buf, sizeof(buf));
        if (strcmp(buf, customAck) == 0) {
          ackReceived = true;
          break;
        }
      }
    }
    radio.stopListening();
    
    if (ackReceived) {
      synced = true;
      startTime = millis();
      Serial.println("Custom ACK ('OK') received, starting FHSS transmission");
    } else {
      Serial.println("No custom ACK, retrying SYNC...");
      delay(500);
    }
  } else {
    // FHSS-передача
    unsigned long currentTime = millis();
    int step = (currentTime - startTime) / 500;
    uint8_t ch = getFHSSChannel(987654321, step);
    radio.setChannel(ch);
    
    radio.stopListening();
    bool ok = radio.write(&message, sizeof(message));
    radio.startListening();  // Готовимся к возможным ответам, но в FHSS не используем
    Serial.print("Sent message on channel ");
    Serial.print(ch);
    Serial.println(ok ? " SUCCESS" : " FAIL");
    delay(500);
  }
}
