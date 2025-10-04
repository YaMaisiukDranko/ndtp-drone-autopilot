/*
 * FHSS Передатчик для ESP32-S3 с nRF24L01
 * Передает сообщение "Hello World" с переключением по 10 каналам
 * Автор: AI Assistant
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Подключение nRF24L01 к ESP32-S3
#define CE_PIN    4   // Можно использовать любой GPIO
#define CSN_PIN   5
#define MOSI_PIN   15
#define MISO_PIN   12
#define SCK_PIN   18   // Можно использовать любой GPIO
// SPI пины для ESP32-S3:
// MOSI: 11 (GPIO11)
// MISO: 13 (GPIO13) 
// SCK:  12 (GPIO12)

// Создание объекта радио
RF24 radio(CE_PIN, CSN_PIN);

// FHSS конфигурация
const byte FHSS_CHANNELS[] = {5, 15, 25, 35, 45, 55, 65, 75, 85, 95};
const byte CHANNEL_COUNT = sizeof(FHSS_CHANNELS);
const unsigned long CHANNEL_TIME = 10; // 10 мс на канал

// Адреса для связи
const byte addresses[][6] = {"TX001", "RX002"};

// Структура пакета данных
struct DataPacket {
  byte sequenceNumber;    // Номер пакета в последовательности
  byte channelIndex;      // Индекс текущего канала
  unsigned long timestamp;// Временная метка
  char data[16];         // Данные (часть сообщения)
  byte checksum;         // Контрольная сумма
};

// Переменные
DataPacket packet;
unsigned long lastChannelSwitch = 0;
byte currentChannelIndex = 0;
byte sequenceNumber = 0;
String message = "Hello World";
unsigned long messageStartTime = 0;
boolean messageComplete = false;

// Функция расчета контрольной суммы
byte calculateChecksum(const DataPacket& pkt) {
  byte sum = 0;
  sum ^= pkt.sequenceNumber;
  sum ^= pkt.channelIndex;
  sum ^= (pkt.timestamp & 0xFF);
  for (int i = 0; i < 16; i++) {
    sum ^= pkt.data[i];
  }
  return sum;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);
  Serial.println("FHSS Передатчик ESP32-S3 + nRF24L01");
  Serial.println("Инициализация...");

  // Инициализация nRF24L01
  if (!radio.begin()) {
    Serial.println("ОШИБКА: nRF24L01 не отвечает!");
    while (1) delay(1000);
  }

  // Настройка радио
  radio.setAutoAck(false);          // Отключаем автоподтверждение
  radio.setRetries(0, 0);          // Отключаем повторы
  radio.setDataRate(RF24_1MBPS);   // Скорость 1 Мбит/с
  radio.setPALevel(RF24_PA_HIGH);  // Максимальная мощность
  radio.setPayloadSize(sizeof(DataPacket));
  
  // Открываем канал для передачи
  radio.openWritingPipe(addresses[0]);
  radio.stopListening();

  Serial.println("Радио инициализировано успешно");
  Serial.print("Размер пакета: ");
  Serial.println(sizeof(DataPacket));
  Serial.println("Начинаем передачу FHSS...");
  
  messageStartTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Переключение каналов каждые CHANNEL_TIME мс
  if (currentTime - lastChannelSwitch >= CHANNEL_TIME) {
    // Переходим на следующий канал
    currentChannelIndex = (currentChannelIndex + 1) % CHANNEL_COUNT;
    radio.setChannel(FHSS_CHANNELS[currentChannelIndex]);
    lastChannelSwitch = currentTime;
    
    // Формируем пакет для передачи
    packet.sequenceNumber = sequenceNumber++;
    packet.channelIndex = currentChannelIndex;
    packet.timestamp = currentTime - messageStartTime;
    
    // Очищаем массив данных
    memset(packet.data, 0, sizeof(packet.data));
    
    // Определяем какую часть сообщения передавать
    int messageLength = message.length();
    int packetDataSize = sizeof(packet.data) - 1; // -1 для null terminator
    int totalPackets = (messageLength + packetDataSize - 1) / packetDataSize;
    
    // Рассчитываем индекс пакета в цикле передачи сообщения
    int messagePacketIndex = (sequenceNumber - 1) % totalPackets;
    int startPos = messagePacketIndex * packetDataSize;
    int endPos = min(startPos + packetDataSize, messageLength);
    
    // Копируем соответствующую часть сообщения
    for (int i = startPos; i < endPos; i++) {
      packet.data[i - startPos] = message[i];
    }
    
    // Рассчитываем контрольную сумму
    packet.checksum = calculateChecksum(packet);
    
    // Передаем пакет
    bool result = radio.write(&packet, sizeof(packet));
    
    // Вывод информации о передаче
    Serial.print("CH:");
    Serial.print(FHSS_CHANNELS[currentChannelIndex]);
    Serial.print(" SEQ:");
    Serial.print(packet.sequenceNumber);
    Serial.print(" DATA:'");
    Serial.print(packet.data);
    Serial.print("' ");
    Serial.print(result ? "OK" : "FAIL");
    Serial.print(" (");
    Serial.print(2400 + FHSS_CHANNELS[currentChannelIndex]);
    Serial.println(" МГц)");
  }
  
  // Небольшая задержка для стабильности
  delay(1);
}