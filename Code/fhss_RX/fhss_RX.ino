/*
 * FHSS Приемник для ESP32-S3 с nRF24L01
 * Принимает и декодирует сообщение "Hello World" с FHSS
 * Автор: AI Assistant
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Подключение nRF24L01 к ESP32-S3
#define CE_PIN    7   // Можно использовать любой GPIO
#define CSN_PIN   6   // Можно использовать любой GPIO
#define MOSI_PIN   10
#define MISO_PIN   9
#define SCK_PIN    8
// SPI пины для ESP32-S3:
// MOSI: 11 (GPIO11)
// MISO: 13 (GPIO13) 
// SCK:  12 (GPIO12)

// Создание объекта радио
RF24 radio(CE_PIN, CSN_PIN);

// FHSS конфигурация (должна совпадать с передатчиком)
const byte FHSS_CHANNELS[] = {5, 15, 25, 35, 45, 55, 65, 75, 85, 95};
const byte CHANNEL_COUNT = sizeof(FHSS_CHANNELS);
const unsigned long CHANNEL_TIME = 10; // 10 мс на канал

// Адреса для связи
const byte addresses[][6] = {"TX001", "RX002"};

// Структура пакета данных (должна совпадать с передатчиком)
struct DataPacket {
  byte sequenceNumber;
  byte channelIndex;
  unsigned long timestamp;
  char data[16];
  byte checksum;
};

// Переменные приемника
DataPacket receivedPacket;
unsigned long lastChannelSwitch = 0;
byte currentChannelIndex = 0;
String receivedMessage = "";
byte lastSequenceNumber = 255; // Начальное значение
unsigned long firstPacketTime = 0;
bool messageStarted = false;

// Статистика
unsigned long packetsReceived = 0;
unsigned long packetsExpected = 0;
unsigned long checksumErrors = 0;

// Буфер для сборки сообщения
struct MessageBuffer {
  String parts[20];  // Максимум 20 частей сообщения
  bool received[20]; // Флаги получения частей
  int maxParts;
};

MessageBuffer msgBuffer;

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
  
  Serial.println("FHSS Приемник ESP32-S3 + nRF24L01");
  Serial.println("Инициализация...");
	//SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);
  // Инициализация nRF24L01
  if (!radio.begin()) {
    Serial.println("ОШИБКА: nRF24L01 не отвечает!");
    while (1) delay(1000);
  }

  // Настройка радио (должна совпадать с передатчиком)
  radio.setAutoAck(false);
  radio.setRetries(0, 0);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setPayloadSize(sizeof(DataPacket));
  
  // Открываем канал для приема
  radio.openReadingPipe(1, addresses[0]);
  radio.startListening();

  // Инициализация буфера сообщения
  msgBuffer.maxParts = 20;
  for (int i = 0; i < msgBuffer.maxParts; i++) {
    msgBuffer.parts[i] = "";
    msgBuffer.received[i] = false;
  }

  Serial.println("Радио инициализировано успешно");
  Serial.print("Размер пакета: ");
  Serial.println(sizeof(DataPacket));
  Serial.println("Ожидание FHSS сигнала...");
  Serial.println("Каналы: 5,15,25,35,45,55,65,75,85,95");
  Serial.println("----------------------------------------");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Переключение каналов каждые CHANNEL_TIME мс
  if (currentTime - lastChannelSwitch >= CHANNEL_TIME) {
    currentChannelIndex = (currentChannelIndex + 1) % CHANNEL_COUNT;
    radio.setChannel(FHSS_CHANNELS[currentChannelIndex]);
    lastChannelSwitch = currentTime;
  }
  
  // Проверяем наличие данных
  if (radio.available()) {
    // Читаем пакет
    radio.read(&receivedPacket, sizeof(receivedPacket));
    packetsReceived++;
    
    // Проверяем контрольную сумму
    byte expectedChecksum = calculateChecksum(receivedPacket);
    if (expectedChecksum != receivedPacket.checksum) {
      checksumErrors++;
      Serial.print("CHECKSUM ERROR на канале ");
      Serial.print(FHSS_CHANNELS[currentChannelIndex]);
      Serial.print(" (пакет ");
      Serial.print(receivedPacket.sequenceNumber);
      Serial.println(")");
      return;
    }
    
    // Первый пакет - запоминаем время начала
    if (!messageStarted) {
      messageStarted = true;
      firstPacketTime = currentTime;
    }
    
    // Вывод информации о принятом пакете
    Serial.print("CH:");
    Serial.print(FHSS_CHANNELS[currentChannelIndex]);
    Serial.print(" SEQ:");
    Serial.print(receivedPacket.sequenceNumber);
    Serial.print(" IDX:");
    Serial.print(receivedPacket.channelIndex);
    Serial.print(" DATA:'");
    Serial.print(receivedPacket.data);
    Serial.print("' TIME:");
    Serial.print(receivedPacket.timestamp);
    Serial.print(" (");
    Serial.print(2400 + FHSS_CHANNELS[currentChannelIndex]);
    Serial.println(" МГц)");
    
    // Сохраняем часть сообщения
    String dataPart = String(receivedPacket.data);
    
    // Определяем позицию этой части в общем сообщении
    // Для простоты используем последовательный порядок
    int messagePartIndex = receivedPacket.sequenceNumber % msgBuffer.maxParts;
    
    if (!msgBuffer.received[messagePartIndex]) {
      msgBuffer.parts[messagePartIndex] = dataPart;
      msgBuffer.received[messagePartIndex] = true;
      
      // Пытаемся собрать полное сообщение
      String fullMessage = "";
      bool hasGaps = false;
      
      for (int i = 0; i < msgBuffer.maxParts; i++) {
        if (msgBuffer.received[i] && msgBuffer.parts[i].length() > 0) {
          fullMessage += msgBuffer.parts[i];
        } else if (msgBuffer.received[i] && msgBuffer.parts[i].length() == 0) {
          break; // Дошли до конца сообщения
        } else if (i > 0 && msgBuffer.received[i-1]) {
          hasGaps = true;
          break;
        }
      }
      
      // Если нашли потенциально полное сообщение
      if (fullMessage.length() > 0 && !hasGaps) {
        Serial.println("----------------------------------------");
        Serial.println("ДЕКОДИРОВАННОЕ СООБЩЕНИЕ:");
        Serial.print(">>> ");
        Serial.print(fullMessage);
        Serial.println(" <<<");
        Serial.println("----------------------------------------");
        Serial.print("Время декодирования: ");
        Serial.print(currentTime - firstPacketTime);
        Serial.println(" мс");
        Serial.print("Пакетов принято: ");
        Serial.println(packetsReceived);
        Serial.print("Ошибок контрольной суммы: ");
        Serial.println(checksumErrors);
        Serial.println("----------------------------------------");
        
        // Сброс для нового сообщения
        for (int i = 0; i < msgBuffer.maxParts; i++) {
          msgBuffer.parts[i] = "";
          msgBuffer.received[i] = false;
        }
        messageStarted = false;
        packetsReceived = 0;
        checksumErrors = 0;
      }
    }
    
    lastSequenceNumber = receivedPacket.sequenceNumber;
  }
  
  // Небольшая задержка для стабильности
  delay(1);
}