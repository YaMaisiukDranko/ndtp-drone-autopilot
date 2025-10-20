#include "RadioSlave.h"
#include <SPI.h>

#define SPI_PORT SPI
#define CE_PIN 44
#define CS_PIN 43
#define IRQ_PIN 3
#define POWER_LEVEL 0
#define PACKET_SIZE 16
#define NUMBER_OF_SENDPACKETS 0
#define NUMBER_OF_RECEIVE_PACKETS 1
#define FRAME_RATE 50

RadioSlave radio;

void setup() {
  Serial.begin(115200);
  radio.Init(&SPI_PORT, CE_PIN, CS_PIN, IRQ_PIN, POWER_LEVEL, PACKET_SIZE, NUMBER_OF_SENDPACKETS, NUMBER_OF_RECEIVE_PACKETS, FRAME_RATE);
}

void loop() {
  radio.WaitAndSend();  // Ждём кадр (не отправляем, так как NUMBER_OF_SENDPACKETS = 0)
  radio.Receive();      // Принимаем пакеты

  ProcessReceived();    // Обрабатываем полученные данные

  // Отладка: текущий канал
  Serial.print("Channel: ");
  Serial.print(radio.GetCurrentChannel());
  Serial.print(" ");

  // Отладка: статистика раз в секунду
  if (radio.IsSecondTick()) {
    Serial.println();
    Serial.print("Slave Received: ");
    Serial.println(radio.GetRecievedPacketsPerSecond());
  }
}

void ProcessReceived() {
  if (radio.IsNewPacket(PACKET1)) {
    int16_t receivedValue = radio.GetNextPacketValue<int16_t>(PACKET1);  // Извлекаем int
    Serial.print("Received: ");
    Serial.println(receivedValue);
  }
}