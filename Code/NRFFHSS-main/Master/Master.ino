#include "RadioMaster.h"
#include <SPI.h>

#define SPI_PORT SPI
#define CE_PIN 4
#define CS_PIN 5
#define POWER_LEVEL 0
#define PACKET_SIZE 16
#define NUMBER_OF_SENDPACKETS 1
#define NUMBER_OF_RECEIVE_PACKETS 0
#define FRAME_RATE 50

RadioMaster radio;

void setup() {
  Serial.begin(115200);
  radio.Init(&SPI_PORT, CE_PIN, CS_PIN, POWER_LEVEL, PACKET_SIZE, NUMBER_OF_SENDPACKETS, NUMBER_OF_RECEIVE_PACKETS, FRAME_RATE);
}

void loop() {
  radio.WaitAndSend();  // Ждём кадр и отправляем
  radio.Receive();      // Принимаем пакеты (не используется, так как NUMBER_OF_RECEIVE_PACKETS = 0)

  AddSendData();        // Добавляем данные для отправки

  // Отладка: текущий канал
  Serial.print("Channel: ");
  Serial.print(radio.GetCurrentChannel());
  Serial.print(" ");

  // Отладка: статистика раз в секунду
  if (radio.IsSecondTick()) {
    Serial.println();
    Serial.print("Master Sent: ");
    Serial.println(radio.GetRecievedPacketsPerSecond());  // Показывает отправленные пакеты
  }
}

void AddSendData() {
  static int counter = 0;  // Счётчик для отправки
  radio.AddNextPacketValue(PACKET1, counter);  // Добавляем int в PACKET1
  counter++;  // Увеличиваем для следующего кадра
  Serial.print("Sent: ");
  Serial.println(counter);
}