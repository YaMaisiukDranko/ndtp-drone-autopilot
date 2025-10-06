// Slave.ino - Example sketch for Slave device

#include <RF24.h>
#include "FHSS_NRF24.h"

// Define pins for ESP32S3 (adjust as needed)
#define CE_PIN 43
#define CSN_PIN 44

RF24 radio(CE_PIN, CSN_PIN);
FHSS_NRF24 fhss(radio, CE_PIN, CSN_PIN, false);  // false for slave

void setup() {
    Serial.begin(115200);
    fhss.begin();
    Serial.println("Slave starting synchronization...");
    if (fhss.synchronize()) {
        Serial.println("Synchronization successful!");
    } else {
        Serial.println("Synchronization failed!");
        while (1);  // Halt
    }
}

void loop() {
    // Convenient input method: Read from Serial
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        if (fhss.sendData(input.c_str(), input.length())) {
            Serial.println("Data sent: " + input);
        } else {
            Serial.println("Send failed!");
        }
    }

    // Check for received data
    char buf[32];
    size_t len = sizeof(buf);
    if (fhss.receiveData(buf, len)) {
        buf[len] = '\0';  // Null-terminate
        Serial.println("Received: " + String(buf));
    }

    delay(10);  // Small delay for loop
}