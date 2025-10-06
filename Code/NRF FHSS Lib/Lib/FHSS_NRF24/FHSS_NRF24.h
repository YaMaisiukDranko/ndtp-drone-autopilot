// FHSS_NRF24.h - Library for FHSS with NRF24L01 on ESP32

#ifndef FHSS_NRF24_H
#define FHSS_NRF24_H

#include <RF24.h>
#include <Arduino.h>

#define FIXED_CHANNEL 76  // Fixed channel for initial synchronization (0-125)
#define HOP_INTERVAL_MS 10  // Hop every 10ms
#define MAX_CHANNELS 126  // NRF24L01 channels: 0-125
#define SEQUENCE_LENGTH 100  // Length of hopping sequence
#define KEY_LENGTH 16  // Length of random key (seed as bytes)

class FHSS_NRF24 {
public:
    FHSS_NRF24(RF24& radio, uint8_t ce_pin, uint8_t csn_pin, bool is_master);
    void begin();
    bool synchronize();
    bool sendData(const void* data, size_t len);
    bool receiveData(void* data, size_t& len);
    void setPipeAddresses(const uint64_t read_pipe, const uint64_t write_pipe);

private:
    RF24& _radio;
    uint8_t _ce_pin;
    uint8_t _csn_pin;
    bool _is_master;
    uint64_t _read_pipe;
    uint64_t _write_pipe;
    uint8_t _hop_sequence[SEQUENCE_LENGTH];
    uint8_t _current_hop_index;
    unsigned long _last_hop_time;
    uint8_t _key[KEY_LENGTH];

    void generateKey();
    void generateHopSequence();
    void hopChannel();
    void switchToTX();
    void switchToRX();
};

#endif