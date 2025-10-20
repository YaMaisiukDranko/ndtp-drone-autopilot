// FHSS_NRF24.cpp - Implementation

#include "FHSS_NRF24.h"

FHSS_NRF24::FHSS_NRF24(RF24& radio, uint8_t ce_pin, uint8_t csn_pin, bool is_master)
    : _radio(radio), _ce_pin(ce_pin), _csn_pin(csn_pin), _is_master(is_master),
      _current_hop_index(0), _last_hop_time(0) {
    // Default pipe addresses
    _read_pipe = 0xE8E8F0F0E1LL;
    _write_pipe = 0xE8E8F0F0E2LL;
}

void FHSS_NRF24::setPipeAddresses(const uint64_t read_pipe, const uint64_t write_pipe) {
    _read_pipe = read_pipe;
    _write_pipe = write_pipe;
}

void FHSS_NRF24::begin() {
    _radio.begin(_ce_pin, _csn_pin);
    _radio.setPALevel(RF24_PA_HIGH);
    _radio.setDataRate(RF24_1MBPS);
    _radio.setAutoAck(true);
    _radio.enableDynamicPayloads();
    if (_is_master) {
        _radio.openWritingPipe(_write_pipe);
        _radio.openReadingPipe(1, _read_pipe);
    } else {
        _radio.openWritingPipe(_read_pipe);  // Swapped for slave
        _radio.openReadingPipe(1, _write_pipe);
    }
    _radio.stopListening();  // Start in TX mode initially
}

void FHSS_NRF24::generateKey() {
    randomSeed(analogRead(0));  // Seed with noise
    for (int i = 0; i < KEY_LENGTH; i++) {
        _key[i] = random(256);
    }
}

void FHSS_NRF24::generateHopSequence() {
    unsigned long seed = 0;
    for (int i = 0; i < KEY_LENGTH; i += 4) {
        seed = (seed << 8) | _key[i];
    }
    randomSeed(seed);
    for (int i = 0; i < SEQUENCE_LENGTH; i++) {
        _hop_sequence[i] = random(MAX_CHANNELS);
    }
}

void FHSS_NRF24::hopChannel() {
    if (millis() - _last_hop_time >= HOP_INTERVAL_MS) {
        _current_hop_index = (_current_hop_index + 1) % SEQUENCE_LENGTH;
        _radio.setChannel(_hop_sequence[_current_hop_index]);
        _last_hop_time = millis();
    }
}

bool FHSS_NRF24::synchronize() {
    _radio.setChannel(FIXED_CHANNEL);
    if (_is_master) {
        generateKey();
        switchToTX();
        if (!_radio.write(_key, KEY_LENGTH)) {
            return false;
        }
        generateHopSequence();
        switchToRX();
    } else {
        switchToRX();
        while (!_radio.available()) {
            delay(10);  // Wait for key
        }
        _radio.read(_key, KEY_LENGTH);
        generateHopSequence();
        switchToTX();
    }
    _radio.setChannel(_hop_sequence[0]);
    _last_hop_time = millis();
    return true;
}

bool FHSS_NRF24::sendData(const void* data, size_t len) {
    hopChannel();
    switchToTX();
    bool success = _radio.write(data, len);
    switchToRX();  // Switch back to RX after send for bidirectional
    return success;
}

bool FHSS_NRF24::receiveData(void* data, size_t& len) {
    hopChannel();
    switchToRX();
    if (_radio.available()) {
        len = _radio.getDynamicPayloadSize();
        _radio.read(data, len);
        return true;
    }
    return false;
}

void FHSS_NRF24::switchToTX() {
    _radio.stopListening();
}

void FHSS_NRF24::switchToRX() {
    _radio.startListening();
}