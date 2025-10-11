// ESP32-S3 + nRF24L01 FHSS Receiver (RX side - on aircraft)
// - Syncs with TX on a fixed channel, then hops over a shared list of 10 channels
// - Receives control packets and prints them to Serial
// - Sends telemetry back using ACK payloads from Serial input

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "telemetry.h"

// ====== Pin configuration (adjust to your wiring) ======
#ifndef NRF24_CE_PIN
#define NRF24_CE_PIN 2    // CHANGE to your CE pin
#endif
#ifndef NRF24_CSN_PIN
#define NRF24_CSN_PIN 3  // CHANGE to your CSN (CS) pin
#endif

#define MOSI_PIN   6
#define MISO_PIN   5
#define SCK_PIN   4

static RF24 radio(NRF24_CE_PIN, NRF24_CSN_PIN);

// Addressing must mirror TX
static const uint8_t txAddress[5] = {'T','X','A','A','A'}; // writing pipe for TX
static const uint8_t rxAddress[5] = {'R','X','A','A','A'}; // this device reading pipe

// ====== FHSS configuration ======
static const uint8_t SYNC_CHANNEL = 70;
static const uint8_t FHSS_CHANNELS[10] = { 97, 15, 62, 4, 110, 23, 81, 36, 55, 7 };
static const uint8_t NUM_CHANNELS = sizeof(FHSS_CHANNELS) / sizeof(FHSS_CHANNELS[0]);

// Timing
static const uint32_t MAX_NO_PACKET_MS = 600; // resync if we stop receiving

// ====== Packets ======
struct ControlPacket {
    uint16_t sequence;
    uint8_t channelIndex;
    uint8_t payloadLength;
    char payload[24];
};

struct TelemetryPacket {
    uint16_t sequence;
    uint8_t payloadLength;
    char payload[24];
};

// ====== State ======
static bool isSynchronized = false;
static uint8_t currentChannelIndex = 0;
static uint16_t telemetrySequence = 0;
static uint32_t lastPacketMillis = 0;
static uint32_t lastControlOutput = 0;
static TelemetryData telemetryData;
static uint32_t lastTelemetryRead = 0;

// ====== Helpers ======
static void setRadioChannel(uint8_t channel)
{
    radio.setChannel(channel);
}

static void configureRadioCommon()
{
    radio.setDataRate(RF24_2MBPS);
    radio.setPALevel(RF24_PA_LOW);
    radio.setCRCLength(RF24_CRC_16);
    radio.setAutoAck(true);
    radio.enableDynamicPayloads();
    //radio.setRetries(3, 5);
}

static void enterSyncMode()
{
    isSynchronized = false;
    currentChannelIndex = 0;
    setRadioChannel(SYNC_CHANNEL);
    radio.openWritingPipe(rxAddress);   // for ACK payload context
    radio.openReadingPipe(1, txAddress);
    radio.startListening();
}

static void prepareAckTelemetry()
{
    // Before sync: always preload ACK with sync magic so TX can detect it
    if (!isSynchronized) {
        const uint8_t ack[4] = {0xD2, 0xC3, 0xF0, 0xA5};
        radio.writeAckPayload(1, ack, sizeof(ack));
        return;
    }

    // After sync: read telemetry from sensors and format it
    TelemetryPacket tp = {};
    tp.sequence = telemetrySequence++;

    // Read telemetry data from sensors every 50ms to avoid overwhelming
    if (millis() - lastTelemetryRead > 50) {
        readTelemetryData(&telemetryData);
        lastTelemetryRead = millis();
    }

    // Format telemetry data as string
    char telemetryString[64] = {0};
    formatTelemetryString(&telemetryData, telemetryString, sizeof(telemetryString));

    // Copy formatted data to packet payload
    tp.payloadLength = (uint8_t)min<size_t>(strlen(telemetryString), sizeof(tp.payload));
    if (tp.payloadLength > 0) {
        memcpy(tp.payload, telemetryString, tp.payloadLength);
    }

    // Always prepare ACK payload, even if empty
    radio.writeAckPayload(1, &tp, sizeof(tp));
}

static void handleSyncFrame(const uint8_t* data, uint8_t len)
{
    if (len < 4) return;
    // Expect magic 0xA5F0C3D2 in little-endian
    if (data[0] == 0xD2 && data[1] == 0xC3 && data[2] == 0xF0 && data[3] == 0xA5) {
        // Send back the same 4 bytes as confirmation in ACK payload
        uint8_t ack[4] = {0xD2, 0xC3, 0xF0, 0xA5};
        radio.writeAckPayload(1, ack, sizeof(ack));
        isSynchronized = true;
        lastPacketMillis = millis();
        Serial.println("SYNC_OK_RX");
    }
}

static void receiveLoop()
{
    // Stay on sync channel until synchronized; then hop
    if (!isSynchronized) {
        setRadioChannel(SYNC_CHANNEL);
    } else {
        setRadioChannel(FHSS_CHANNELS[currentChannelIndex]);
    }

    // Process multiple packets in one loop to avoid missing packets
    uint8_t packetCount = 0;
    while (radio.available() && packetCount < 5) { // Limit to prevent blocking
        uint8_t len = radio.getDynamicPayloadSize();
        if (len == 0 || len > 32) { 
            radio.flush_rx(); 
            break; 
        }

        uint8_t buf[32] = {0};
        radio.read(buf, len);

        if (!isSynchronized && len >= 4) {
            handleSyncFrame(buf, len);
            packetCount++;
            continue;
        }

        if (len >= sizeof(ControlPacket)) len = sizeof(ControlPacket);
        ControlPacket pkt = {};
        memcpy(&pkt, buf, len);

        // Print control payload to Serial with controlled frequency
        if (pkt.payloadLength > 0 && pkt.payloadLength <= sizeof(pkt.payload)) {
            // Limit output frequency to avoid spam - output at most once every 25ms (40Hz max)
            if (millis() - lastControlOutput > 25) {
                Serial.print("CTL:");
                Serial.write((const uint8_t*)pkt.payload, pkt.payloadLength);
                Serial.println();
                lastControlOutput = millis();
            }
        }

        lastPacketMillis = millis();

        // After successfully handling a packet, hop to next channel (only when synced)
        if (isSynchronized) {
            currentChannelIndex = (currentChannelIndex + 1) % NUM_CHANNELS;
        }
        
        packetCount++;
    }
}

static void attemptResyncIfNeeded()
{
    if (isSynchronized && millis() - lastPacketMillis > MAX_NO_PACKET_MS) {
        enterSyncMode();
    }
}

void setup()
{
    Serial.begin(115200);
    delay(50);

    // Initialize telemetry sensors
    if (!initializeTelemetrySensors()) {
        Serial.println("WARNING: Some telemetry sensors failed to initialize!");
    }

    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, NRF24_CSN_PIN);

    if (!radio.begin()) {
        // keep going; we'll retry operations
    }

    configureRadioCommon();
    radio.enableAckPayload();
    radio.setChannel(SYNC_CHANNEL);
    radio.openWritingPipe(rxAddress);
    radio.openReadingPipe(1, txAddress);
    radio.startListening();

    enterSyncMode();
}

void loop()
{
    // Always prepare an ACK payload (telemetry) from Serial input if any
    prepareAckTelemetry();

    receiveLoop();
    attemptResyncIfNeeded();

    // Minimal delay for high responsiveness
    delayMicroseconds(100);
}


