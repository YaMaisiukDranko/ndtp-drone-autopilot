// ESP32-S3 + nRF24L01 FHSS Transmitter (TX side)
// - Uses SPI and RF24 library
// - Performs initial sync on a fixed channel, then hops over a pseudo-random list of 10 channels
// - Sends control data read from Serial to the aircraft
// - Receives telemetry back via ACK payloads and prints to Serial

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

// ====== Pin configuration (adjust to your wiring) ======
// ESP32-S3 hardware SPI pins can be configured. Keep MOSI/MISO/SCK on hardware SPI pins for best performance.
// Define CE and CSN for the nRF24L01.
#ifndef NRF24_CE_PIN
#define NRF24_CE_PIN 4    // CHANGE to your CE pin
#endif
#ifndef NRF24_CSN_PIN
#define NRF24_CSN_PIN 5  // CHANGE to your CSN (CS) pin
#endif

#define MOSI_PIN   15
#define MISO_PIN   12
#define SCK_PIN   18 
// ====== Radio setup ======
static RF24 radio(NRF24_CE_PIN, NRF24_CSN_PIN);

// 5-byte addresses for RX/TX pipes (must match on both sides)
static const uint8_t txAddress[5] = {'T','X','A','A','A'}; // for TX writing
static const uint8_t rxAddress[5] = {'R','X','A','A','A'}; // for reading ACK payloads

// ====== FHSS configuration ======
// One fixed channel for sync, then 10 pseudo-random channels for hopping (0..125 valid for nRF24)
static const uint8_t SYNC_CHANNEL = 70;
static const uint8_t FHSS_CHANNELS[10] = { 97, 15, 62, 4, 110, 23, 81, 36, 55, 7 };
static const uint8_t NUM_CHANNELS = sizeof(FHSS_CHANNELS) / sizeof(FHSS_CHANNELS[0]);

// Slot timing and retry behavior
static const uint32_t PACKET_INTERVAL_MS = 5;     // send every 5 ms (200 Hz)
static const uint32_t MAX_NO_ACK_MS = 500;        // if no ACK telemetry for 500ms, attempt resync

// ====== Simple packet formats ======
struct ControlPacket {
    uint16_t sequence;           // increments each packet
    uint8_t channelIndex;        // current FHSS channel index (for debugging/verification)
    uint8_t payloadLength;       // number of valid bytes in payload[]
    char payload[24];            // control data from Serial (truncated to 24)
};

struct TelemetryPacket {
    uint16_t sequence;           // mirrors RX's view or its own telemetry sequence
    uint8_t payloadLength;
    char payload[24];
};

// ====== State ======
static bool isSynchronized = false;
static uint8_t currentChannelIndex = 0;
static uint16_t controlSequence = 0;
static uint32_t lastPacketMillis = 0;
static uint32_t lastAckMillis = 0;
static uint32_t lastSyncWaitOutput = 0;

// ====== Helpers ======
static void setRadioChannel(uint8_t channel)
{
    radio.setChannel(channel);
}

static void configureRadioCommon()
{
    // Data rate and power can be tuned for range/latency
    radio.setDataRate(RF24_2MBPS);
    radio.setPALevel(RF24_PA_LOW);
    radio.setCRCLength(RF24_CRC_16);
    radio.setAutoAck(true);
    radio.enableDynamicPayloads();
    radio.setRetries(3, 5); // delay, count
}

static void enterSyncMode()
{
    isSynchronized = false;
    currentChannelIndex = 0;
    setRadioChannel(SYNC_CHANNEL);
    radio.stopListening();
    radio.openWritingPipe(txAddress);
    radio.openReadingPipe(1, rxAddress);
    radio.startListening(); // enable ACK payload reception
}

static bool trySyncOnce()
{
    // Send a small sync beacon; RX should respond with ACK payload confirming sync
    struct {
        uint32_t magic;
        uint8_t version;
        uint8_t channelCount;
        uint8_t seed; // not used here; placeholder for future PRNG-based FHSS
    } syncFrame;

    syncFrame.magic = 0xA5F0C3D2;
    syncFrame.version = 1;
    syncFrame.channelCount = NUM_CHANNELS;
    syncFrame.seed = 0x42;

    radio.stopListening();
    setRadioChannel(SYNC_CHANNEL);
    delayMicroseconds(150);

    bool writeOk = radio.write(&syncFrame, sizeof(syncFrame));
    radio.startListening();

    if (writeOk) {
        // Check if RX provided an ACK payload confirming sync
        if (radio.isAckPayloadAvailable()) {
            uint8_t buf[32] = {0};
            uint8_t len = radio.getDynamicPayloadSize();
            if (len > 32) len = 32;
            radio.read(buf, len);

            // Minimal validation: expect the same magic back
            if (len >= 4 && buf[0] == 0xD2 && buf[1] == 0xC3 && buf[2] == 0xF0 && buf[3] == 0xA5) {
                isSynchronized = true;
                lastAckMillis = millis();
                Serial.println("SYNC_OK");
                return true;
            }
        }
    }
    return false;
}

static void sendControlAndReadTelemetry()
{
    // Prepare control payload from Serial, truncate to 24 bytes
    ControlPacket pkt = {};
    pkt.sequence = controlSequence++;
    pkt.channelIndex = currentChannelIndex;

    char temp[64] = {0};
    size_t readLen = 0;
    uint32_t tStart = millis();
    // Non-blocking small window to accumulate available bytes
    while (Serial.available() && readLen < sizeof(temp) - 1) {
        int c = Serial.read();
        if (c < 0) break;
        temp[readLen++] = (char)c;
        // avoid stalling the loop
        if (millis() - tStart > 2) break;
    }

    // Copy up to 24 bytes into packet
    pkt.payloadLength = (uint8_t)min<size_t>(readLen, sizeof(pkt.payload));
    if (pkt.payloadLength > 0) {
        memcpy(pkt.payload, temp, pkt.payloadLength);
    }

    // Hop channel and transmit
    setRadioChannel(FHSS_CHANNELS[currentChannelIndex]);
    radio.stopListening();
    bool ok = radio.write(&pkt, sizeof(pkt));
    radio.startListening();

    if (ok) {
        lastPacketMillis = millis();
        // Receive telemetry via ACK payload (if present)
        if (radio.isAckPayloadAvailable()) {
            TelemetryPacket telemetry = {};
            uint8_t len = radio.getDynamicPayloadSize();
            if (len > sizeof(telemetry)) len = sizeof(telemetry);
            radio.read(&telemetry, len);

            if (telemetry.payloadLength > 0 && telemetry.payloadLength <= sizeof(telemetry.payload)) {
                Serial.print("TEL:");
                Serial.write((const uint8_t*)telemetry.payload, telemetry.payloadLength);
                Serial.println();
                
                // Debug: print received payload length
                Serial.print("TEL_DEBUG_TX: len=");
                Serial.print(telemetry.payloadLength);
                Serial.print(" data='");
                for (int i = 0; i < telemetry.payloadLength; i++) {
                    Serial.print((char)telemetry.payload[i]);
                }
                Serial.println("'");
            }
            lastAckMillis = millis();
        }

        // Advance FHSS index after each attempt
        currentChannelIndex = (currentChannelIndex + 1) % NUM_CHANNELS;
    }
}

static void attemptResyncIfNeeded()
{
    if (!isSynchronized) return;
    if (millis() - lastAckMillis > MAX_NO_ACK_MS) {
        // Lost link; re-enter sync mode
        enterSyncMode();
    }
}

void setup()
{
    Serial.begin(115200);
    delay(50);

    // Initialize SPI explicitly as requested
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, NRF24_CSN_PIN);

    // Initialize radio
    if (!radio.begin()) {
        // If radio fails to init, keep trying silently
    }

    configureRadioCommon();

    // Enable ACK payloads
    radio.enableAckPayload();
    radio.openWritingPipe(txAddress);
    radio.openReadingPipe(1, rxAddress);

    enterSyncMode();
}

void loop()
{
    if (!isSynchronized) {
        // Try to sync at ~10 Hz (every 100ms)
        static uint32_t lastSyncAttempt = 0;
        if (millis() - lastSyncAttempt > 100) {
            lastSyncAttempt = millis();
            bool synced = trySyncOnce();
            if (!synced) {
                // Output SYNC_WAIT only once every 2 seconds to avoid spam
                if (millis() - lastSyncWaitOutput > 2000) {
                    Serial.println("SYNC_WAIT");
                    lastSyncWaitOutput = millis();
                }
            }
        }
        delay(1);
        return;
    }

    // Maintain FHSS and send packets on a fixed interval
    if (millis() - lastPacketMillis >= PACKET_INTERVAL_MS) {
        sendControlAndReadTelemetry();
    }

    attemptResyncIfNeeded();
    
    // Minimal delay for high frequency operation
    delayMicroseconds(100);
}


