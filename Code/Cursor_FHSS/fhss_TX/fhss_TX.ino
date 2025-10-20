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

// ====== Joystick configuration ======
#define X_LEFT_PIN  16
#define Y_LEFT_PIN  7
#define X_RIGHT_PIN 14
#define Y_RIGHT_PIN 3

// Joystick calibration structure
struct JoystickCalibration {
    int16_t x_left_min, x_left_max, x_left_center;
    int16_t y_left_min, y_left_max, y_left_center;
    int16_t x_right_min, x_right_max, x_right_center;
    int16_t y_right_min, y_right_max, y_right_center;
};

// Joystick data structure
struct JoystickData {
    int16_t x_left, y_left;   // Left joystick (Mode 1: Throttle/Yaw)
    int16_t x_right, y_right; // Right joystick (Mode 1: Pitch/Roll)
}; 
// ====== Radio setup ======
static RF24 radio(NRF24_CE_PIN, NRF24_CSN_PIN);

// 5-byte addresses for RX/TX pipes (must match on both sides)
static const uint8_t txAddress[5] = {'T','X','A','A','A'}; // for TX writing
static const uint8_t rxAddress[5] = {'R','X','A','A','A'}; // for reading ACK payloads

// ====== FHSS configuration ======
// One fixed channel for sync, then 10 pseudo-random channels for hopping (0..125 valid for nRF24)
static const uint8_t SYNC_CHANNEL = 70;
static const uint8_t FHSS_CHANNELS[7] = {88, 90, 92, 94, 96, 98, 100};
static const uint8_t NUM_CHANNELS = sizeof(FHSS_CHANNELS) / sizeof(FHSS_CHANNELS[0]);

// Slot timing and retry behavior
static const uint32_t PACKET_INTERVAL_MS = 2;     // send every 2 ms (500 Hz) - higher frequency
static const uint32_t MAX_NO_ACK_MS = 100;        // if no ACK telemetry for 200ms, attempt resync

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

// ====== Joystick state ======
static JoystickCalibration calibration = {0};
static JoystickData currentJoystickData = {0};
static bool calibrationComplete = false;
static uint32_t lastJoystickRead = 0;

// ====== Helpers ======
static void setRadioChannel(uint8_t channel)
{
    radio.setChannel(channel);
}

// ====== Joystick functions ======
static void performJoystickCalibration()
{
    Serial.println("=== JOYSTICK CALIBRATION ===");
    Serial.println("Center calibration - 3 seconds");
    Serial.println("Keep joysticks in center position...");
    
    // Center calibration (3 seconds)
    uint32_t startTime = millis();
    int32_t x_left_sum = 0, y_left_sum = 0, x_right_sum = 0, y_right_sum = 0;
    int32_t sampleCount = 0;
    
    while (millis() - startTime < 3000) {
        x_left_sum += analogRead(X_LEFT_PIN);
        y_left_sum += analogRead(Y_LEFT_PIN);
        x_right_sum += analogRead(X_RIGHT_PIN);
        y_right_sum += analogRead(Y_RIGHT_PIN);
        sampleCount++;
        delay(10);
    }
    
    calibration.x_left_center = x_left_sum / sampleCount;
    calibration.y_left_center = y_left_sum / sampleCount;
    calibration.x_right_center = x_right_sum / sampleCount;
    calibration.y_right_center = y_right_sum / sampleCount;
    
    Serial.println("Center calibration complete!");
    Serial.print("Left center: X="); Serial.print(calibration.x_left_center);
    Serial.print(" Y="); Serial.println(calibration.y_left_center);
    Serial.print("Right center: X="); Serial.print(calibration.x_right_center);
    Serial.print(" Y="); Serial.println(calibration.y_right_center);
    
    Serial.println("Range calibration - 5 seconds");
    Serial.println("Move joysticks to all extreme positions...");
    
    // Range calibration (5 seconds)
    startTime = millis();
    calibration.x_left_min = calibration.x_left_max = calibration.x_left_center;
    calibration.y_left_min = calibration.y_left_max = calibration.y_left_center;
    calibration.x_right_min = calibration.x_right_max = calibration.x_right_center;
    calibration.y_right_min = calibration.y_right_max = calibration.y_right_center;
    
    while (millis() - startTime < 5000) {
        int16_t x_left = analogRead(X_LEFT_PIN);
        int16_t y_left = analogRead(Y_LEFT_PIN);
        int16_t x_right = analogRead(X_RIGHT_PIN);
        int16_t y_right = analogRead(Y_RIGHT_PIN);
        
        if (x_left < calibration.x_left_min) calibration.x_left_min = x_left;
        if (x_left > calibration.x_left_max) calibration.x_left_max = x_left;
        if (y_left < calibration.y_left_min) calibration.y_left_min = y_left;
        if (y_left > calibration.y_left_max) calibration.y_left_max = y_left;
        if (x_right < calibration.x_right_min) calibration.x_right_min = x_right;
        if (x_right > calibration.x_right_max) calibration.x_right_max = x_right;
        if (y_right < calibration.y_right_min) calibration.y_right_min = y_right;
        if (y_right > calibration.y_right_max) calibration.y_right_max = y_right;
        
        delay(10);
    }
    
    Serial.println("Range calibration complete!");
    Serial.print("Left range: X="); Serial.print(calibration.x_left_min);
    Serial.print("-"); Serial.print(calibration.x_left_max);
    Serial.print(" Y="); Serial.print(calibration.y_left_min);
    Serial.print("-"); Serial.println(calibration.y_left_max);
    Serial.print("Right range: X="); Serial.print(calibration.x_right_min);
    Serial.print("-"); Serial.print(calibration.x_right_max);
    Serial.print(" Y="); Serial.print(calibration.y_right_min);
    Serial.print("-"); Serial.println(calibration.y_right_max);
    
    calibrationComplete = true;
    Serial.println("=== CALIBRATION COMPLETE ===");
}

static void readJoystickData()
{
    if (!calibrationComplete) return;
    
    // Read raw values
    int16_t x_left_raw = analogRead(X_LEFT_PIN);
    int16_t y_left_raw = analogRead(Y_LEFT_PIN);
    int16_t x_right_raw = analogRead(X_RIGHT_PIN);
    int16_t y_right_raw = analogRead(Y_RIGHT_PIN);
    
    // Convert to normalized values (-1000 to +1000)
    currentJoystickData.x_left = map(x_left_raw, calibration.x_left_min, calibration.x_left_max, -1000, 1000);
    currentJoystickData.y_left = map(y_left_raw, calibration.y_left_min, calibration.y_left_max, -1000, 1000);
    currentJoystickData.x_right = map(x_right_raw, calibration.x_right_min, calibration.x_right_max, -1000, 1000);
    currentJoystickData.y_right = map(y_right_raw, calibration.y_right_min, calibration.y_right_max, -1000, 1000);
    
    // Clamp values to ensure they stay within bounds
    currentJoystickData.x_left = constrain(currentJoystickData.x_left, -1000, 1000);
    currentJoystickData.y_left = constrain(currentJoystickData.y_left, -1000, 1000);
    currentJoystickData.x_right = constrain(currentJoystickData.x_right, -1000, 1000);
    currentJoystickData.y_right = constrain(currentJoystickData.y_right, -1000, 1000);
}

static void formatJoystickData(char* buffer, size_t bufferSize)
{
    // Format: "J:LX:LY:RX:RY" where values are -1000 to +1000
    snprintf(buffer, bufferSize, "J:%d:%d:%d:%d", 
        currentJoystickData.x_left, currentJoystickData.y_left,
        currentJoystickData.x_right, currentJoystickData.y_right);
}

static void configureRadioCommon()
{
    // Data rate and power can be tuned for range/latency
    radio.setDataRate(RF24_2MBPS);  // Maximum speed for nRF24L01
    radio.setPALevel(RF24_PA_LOW);  // Maximum power for better reliability
    radio.setCRCLength(RF24_CRC_8); // Faster CRC for better performance
    radio.setAutoAck(true);
    radio.enableDynamicPayloads();
    radio.setRetries(1, 3); // Reduced retries for faster transmission
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
    // Prepare control payload from joystick data
    ControlPacket pkt = {};
    pkt.sequence = controlSequence++;
    pkt.channelIndex = currentChannelIndex;

    // Read joystick data every 20ms for smooth control
    if (millis() - lastJoystickRead > 20) {
        readJoystickData();
        lastJoystickRead = millis();
    }

    // Format joystick data for transmission
    char joystickBuffer[32] = {0};
    formatJoystickData(joystickBuffer, sizeof(joystickBuffer));

    // Copy up to 24 bytes into packet
    pkt.payloadLength = (uint8_t)min<size_t>(strlen(joystickBuffer), sizeof(pkt.payload));
    if (pkt.payloadLength > 0) {
        memcpy(pkt.payload, joystickBuffer, pkt.payloadLength);
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
                
                // (debug prints removed to reduce Serial overhead)
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

    // Initialize joystick pins
    pinMode(X_LEFT_PIN, INPUT);
    pinMode(Y_LEFT_PIN, INPUT);
    pinMode(X_RIGHT_PIN, INPUT);
    pinMode(Y_RIGHT_PIN, INPUT);

    // Perform joystick calibration
    performJoystickCalibration();

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
        // Try to sync at ~20 Hz (every 50ms) - faster sync attempts
        static uint32_t lastSyncAttempt = 0;
        if (millis() - lastSyncAttempt > 50) {
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


