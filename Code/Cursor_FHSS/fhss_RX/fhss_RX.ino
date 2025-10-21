// ESP32-S3 + nRF24L01 FHSS Receiver (RX side - on aircraft)
// - Syncs with TX on a fixed channel, then hops over a shared list of 10 channels
// - Receives control packets and prints them to Serial
// - Sends telemetry back using ACK payloads from Serial input

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "telemetry.h"

// ====== Pin configuration for ESP32-C6 Supermini ======
#ifndef NRF24_CE_PIN
#define NRF24_CE_PIN 44    // CE pin for ESP32-C6 Supermini
#endif
#ifndef NRF24_CSN_PIN
#define NRF24_CSN_PIN 43  // CSN pin for ESP32-C6 Supermini
#endif

#define MOSI_PIN   9
#define MISO_PIN   8
#define SCK_PIN    7


static RF24 radio(NRF24_CE_PIN, NRF24_CSN_PIN);

// Addressing must mirror TX
static const uint8_t txAddress[5] = {'T','X','A','A','A'}; // writing pipe for TX
static const uint8_t rxAddress[5] = {'R','X','A','A','A'}; // this device reading pipe

// ====== FHSS configuration ======
static const uint8_t SYNC_CHANNEL = 70;
static const uint8_t FHSS_CHANNELS[7] = { 88, 90, 92, 94, 96, 98, 100 };
static const uint8_t NUM_CHANNELS = sizeof(FHSS_CHANNELS) / sizeof(FHSS_CHANNELS[0]);

// Timing
static const uint32_t MAX_NO_PACKET_MS = 100; // resync if we stop receiving - faster timeout

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

// ====== Joystick data structure ======
struct JoystickData {
    int16_t x_left, y_left;   // Left joystick (Mode 1: Throttle/Yaw)
    int16_t x_right, y_right; // Right joystick (Mode 1: Pitch/Roll)
};



// ====== State ======
static bool isSynchronized = false;
static uint8_t currentChannelIndex = 0;
static uint16_t telemetrySequence = 0;
static uint32_t lastPacketMillis = 0;
static uint32_t lastControlOutput = 0;
static TelemetryData telemetryData;
static uint32_t lastTelemetryRead = 0;
static uint8_t telemetryPacketIndex = 0; // 0=accel, 1=gyro, 2=pressure
static JoystickData lastJoystickData = {0};

// ====== Motor control (PWM on GPIO 1,2,3,4) ======
static const int MOTOR_PINS[4] = {1, 2, 3, 4};

static inline uint8_t mapThrottleToPwm(int16_t raw)
{
    // Инвертируем: внизу (y_left = -1000) = 0, вверху (y_left = +1000) = 1000
    if (raw < -1000) raw = -1000;
    if (raw >  1000) raw =  1000;

    // Инверсия оси: чем выше стик, тем больше газ
    int16_t inverted = -raw; // теперь верх = -1000 → ниж = +1000

    long shifted = (long)inverted + 1000;   // диапазон 0..2000
    long pwm = (shifted * 255L) / 2000L;    // 0..255 для analogWrite

    if (pwm < 0)   pwm = 0;
    if (pwm > 255) pwm = 255;

    return (uint8_t)pwm;
}


static inline void motorControl()
{
    uint8_t pwm = mapThrottleToPwm(lastJoystickData.y_left);
    for (int i = 0; i < 4; ++i) {
        analogWrite(MOTOR_PINS[i], pwm);
    }
}



static uint32_t lastJoystickOutput = 0;

// ====== Helpers ======
static void setRadioChannel(uint8_t channel)
{
    radio.setChannel(channel);
}

// ====== Joystick functions ======
static bool parseJoystickData(const char* payload, size_t payloadLength, JoystickData* joystickData)
{
    // Expected format: "J:LX:LY:RX:RY" where values are -1000 to +1000
    if (payloadLength < 10 || payload[0] != 'J' || payload[1] != ':') {
        return false;
    }
    
    char* endPtr;
    const char* start = payload + 2; // Skip "J:"
    
    // Parse LX (Left X)
    joystickData->x_left = (int16_t)strtol(start, &endPtr, 10);
    if (*endPtr != ':') return false;
    start = endPtr + 1;
    
    // Parse LY (Left Y)
    joystickData->y_left = (int16_t)strtol(start, &endPtr, 10);
    if (*endPtr != ':') return false;
    start = endPtr + 1;
    
    // Parse RX (Right X)
    joystickData->x_right = (int16_t)strtol(start, &endPtr, 10);
    if (*endPtr != ':') return false;
    start = endPtr + 1;
    
    // Parse RY (Right Y)
    joystickData->y_right = (int16_t)strtol(start, &endPtr, 10);
    
    // Validate ranges
    if (joystickData->x_left < -1000 || joystickData->x_left > 1000 ||
        joystickData->y_left < -1000 || joystickData->y_left > 1000 ||
        joystickData->x_right < -1000 || joystickData->x_right > 1000 ||
        joystickData->y_right < -1000 || joystickData->y_right > 1000) {
        return false;
    }
    
    return true;
}

static void outputJoystickData(const JoystickData* joystickData)
{
    // Output joystick data with controlled frequency (max 50Hz)
    if (millis() - lastJoystickOutput > 20) {
        Serial.print("JOYSTICK: ");
        Serial.print("LX="); Serial.print(joystickData->x_left);
        Serial.print(" LY="); Serial.print(joystickData->y_left);
        Serial.print(" RX="); Serial.print(joystickData->x_right);
        Serial.print(" RY="); Serial.println(joystickData->y_right);
        
        // Mode 1 interpretation
        Serial.print("MODE1: Throttle="); Serial.print(joystickData->y_left);
        Serial.print(" Yaw="); Serial.print(joystickData->x_left);
        Serial.print(" Pitch="); Serial.print(joystickData->y_right);
        Serial.print(" Roll="); Serial.println(joystickData->x_right);
        
        lastJoystickOutput = millis();
    }
}

static void configureRadioCommon()
{
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

    // Read telemetry data from sensors every 20ms for faster updates
    if (millis() - lastTelemetryRead > 20) {
        readTelemetryData(&telemetryData);
        lastTelemetryRead = millis();
    }

    // Send telemetry in parts to fit in 24 bytes
    char telemetryString[32] = {0};
    
    switch (telemetryPacketIndex) {
        case 0: // Accelerometer
            snprintf(telemetryString, sizeof(telemetryString), "A:%.1f:%.1f:%.1f",
                telemetryData.accel_x, telemetryData.accel_y, telemetryData.accel_z);
            break;
        case 1: // Gyroscope
            snprintf(telemetryString, sizeof(telemetryString), "G:%.1f:%.1f:%.1f",
                telemetryData.gyro_x, telemetryData.gyro_y, telemetryData.gyro_z);
            break;
        case 2: // Pressure
            snprintf(telemetryString, sizeof(telemetryString), "P:%.0f",
                telemetryData.pressure);
            break;
    }
    
    // (debug prints removed to avoid blocking the radio loop)

    // Copy formatted data to packet payload
    tp.payloadLength = (uint8_t)min<size_t>(strlen(telemetryString), sizeof(tp.payload));
    if (tp.payloadLength > 0) {
        memcpy(tp.payload, telemetryString, tp.payloadLength);
    }
    
    // Cycle through telemetry packets
    telemetryPacketIndex = (telemetryPacketIndex + 1) % 3;

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

        // Process control payload
        if (pkt.payloadLength > 0 && pkt.payloadLength <= sizeof(pkt.payload)) {
            // Check if this is joystick data
            if (pkt.payload[0] == 'J' && pkt.payload[1] == ':') {
                JoystickData joystickData;
                if (parseJoystickData(pkt.payload, pkt.payloadLength, &joystickData)) {
                    lastJoystickData = joystickData;
                    outputJoystickData(&joystickData);
                }
            } else {
                // Print other control data with controlled frequency
                if (millis() - lastControlOutput > 10) {
                    Serial.print("CTL:");
                    Serial.write((const uint8_t*)pkt.payload, pkt.payloadLength);
                    Serial.println();
                    lastControlOutput = millis();
                }
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
    // Initialize motor output pins
    for (int i = 0; i < 4; ++i) {
        pinMode(MOTOR_PINS[i], OUTPUT);
    }

    //SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, NRF24_CSN_PIN);
    Serial.begin(115200);
    delay(50);

    // Initialize telemetry sensors
    if (!initializeTelemetrySensors()) {
        Serial.println("WARNING: Some telemetry sensors failed to initialize!");
    }

    

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
    pinMode(1, OUTPUT);
}

void loop()
{
    // Always prepare an ACK payload (telemetry) from Serial input if any
    prepareAckTelemetry();

    receiveLoop();
    attemptResyncIfNeeded();

    // Minimal delay for high responsiveness
    delayMicroseconds(100);
    motorControl();
}


