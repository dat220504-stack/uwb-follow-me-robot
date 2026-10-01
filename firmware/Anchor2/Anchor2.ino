/*
  A2: capture RAW range for Tag 0x007D, send it to A1 using UART2.
  TX GPIO17 -> A1 RX GPIO16; common GND; 115200 baud, 8N1, 3.3 V.
  Callback only stores a sample/event. Sending runs outside DW1000Ranging.loop().
  Same UWB mode and antenna delay as the supplied original; library unchanged.
  Use this entire sketch folder, including UwbUart.h. See docs/UART.md.
*/
#include <SPI.h>
#include <math.h>
#include <esp_random.h>
#include <soc/soc_caps.h>
#include "DW1000.h"
#include "DW1000Ranging.h"
#include "UwbUart.h"

char ANCHOR_ADD[] = "87:17:5B:D5:A9:9A:E2:9C";
constexpr uint8_t SPI_SCK = 18, SPI_MISO = 19, SPI_MOSI = 23;
constexpr uint8_t PIN_RST = 27, PIN_IRQ = 34, PIN_SS = 4;
constexpr uint16_t ANTENNA_DELAY = 16450;
constexpr int UART_RX_PIN = 16, UART_TX_PIN = 17;
constexpr uint32_t UART_BAUD = 115200;
constexpr uint32_t MAX_SAMPLE_AGE_MS = 80;
constexpr float MAX_VALID_RANGE_M = 10.0f;
constexpr bool DEBUG_LOG = false;
HardwareSerial AnchorUart(2);

struct CapturedSample {
    float meters = 0.0f;
    float rxPower = 0.0f;
    uint64_t pollStamp = 0;
    uint32_t timeMs = 0;
};
CapturedSample pendingSample;
bool samplePending = false, tagLostPending = false, tagAddedPending = false;
uint32_t bootId = 0, sequence = 0;

void newRange();
void newBlink(DW1000Device *device);
void inactiveDevice(DW1000Device *device);
void serviceUartTx();

void setup() {
    Serial.begin(115200);
    // No software TX queue: availableForWrite() reports hardware FIFO space.
    AnchorUart.setTxBufferSize(0);
    AnchorUart.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    bootId = esp_random();
    delay(1000);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
    DW1000.setAntennaDelay(ANTENNA_DELAY);
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachBlinkDevice(newBlink);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_SHORTDATA_FAST_LOWPOWER, false);
    Serial.println("A2 READY | UART TX=17 | antenna_delay=16450");
}

void loop() {
    DW1000Ranging.loop();
    // Ensure a reconnect invalidates cached A1 data before sending a new sample.
    if (tagAddedPending) { tagLostPending = true; tagAddedPending = false; }
    serviceUartTx();
}

void newRange() {
    DW1000Device *device = DW1000Ranging.getDistantDevice();
    if (!device || device->getShortAddress() != UwbLink::TAG_SHORT) return;
    pendingSample.meters = device->getRange();
    pendingSample.rxPower = device->getRXPower();
    pendingSample.pollStamp = uint64_t(device->timePollSent.getTimestamp()) & UwbLink::POLL_MASK;
    pendingSample.timeMs = millis();
    samplePending = true;
}

void newBlink(DW1000Device *device) {
    if (device && device->getShortAddress() == UwbLink::TAG_SHORT) tagAddedPending = true;
}

void inactiveDevice(DW1000Device *device) {
    if (device && device->getShortAddress() == UwbLink::TAG_SHORT) {
        tagLostPending = true;
        samplePending = false;
    }
}

void serviceUartTx() {
    if (!samplePending && !tagLostPending) return;
    // Never wait for UART space. A single pending sample keeps the newest data.
    const int txSpace = AnchorUart.availableForWrite();
    if (txSpace < UwbLink::FRAME_SIZE) return;
    UwbLink::Frame frame;
    frame.bootId = bootId;
    frame.sequence = ++sequence;
    if (tagLostPending) {
        frame.kind = UwbLink::TAG_LOST;
        tagLostPending = false;
    } else {
        const uint32_t queuedBytes = txSpace < SOC_UART_FIFO_LEN ? SOC_UART_FIFO_LEN - txSpace : 0;
        const uint32_t queueMs = (queuedBytes * 10000UL + UART_BAUD - 1) / UART_BAUD;
        const uint32_t age = uint32_t(millis() - pendingSample.timeMs) + queueMs;
        samplePending = false;
        if (age + UwbLink::WIRE_TIME_MS > MAX_SAMPLE_AGE_MS) return;
        frame.pollStamp = pendingSample.pollStamp;
        frame.ageMs = uint16_t(age);
        frame.valid = isfinite(pendingSample.meters) && pendingSample.meters > 0.0f &&
                      pendingSample.meters <= MAX_VALID_RANGE_M;
        if (frame.valid) frame.rangeMm = int32_t(lroundf(pendingSample.meters * 1000.0f));
        if (isfinite(pendingSample.rxPower) && fabsf(pendingSample.rxPower) <= 3276.0f)
            frame.rxDb10 = int16_t(lroundf(pendingSample.rxPower * 10.0f));
    }
    uint8_t bytes[UwbLink::FRAME_SIZE];
    UwbLink::encode(frame, bytes);
    AnchorUart.write(bytes, sizeof(bytes));
    if (DEBUG_LOG) {
        Serial.print("A2_UART,seq="); Serial.print(frame.sequence);
        Serial.print(",kind="); Serial.print(frame.kind);
        Serial.print(",range_mm="); Serial.print(frame.rangeMm);
        Serial.print(",valid="); Serial.println(frame.valid ? 1 : 0);
    }
}
