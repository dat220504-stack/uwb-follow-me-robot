/*
  A2: capture RAW range for Tag 0x007D, send it to A1 using UART2.
  TX GPIO17 -> A1 RX GPIO16; common GND; 115200 baud, 8N1, 3.3 V.
  Callback only stores a sample/event. Sending runs outside DW1000Ranging.loop().
  Same UWB mode and antenna delay as the supplied original; library unchanged.
  UART dung dong chu de doc; phan gui nam trong sketch. Xem docs/UART.md.
*/
#include <SPI.h>
#include <math.h>
#include "DW1000.h"
#include "DW1000Ranging.h"
#include <stdio.h>
#include <string.h>

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
const uint16_t TAG_SHORT = 0x007D;
const uint64_t POLL_MASK = (uint64_t(1) << 40) - 1;
const uint32_t UART_TRANSFER_MS = 6;

struct CapturedSample {
    float meters = 0.0f;
    uint64_t pollStamp = 0;
    uint32_t timeMs = 0;
};
CapturedSample pendingSample;
bool samplePending = false, tagLostPending = false, tagAddedPending = false;

void newRange();
void newBlink(DW1000Device *device);
void inactiveDevice(DW1000Device *device);
void serviceUartTx();
uint8_t uartChecksum(const char *text);

void setup() {
    Serial.begin(115200);
    // No software TX queue: availableForWrite() reports hardware FIFO space.
    AnchorUart.setTxBufferSize(0);
    AnchorUart.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
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
    if (!device || device->getShortAddress() != TAG_SHORT) return;
    pendingSample.meters = device->getRange();
    pendingSample.pollStamp = uint64_t(device->timePollSent.getTimestamp()) & POLL_MASK;
    pendingSample.timeMs = millis();
    samplePending = true;
}

void newBlink(DW1000Device *device) {
    if (device && device->getShortAddress() == TAG_SHORT) tagAddedPending = true;
}

void inactiveDevice(DW1000Device *device) {
    if (device && device->getShortAddress() == TAG_SHORT) {
        tagLostPending = true;
        samplePending = false;
    }
}

uint8_t uartChecksum(const char *text) {
    uint8_t sum = 0;
    for (uint8_t i = 0; text[i] != '\0'; ++i) sum ^= uint8_t(text[i]);
    return sum;
}

void serviceUartTx() {
    if (!samplePending && !tagLostPending) return;
    // FIFO ESP32 co 128 byte. Chi gui khi trong, khong dung vong cho.
    if (AnchorUart.availableForWrite() < 128) return;
    uint64_t poll = 0;
    long rangeMm = 0;
    uint32_t age = 0;
    int valid = -1; // -1: Tag mat/khoi dong lai; 0: range loi; 1: range tot.
    if (tagLostPending) {
        tagLostPending = false;
    } else {
        age = uint32_t(millis() - pendingSample.timeMs);
        samplePending = false;
        if (age + UART_TRANSFER_MS > MAX_SAMPLE_AGE_MS) return;
        poll = pendingSample.pollStamp;
        valid = isfinite(pendingSample.meters) && pendingSample.meters > 0.0f &&
                pendingSample.meters <= MAX_VALID_RANGE_M ? 1 : 0;
        if (valid == 1) rangeMm = long(lroundf(pendingSample.meters * 1000.0f));
    }
    char message[48];
    snprintf(message, sizeof(message), "A2,%llu,%ld,%lu,%d",
             (unsigned long long)poll, rangeMm, (unsigned long)age, valid);
    char packet[64];
    snprintf(packet, sizeof(packet), "$%s*%02X\n", message, (unsigned int)uartChecksum(message));
    AnchorUart.write((const uint8_t *)packet, strlen(packet));
    if (DEBUG_LOG) Serial.print(packet);
}
