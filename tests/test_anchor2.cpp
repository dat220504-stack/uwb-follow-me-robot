#include <cassert>
#include <iostream>
#include "stubs/DW1000Ranging.h"
// Arduino tu khai bao ham; C++ tren may can khai bao truoc.
void newRange();
void newBlink(DW1000Device *device);
void inactiveDevice(DW1000Device *device);
void serviceUartTx();
uint8_t uartChecksum(const char *text);
#include "../firmware/Anchor2/Anchor2.ino"

void capture(uint64_t poll, float meters) {
    auto &device = DW1000Ranging.device;
    device.address = TAG_SHORT;
    device.range = meters; device.timePollSent.value = int64_t(poll);
    newRange();
}

std::string lastLine() {
    const auto &data = AnchorUart.tx;
    assert(!data.empty() && data.back() == '\n');
    size_t start = data.size() - 1;
    while (start > 0 && data[start - 1] != '\n') --start;
    return std::string(data.begin() + start, data.end());
}

void expectPayload(const std::string &payload) {
    unsigned int checksum = 0;
    for (unsigned char c : payload) checksum ^= c;
    char suffix[8]; snprintf(suffix, sizeof(suffix), "*%02X\n", checksum);
    assert(lastLine() == "$" + payload + suffix);
}

int main() {
    fakeNow = 100;
    capture(123, 1.234f); assert(AnchorUart.tx.empty());
    serviceUartTx(); expectPayload("A2,123,1234,0,1");
    const size_t written = AnchorUart.tx.size(); serviceUartTx();
    assert(AnchorUart.tx.size() == written);

    // Callback does not write; busy FIFO keeps only the latest measurement.
    AnchorUart.txSpace = 0; capture(124, 1.1f); serviceUartTx();
    assert(samplePending && AnchorUart.tx.size() == written);
    capture(125, 1.3f); fakeNow += 10;
    AnchorUart.txSpace = 127; serviceUartTx();
    assert(samplePending && AnchorUart.tx.size() == written);
    AnchorUart.txSpace = 128; serviceUartTx(); expectPayload("A2,125,1300,10,1");
    const size_t freshWritten = AnchorUart.tx.size();
    capture(126, 1.4f); fakeNow += 75; serviceUartTx();
    assert(!samplePending && AnchorUart.tx.size() == freshWritten);
    capture(126, 1.4f); fakeNow += 74; serviceUartTx(); expectPayload("A2,126,1400,74,1");

    capture(127, NAN); serviceUartTx(); expectPayload("A2,127,0,0,0");
    capture(128, -1); serviceUartTx(); expectPayload("A2,128,0,0,0");
    capture(129, 11); serviceUartTx(); expectPayload("A2,129,0,0,0");
    capture((uint64_t(1) << 40) | 130, 1); serviceUartTx(); expectPayload("A2,130,1000,0,1");
    capture(POLL_MASK, 10); serviceUartTx(); expectPayload("A2,1099511627775,10000,0,1");
    assert(lastLine().size() < 64);

    capture(131, 1.1f); inactiveDevice(&DW1000Ranging.device);
    serviceUartTx(); expectPayload("A2,0,0,0,-1"); assert(!samplePending);
    const size_t afterLoss = AnchorUart.tx.size();
    DW1000Ranging.device.address = 0xCAFE; newRange(); newBlink(&DW1000Ranging.device);
    inactiveDevice(&DW1000Ranging.device); serviceUartTx();
    assert(AnchorUart.tx.size() == afterLoss);

    // Send reconnect notice first, then the retained fresh measurement.
    DW1000Ranging.device.address = TAG_SHORT;
    newBlink(&DW1000Ranging.device); capture(132, 1.2f); loop();
    expectPayload("A2,0,0,0,-1"); assert(samplePending);
    loop(); expectPayload("A2,132,1200,0,1");
    std::cout << "Anchor2: capture, text output, UART backpressure, age, invalid range, loss PASS\n";
}
