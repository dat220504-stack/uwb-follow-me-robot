#include <cassert>
#include <iostream>
#include "../firmware/Anchor2/Anchor2.ino"

void capture(uint64_t poll, float meters) {
    auto &device = DW1000Ranging.device;
    device.address = UwbLink::TAG_SHORT;
    device.range = meters; device.power = -70.1f;
    device.timePollSent.value = int64_t(poll);
    newRange();
}
UwbLink::Frame lastFrame() {
    UwbLink::Frame frame;
    assert(AnchorUart.tx.size() >= UwbLink::FRAME_SIZE);
    assert(UwbLink::decode(AnchorUart.tx.data() + AnchorUart.tx.size() - UwbLink::FRAME_SIZE, frame));
    return frame;
}

int main() {
    fakeNow = 100; bootId = 42;
    capture(123, 1.234f); assert(AnchorUart.tx.empty());
    serviceUartTx(); auto frame = lastFrame();
    assert(frame.valid && frame.rangeMm == 1234 && frame.rxDb10 == -701);
    assert(frame.pollStamp == 123 && frame.bootId == 42 && frame.sequence == 1);
    assert(frame.ageMs == 0);
    const size_t written = AnchorUart.tx.size(); serviceUartTx();
    assert(AnchorUart.tx.size() == written);

    AnchorUart.txSpace = 0; capture(124, 1.1f); serviceUartTx();
    assert(samplePending && AnchorUart.tx.size() == written);
    capture(125, 1.3f); fakeNow += 10;
    AnchorUart.txSpace = 32; serviceUartTx(); frame = lastFrame();
    assert(frame.pollStamp == 125 && frame.rangeMm == 1300 && frame.ageMs == 19);
    capture(126, 1.4f); fakeNow += 81; serviceUartTx();
    assert(!samplePending && lastFrame().pollStamp == 125);

    AnchorUart.txSpace = 128; capture(127, NAN); serviceUartTx(); frame = lastFrame();
    assert(!frame.valid && frame.rangeMm == 0);
    capture(128, -1); serviceUartTx(); assert(!lastFrame().valid);
    capture(129, 11); serviceUartTx(); assert(!lastFrame().valid);
    capture((uint64_t(1) << 40) | 130, 1); serviceUartTx(); assert(lastFrame().pollStamp == 130);

    capture(131, 1.1f); inactiveDevice(&DW1000Ranging.device);
    serviceUartTx(); frame = lastFrame();
    assert(frame.kind == UwbLink::TAG_LOST && !frame.valid && !samplePending);
    const size_t afterLoss = AnchorUart.tx.size();
    DW1000Ranging.device.address = 0xCAFE; newRange(); newBlink(&DW1000Ranging.device);
    inactiveDevice(&DW1000Ranging.device); serviceUartTx(); assert(AnchorUart.tx.size() == afterLoss);

    DW1000Ranging.device.address = UwbLink::TAG_SHORT;
    newBlink(&DW1000Ranging.device); capture(132, 1.2f); loop();
    assert(lastFrame().kind == UwbLink::TAG_LOST && samplePending);
    loop(); assert(lastFrame().kind == UwbLink::SAMPLE && lastFrame().pollStamp == 132);
    std::cout << "Anchor2: capture, encoding, UART backpressure, age, invalid range, loss PASS\n";
}
