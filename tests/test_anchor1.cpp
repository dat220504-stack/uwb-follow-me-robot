#include <cassert>
#include <iostream>
#include "../firmware/Anchor1/Anchor1.ino"

void resetTest() {
    fakeNow = 100;
    offsetCalibrated = false;
    OFFSET_A2_M = 0.0f;
    resetMeasurements();
    anchor1 = {}; anchor2 = {}; pendingLocal = {};
    localTagLost = localTagAdded = false;
    remoteSessionKnown = false;
    uartParser = {};
    lastUartServiceMs = fakeNow;
    discardUartBacklog = false;
    pairedCount = rejectedFrames = 0;
    targetTimedOut = false;
    voteWindowStartMs = calibrationLastStatusMs = fakeNow;
    Serial.output.clear(); AnchorUart.rx.clear();
    DW1000Ranging.device = {};
}

UwbLink::Frame remote(uint64_t poll, uint32_t seq = 1, int32_t mm = 1100) {
    UwbLink::Frame frame;
    frame.bootId = 42; frame.sequence = seq; frame.pollStamp = poll;
    frame.rangeMm = mm; frame.rxDb10 = -701; frame.valid = true;
    return frame;
}

void local(uint64_t poll, float range = 1.2f) {
    auto &device = DW1000Ranging.device;
    device.address = UwbLink::TAG_SHORT;
    device.range = range; device.timePollSent.value = int64_t(poll);
    newRange();
    processLocalSample();
}

std::vector<uint8_t> bytes(const UwbLink::Frame &frame) {
    std::vector<uint8_t> data(UwbLink::FRAME_SIZE);
    UwbLink::encode(frame, data.data()); return data;
}

void receive(const std::vector<uint8_t> &data) {
    AnchorUart.rx.insert(AnchorUart.rx.end(), data.begin(), data.end());
    while (AnchorUart.available()) serviceUart();
}

int main() {
    // Both arrival orders, exactly one use, and the Tag's 40-bit wrap.
    resetTest();
    receive(bytes(remote(UwbLink::POLL_MASK - 1)));
    fakeNow += 10; local(UwbLink::POLL_MASK - 1); tryMakePair();
    assert(pairedCount == 1);
    tryMakePair(); local(UwbLink::POLL_MASK - 1);
    receive(bytes(remote(UwbLink::POLL_MASK - 1, 2))); tryMakePair();
    assert(pairedCount == 1);
    fakeNow += 10; local(0); receive(bytes(remote(0, 3))); tryMakePair();
    assert(pairedCount == 2);

    resetTest(); local(21); receive(bytes(remote(22))); tryMakePair();
    assert(pairedCount == 0);
    fakeNow += 10; local(22); tryMakePair(); assert(pairedCount == 1);

    // Local expiry, remote send age, and age accumulated after UART receipt.
    resetTest(); local(31); fakeNow += 81;
    acceptRemoteFrame(remote(31), fakeNow); tryMakePair(); assert(pairedCount == 0);
    resetTest(); local(32); auto old = remote(32); old.ageMs = 78;
    acceptRemoteFrame(old, fakeNow); tryMakePair(); assert(pairedCount == 0);
    resetTest(); receive(bytes(remote(33))); fakeNow += 78;
    local(33); tryMakePair(); assert(pairedCount == 0);

    // Corruption, dropped bytes, false sync, unknown source, and partial timeout.
    resetTest(); local(40);
    auto broken = bytes(remote(40)); broken[22] ^= 1;
    receive(broken); tryMakePair();
    assert(pairedCount == 0 && uartParser.badFrames == 1);
    auto shortened = bytes(remote(40)); shortened.erase(shortened.begin() + 9);
    auto good = bytes(remote(40, 2)); shortened.insert(shortened.end(), good.begin(), good.end());
    receive(shortened); tryMakePair(); assert(pairedCount == 1);
    resetTest(); local(41); auto unknown = bytes(remote(41)); unknown[6] = 0xAA;
    UwbLink::putLE(unknown.data() + 30, UwbLink::crc16(unknown.data(), 30), 2);
    receive(unknown); tryMakePair(); assert(pairedCount == 0);
    good = bytes(remote(41)); receive({good.begin(), good.begin() + 16});
    fakeNow += 21; receive({good.begin() + 16, good.end()});
    tryMakePair(); assert(pairedCount == 0);
    receive(good); tryMakePair(); assert(pairedCount == 1);
    resetTest(); local(42); receive({0x00, 0xB5, 0xB5, 0x62, 0x00});
    receive(bytes(remote(42))); tryMakePair(); assert(pairedCount == 1);

    // Duplicate/out-of-order sequences never refresh a cached measurement.
    resetTest(); receive(bytes(remote(50, UINT32_MAX)));
    receive(bytes(remote(51, 0))); assert(anchor2.pollStamp == 51);
    fakeNow += 10; receive(bytes(remote(52, UINT32_MAX)));
    assert(anchor2.pollStamp == 51 && rejectedFrames == 1);
    receive(bytes(remote(52, 0))); assert(rejectedFrames == 2);

    // A1 backlog is discarded after a loop stall, not stamped as new data.
    resetTest(); local(60); auto queued = bytes(remote(60));
    AnchorUart.rx.insert(AnchorUart.rx.end(), queued.begin(), queued.end());
    fakeNow += 81; serviceUart(); tryMakePair();
    assert(pairedCount == 0 && !anchor2.fresh && !AnchorUart.available());
    ++fakeNow; local(61); receive(bytes(remote(61, 2))); tryMakePair();
    assert(pairedCount == 1);

    // A2 reset/loss clears cached samples and votes while preserving RAM offset.
    resetTest(); offsetCalibrated = true; OFFSET_A2_M = 0.1f;
    receive(bytes(remote(70))); local(70); tryMakePair(); assert(filterReady);
    local(71); auto reboot = remote(71); reboot.bootId = 43;
    receive(bytes(reboot)); tryMakePair();
    assert(pairedCount == 1 && !filterReady && OFFSET_A2_M == 0.1f);
    auto lost = remote(0, 2); lost.bootId = 43; lost.kind = UwbLink::TAG_LOST; lost.valid = false;
    receive(bytes(lost)); assert(!anchor1.fresh && !anchor2.fresh && offsetCalibrated);
    fakeNow = 600; flushDirectionVotes(fakeNow);
    assert(Serial.output.find("KHONG CO DU LIEU | valid=0") != std::string::npos);

    // Run the actual moved calibration, then verify offset is applied once.
    resetTest();
    for (uint32_t i = 0; i <= 60; ++i) {
        fakeNow = 100 + i * 100;
        local(1000 + i, 1.2f);
        acceptRemoteFrame(remote(1000 + i, i + 1, 1100), fakeNow);
        tryMakePair();
    }
    assert(offsetCalibrated && fabsf(OFFSET_A2_M - 0.1f) < 0.00001f);
    assert(Serial.output.find("CALIB_OK") != std::string::npos);
    assert(filterReady && fabsf(filteredAngle) < 0.01f && directionIndex == 3);
    const uint32_t count = pairedCount;
    tryMakePair(); assert(pairedCount == count);

    resetTest();
    for (uint32_t i = 0; i <= 65; ++i) {
        fakeNow = 100 + i * 100;
        local(2000 + i, i % 2 ? 1.5f : 0.9f);
        acceptRemoteFrame(remote(2000 + i, i + 1), fakeNow); tryMakePair();
    }
    assert(!offsetCalibrated && Serial.output.find("CALIB_CHUA_ON_DINH") != std::string::npos);

    // Geometry sign, rejection, vote ties, empty windows, and target timeout.
    resetTest(); offsetCalibrated = true;
    calculateGeometry(sqrtf(1.0f + 0.55f * 0.55f), sqrtf(1.0f + 0.05f * 0.05f));
    assert(directionIndex == 4 && filteredAngle > 10.0f);
    resetMeasurements();
    calculateGeometry(sqrtf(1.0f + 0.05f * 0.05f), sqrtf(1.0f + 0.55f * 0.55f));
    assert(directionIndex == 2 && filteredAngle < -10.0f);
    resetMeasurements(); calculateGeometry(1.0f, 2.0f); assert(!filterReady);
    addDirectionVote(2, 1, 1, -15, 101); addDirectionVote(4, 1, 1, 15, 102);
    flushDirectionVotes(600); assert(lastVotedDirection == 4);
    addDirectionVote(4, 1, 1, 15, 601); addDirectionVote(2, 1, 1, -15, 602);
    flushDirectionVotes(1100); assert(lastVotedDirection == 4);
    flushDirectionVotes(1600); assert(lastVotedDirection == -1);
    resetTest(); offsetCalibrated = true; local(90); acceptRemoteFrame(remote(90), fakeNow); tryMakePair();
    fakeNow = 601; lastUartServiceMs = fakeNow; loop();
    assert(targetTimedOut && !filterReady && lastVotedDirection == -1);
    assert(Serial.output.find("valid=0") != std::string::npos);
    std::cout << "Anchor1: pairing, parser, restart, calibration, geometry, votes, timeout PASS\n";
}
