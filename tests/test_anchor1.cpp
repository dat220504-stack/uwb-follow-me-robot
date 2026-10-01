#include <cassert>
#include <iostream>
#include "../firmware/Anchor1/Anchor1.ino"

void resetTest() {
    fakeNow = 100;
    offsetCalibrated = false;
    OFFSET_A2_M = 0.0f;
    resetMeasurements();
    anchor1 = {}; anchor2 = {};
    localTagLost = localTagAdded = false;
    uartLength = 0; uartReading = false; uartLineStartMs = fakeNow;
    lastRemotePollKnown = false; lastRemotePoll = 0;
    lastUartServiceMs = fakeNow;
    discardUartBacklog = false;
    pairedCount = rejectedFrames = 0;
    targetTimedOut = false;
    voteWindowStartMs = calibrationLastStatusMs = fakeNow;
    Serial.output.clear(); AnchorUart.rx.clear();
    DW1000Ranging.device = {};
}

// Build test input independently of the firmware's checksum/parser.
std::string packet(const std::string &payload) {
    unsigned int checksum = 0;
    for (unsigned char c : payload) checksum ^= c;
    char suffix[8]; snprintf(suffix, sizeof(suffix), "*%02X\n", checksum);
    return "$" + payload + suffix;
}

std::string remote(uint64_t poll, long mm = 1100, unsigned int age = 0, int valid = 1) {
    return packet("A2," + std::to_string(poll) + "," + std::to_string(mm) + "," +
                  std::to_string(age) + "," + std::to_string(valid));
}

void local(uint64_t poll, float range = 1.2f) {
    auto &device = DW1000Ranging.device;
    device.address = TAG_SHORT;
    device.range = range; device.timePollSent.value = int64_t(poll);
    newRange();
}

void receive(const std::string &data) {
    AnchorUart.rx.insert(AnchorUart.rx.end(), data.begin(), data.end());
    while (AnchorUart.available()) serviceUart();
}

int main() {
    // Both arrival orders, exactly one use, and the Tag's 40-bit wrap.
    resetTest();
    receive(remote(POLL_MASK - 1));
    fakeNow += 10; local(POLL_MASK - 1); tryMakePair();
    assert(pairedCount == 1);
    tryMakePair(); local(POLL_MASK - 1); receive(remote(POLL_MASK - 1)); tryMakePair();
    assert(pairedCount == 1);
    fakeNow += 10; local(0); receive(remote(0)); tryMakePair();
    assert(pairedCount == 2);
    resetTest(); local(21); receive(remote(22)); tryMakePair();
    assert(pairedCount == 0);
    fakeNow += 10; local(22); tryMakePair(); assert(pairedCount == 1);

    // Local expiry, remote send age, and age accumulated after UART receipt.
    resetTest(); local(31); fakeNow += 81; lastUartServiceMs = fakeNow;
    receive(remote(31)); tryMakePair(); assert(pairedCount == 0);
    resetTest(); local(32); receive(remote(32, 1100, 78)); tryMakePair();
    assert(pairedCount == 0);
    resetTest(); receive(remote(33)); fakeNow += 75;
    local(33); tryMakePair(); assert(pairedCount == 0);
    resetTest(); local(34); receive(remote(34, 1100, 74)); tryMakePair();
    assert(pairedCount == 1);

    // Corruption, dropped bytes, unknown source, and partial-line timeout.
    resetTest(); local(40);
    auto broken = remote(40); broken[7] ^= 1;
    receive(broken); tryMakePair(); assert(pairedCount == 0 && rejectedFrames == 1);
    auto shortened = remote(40); shortened.erase(shortened.begin() + 8);
    receive(shortened + remote(40)); tryMakePair(); assert(pairedCount == 1);
    resetTest(); local(41); receive(packet("A3,41,1100,0,1")); tryMakePair();
    assert(pairedCount == 0 && rejectedFrames == 1);
    auto good = remote(41);
    receive(good.substr(0, 12)); fakeNow += 21; receive(good.substr(12));
    tryMakePair(); assert(pairedCount == 0);
    receive(good); tryMakePair(); assert(pairedCount == 1);
    resetTest(); local(42); receive("noise\n$bad" + remote(42));
    tryMakePair(); assert(pairedCount == 1);
    resetTest(); receive("$" + std::string(70, 'x') + "\n");
    assert(rejectedFrames == 1 && !uartReading);
    local(43); receive(remote(43)); tryMakePair(); assert(pairedCount == 1);

    // Valid-checksum but malformed numbers must not become fresh samples.
    for (const auto &payload : {"A2,44,1100,-1,1", "A2,44,1100,100,1",
                               "A2,1099511627776,1100,0,1", "A2,44,1100,0,2",
                               "A2,44,1100,0,1junk"}) {
        resetTest(); local(44); receive(packet(payload)); tryMakePair();
        assert(pairedCount == 0 && rejectedFrames == 1);
    }
    for (long mm : {0L, -1L, 10001L}) {
        resetTest(); local(44); receive(remote(44, mm)); tryMakePair();
        assert(pairedCount == 0);
    }
    resetTest(); local(44, NAN); receive(remote(44)); tryMakePair(); assert(pairedCount == 0);

    // Repeating a line cannot extend its original freshness deadline.
    resetTest(); receive(remote(50)); fakeNow += 70; receive(remote(50));
    assert(anchor2.timeMs == 100 && anchor2.sourceAgeMs == UART_TRANSFER_MS);
    fakeNow += 5; local(50); tryMakePair(); assert(pairedCount == 0);

    // A1 backlog is discarded after a loop stall or excessive RX accumulation.
    resetTest(); local(60); auto queued = remote(60);
    AnchorUart.rx.insert(AnchorUart.rx.end(), queued.begin(), queued.end());
    fakeNow += 81; serviceUart(); tryMakePair();
    assert(pairedCount == 0 && !anchor2.fresh && !AnchorUart.available());
    ++fakeNow; local(61); receive(remote(61)); tryMakePair(); assert(pairedCount == 1);
    resetTest(); local(62); receive(std::string(129, 'x') + remote(62));
    tryMakePair(); assert(pairedCount == 0 && !discardUartBacklog);

    // Loss/reconnect clears samples and votes, retaining the calibrated RAM offset.
    resetTest(); offsetCalibrated = true; OFFSET_A2_M = 0.1f;
    receive(remote(70)); local(70); tryMakePair(); assert(filterReady);
    local(71); receive(remote(0, 0, 0, -1));
    assert(!anchor1.fresh && !anchor2.fresh && !filterReady && offsetCalibrated);
    assert(OFFSET_A2_M == 0.1f);
    fakeNow = 600; flushDirectionVotes(fakeNow);
    assert(Serial.output.find("KHONG CO DU LIEU | valid=0") != std::string::npos);
    lastUartServiceMs = fakeNow;
    local(72); receive(remote(72)); tryMakePair(); assert(filterReady && pairedCount == 2);

    // Run the actual moved calibration, then verify offset is applied once.
    resetTest();
    for (uint32_t i = 0; i <= 60; ++i) {
        fakeNow = 100 + i * 100; lastUartServiceMs = fakeNow;
        local(1000 + i, 1.2f);
        receive(remote(1000 + i));
        tryMakePair();
    }
    assert(offsetCalibrated && fabsf(OFFSET_A2_M - 0.1f) < 0.00001f);
    assert(Serial.output.find("CALIB_OK") != std::string::npos);
    assert(filterReady && fabsf(filteredAngle) < 0.01f && directionIndex == 3);
    const uint32_t count = pairedCount;
    tryMakePair(); assert(pairedCount == count);

    resetTest();
    for (uint32_t i = 0; i <= 65; ++i) {
        fakeNow = 100 + i * 100; lastUartServiceMs = fakeNow;
        local(2000 + i, i % 2 ? 1.5f : 0.9f);
        receive(remote(2000 + i)); tryMakePair();
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
    resetTest(); offsetCalibrated = true; local(90); receive(remote(90)); tryMakePair();
    fakeNow = 601; lastUartServiceMs = fakeNow; loop();
    assert(targetTimedOut && !filterReady && lastVotedDirection == -1);
    assert(Serial.output.find("valid=0") != std::string::npos);
    std::cout << "Anchor1: pairing, text parser, freshness, calibration, geometry, votes, timeout PASS\n";
}
