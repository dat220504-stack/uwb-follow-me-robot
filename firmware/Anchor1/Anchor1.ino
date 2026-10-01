/*
  A1: RAW d1 + UART d2 -> auto OFFSET A2 -> geometry -> Kalman -> vote 500 ms.
  Tag still owns A2 -> A1 UWB order. No reply-slot validation here.
  Pairs require the SAME 40-bit Tag POLL timestamp and age <= 80 ms.
  Each callback only captures a sample/event; UART and calculations run in loop().
  At A1 startup/reset, keep Tag stationary in front, centered, about 1 m away.
  Wait for CALIB_OK. Offset stays in RAM across Tag/A2 reconnects.
  Use this entire sketch folder, including UwbUart.h. See docs/UART.md.
*/
#include <SPI.h>
#include <math.h>
#include "DW1000.h"
#include "DW1000Ranging.h"
#include "UwbUart.h"

char ANCHOR_ADD[] = "86:17:5B:D5:A9:9A:E2:9C";
constexpr uint8_t SPI_SCK = 18, SPI_MISO = 19, SPI_MOSI = 23;
constexpr uint8_t PIN_RST = 27, PIN_IRQ = 34, PIN_SS = 4;
constexpr uint16_t ANTENNA_DELAY = 16461;
constexpr int UART_RX_PIN = 16, UART_TX_PIN = 17;
constexpr uint32_t UART_BAUD = 115200;
constexpr uint32_t TARGET_TIMEOUT_MS = 500;
constexpr uint16_t UART_BYTE_BUDGET = 64;
const float ANCHOR_SPACING_M = 0.5f;
HardwareSerial AnchorUart(2);

const uint32_t MAX_PAIR_SKEW_MS = 80;
float OFFSET_A2_M = 0.0f; // Tu tinh mot lan moi khi bat/reset,
const uint32_t CALIB_SETTLE_MS = 2000;
const uint32_t CALIB_MIN_COLLECT_MS = 3000;
const uint16_t CALIB_MIN_PAIRS = 30;
const uint32_t CALIB_MAX_PAIR_GAP_MS = 1500;
const uint32_t CALIB_TIMEOUT_MS = 60000; // Cho phep thu du mau khi tan so ranging thap.
const float CALIB_MAX_STD_M = 0.10f; // Nguong thu nghiem: do lech chuan moi range <= 10 cm.
// Range vo ly se bi bo.
const float MAX_VALID_RANGE_M = 10.0f;
const bool DEBUG_LOG = false; // true: them RAW / PAIR / UART; false: chi ket qua vote.
const bool PRINT_DETAILS = true; // false: chi in THANG / TRAI NHE / ...
const uint32_t VOTE_WINDOW_MS = 500;
const uint8_t DIRECTION_COUNT = 7;
const char *const DIRECTION_NAMES[DIRECTION_COUNT] = {
    "BEN TRAI", "TRAI CHEO", "TRAI NHE", "THANG",
    "PHAI NHE", "PHAI CHEO", "BEN PHAI"
};

// Kalman loc GOC, khong thay doi range hay antenna delay.
// Q: do bat dinh tang moi giay (do^2/s); R: nhieu do goc (do^2).
const float KALMAN_Q = 36.0f;   // Tang: bam nhanh hon, rung nhieu hon.
const float KALMAN_R = 25.0f;   // Tang: muot hon, tre hon.
const uint32_t FILTER_RESET_MS = 800;


struct RangeSample {
    float meters = 0.0f;
    float rxPower = 0.0f;
    uint64_t pollStamp = 0;
    uint32_t timeMs = 0; // Always A1's clock, for local callback or UART receipt.
    uint32_t sourceAgeMs = 0;
    bool fresh = false;
};
RangeSample anchor1, anchor2, pendingLocal;
bool localPending = false, localTagLost = false, localTagAdded = false;
bool remoteSessionKnown = false;
uint32_t remoteBootId = 0, remoteSequence = 0;
UwbLink::Parser uartParser;
uint32_t lastUartServiceMs = 0;
bool discardUartBacklog = true;
bool lastPairKnown = false, lastConsumedPollKnown = false;
uint64_t lastConsumedPoll = 0;
uint32_t lastPairMs = 0;
bool targetTimedOut = false;
uint32_t pairedCount = 0, rejectedFrames = 0;

bool filterReady = false;
float filteredAngle = 0.0f;
float angleVariance = 0.0f;
uint32_t lastAngleMs = 0;
int directionIndex = 3; // 0..6, THANG la 3.

struct DirectionVote
{
    uint32_t count;
    uint32_t lastOrder;
    float d1;
    float d2;
    float angleDeg;
};

DirectionVote directionVotes[DIRECTION_COUNT] = {};
uint32_t voteWindowStartMs = 0;
uint32_t voteSequence = 0;
int lastVotedDirection = -1;

bool offsetCalibrated = false;
bool calibrationStarted = false;
bool calibrationCollecting = false;
uint32_t calibrationStartMs = 0;
uint32_t calibrationCollectStartMs = 0;
uint32_t calibrationLastPairMs = 0;
uint32_t calibrationLastStatusMs = 0;
uint32_t calibrationPairs = 0;
float calibrationMeanD1 = 0.0f;
float calibrationMeanD2 = 0.0f;
float calibrationM2D1 = 0.0f;
float calibrationM2D2 = 0.0f;


void newRange();
void newBlink(DW1000Device *device);
void inactiveDevice(DW1000Device *device);
void clearFreshPair();
void resetMeasurements();
void processLocalSample();
void serviceUart();
void acceptRemoteFrame(const UwbLink::Frame &frame, uint32_t now);
void tryMakePair();
void dropOldSingleSample();
bool hasRecentAnchorData(uint32_t now);
void calculateGeometry(float d1, float d2);
void updateFilteredDirection(float angleDeg, float d1, float d2);
void clearDirectionVotes();
void flushDirectionVotes(uint32_t now);
void addDirectionVote(int direction, float d1, float d2, float angleDeg, uint32_t now);
void resetStartupCalibration();
void collectCalibrationPair(float d1Raw, float d2Raw, uint32_t now);
void serviceStartupCalibration(uint32_t now);

void setup() {
    Serial.begin(115200);
    AnchorUart.setRxBufferSize(256);
    AnchorUart.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    delay(1000);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
    DW1000.setAntennaDelay(ANTENNA_DELAY);
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachBlinkDevice(newBlink);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_SHORTDATA_FAST_LOWPOWER, false);
    voteWindowStartMs = millis();
    calibrationLastStatusMs = voteWindowStartMs;
    lastUartServiceMs = voteWindowStartMs;
    Serial.println("A1 READY | UART RX=16 | antenna_delay=16461");
    Serial.println("CALIB: DAT TAG DUNG YEN GIUA A1/A2, PHIA TRUOC CACH TAM 1 m.");
    Serial.println("CALIB: CHO CALIB_OK ROI MOI DI CHUYEN.");
}

void loop() {
    DW1000Ranging.loop();
    if (localTagLost || localTagAdded) {
        resetMeasurements();
        localTagLost = localTagAdded = false;
    }
    processLocalSample();
    serviceUart();
    dropOldSingleSample();
    tryMakePair();
    const uint32_t now = millis();
    if (offsetCalibrated && lastPairKnown && !targetTimedOut && uint32_t(now - lastPairMs) > TARGET_TIMEOUT_MS) {
        resetMeasurements();
        targetTimedOut = true;
    }
    if (!offsetCalibrated) serviceStartupCalibration(now);
    else flushDirectionVotes(now);
}

void newRange() {
    DW1000Device *device = DW1000Ranging.getDistantDevice();
    if (!device || device->getShortAddress() != UwbLink::TAG_SHORT) return;
    pendingLocal.meters = device->getRange();
    pendingLocal.rxPower = device->getRXPower();
    pendingLocal.pollStamp = uint64_t(device->timePollSent.getTimestamp()) & UwbLink::POLL_MASK;
    pendingLocal.timeMs = millis();
    pendingLocal.sourceAgeMs = 0;
    pendingLocal.fresh = true;
    localPending = true;
}

void newBlink(DW1000Device *device) {
    if (device && device->getShortAddress() == UwbLink::TAG_SHORT) localTagAdded = true;
}

void inactiveDevice(DW1000Device *device) {
    if (device && device->getShortAddress() == UwbLink::TAG_SHORT) localTagLost = true;
}

void processLocalSample() {
    if (!localPending) return;
    localPending = false;
    if (!isfinite(pendingLocal.meters) || pendingLocal.meters <= 0.0f ||
        pendingLocal.meters > MAX_VALID_RANGE_M) {
        clearFreshPair();
        return;
    }
    if (lastConsumedPollKnown && pendingLocal.pollStamp == lastConsumedPoll) return;
    anchor1 = pendingLocal;
}

void serviceUart() {
    const uint32_t now = millis();
    // Bytes already buffered during a long A1 stall cannot be assigned a new age.
    if (uint32_t(now - lastUartServiceMs) > MAX_PAIR_SKEW_MS ||
        AnchorUart.available() > 4 * UwbLink::FRAME_SIZE) {
        discardUartBacklog = true;
        uartParser.reset();
        resetMeasurements();
    }
    lastUartServiceMs = now;
    for (uint16_t i = 0; i < UART_BYTE_BUDGET && AnchorUart.available(); ++i) {
        const int value = AnchorUart.read();
        if (value < 0) break;
        if (discardUartBacklog) continue;
        UwbLink::Frame frame;
        if (uartParser.feed(uint8_t(value), millis(), frame)) acceptRemoteFrame(frame, millis());
    }
    if (discardUartBacklog && !AnchorUart.available()) discardUartBacklog = false;
}

void acceptRemoteFrame(const UwbLink::Frame &frame, uint32_t now) {
    if (remoteSessionKnown && frame.bootId == remoteBootId &&
        !UwbLink::newerSequence(frame.sequence, remoteSequence)) {
        ++rejectedFrames;
        return;
    }
    if (remoteSessionKnown && frame.bootId != remoteBootId) resetMeasurements();
    remoteSessionKnown = true;
    remoteBootId = frame.bootId;
    remoteSequence = frame.sequence;
    if (frame.kind == UwbLink::TAG_LOST) {
        resetMeasurements();
        return;
    }
    const uint32_t age = uint32_t(frame.ageMs) + UwbLink::WIRE_TIME_MS + uartParser.assemblyMs;
    if (!frame.valid || frame.rangeMm <= 0 || frame.rangeMm > int32_t(MAX_VALID_RANGE_M * 1000) ||
        age > MAX_PAIR_SKEW_MS) {
        ++rejectedFrames;
        clearFreshPair();
        return;
    }
    if (lastConsumedPollKnown && frame.pollStamp == lastConsumedPoll) return;
    anchor2.meters = frame.rangeMm / 1000.0f;
    anchor2.rxPower = frame.rxDb10 / 10.0f;
    anchor2.pollStamp = frame.pollStamp;
    anchor2.timeMs = now;
    anchor2.sourceAgeMs = age;
    anchor2.fresh = true;
}

void clearFreshPair() {
    anchor1.fresh = anchor2.fresh = false;
}

void resetMeasurements() {
    clearFreshPair();
    localPending = false;
    lastPairKnown = false;
    lastConsumedPollKnown = false;
    filterReady = false;
    clearDirectionVotes();
    lastVotedDirection = -1;
    // Keep the calibrated offset across Tag/A2 reconnects.
    if (!offsetCalibrated) resetStartupCalibration();
}

void dropOldSingleSample() {
    const uint32_t now = millis();
    if (anchor1.fresh && uint32_t(now - anchor1.timeMs) > MAX_PAIR_SKEW_MS) anchor1.fresh = false;
    if (anchor2.fresh && uint32_t(now - anchor2.timeMs) + anchor2.sourceAgeMs > MAX_PAIR_SKEW_MS)
        anchor2.fresh = false;
}

bool hasRecentAnchorData(uint32_t now) {
    return lastPairKnown && uint32_t(now - lastPairMs) <= CALIB_MAX_PAIR_GAP_MS;
}

void tryMakePair() {
    if (!anchor1.fresh || !anchor2.fresh || anchor1.pollStamp != anchor2.pollStamp) return;
    // Even same-round data is discarded after the local freshness deadline.
    dropOldSingleSample();
    if (!anchor1.fresh || !anchor2.fresh) return;
    const uint32_t now = millis();
    const float d1 = anchor1.meters;
    const float d2Raw = anchor2.meters;
    lastConsumedPoll = anchor1.pollStamp;
    lastConsumedPollKnown = true;
    clearFreshPair();
    lastPairKnown = true;
    lastPairMs = now;
    targetTimedOut = false;
    ++pairedCount;
    if (DEBUG_LOG) {
        Serial.print("PAIR,n="); Serial.print(pairedCount);
        Serial.print(",d1_raw="); Serial.print(d1, 3);
        Serial.print(",d2_raw="); Serial.print(d2Raw, 3);
        Serial.print(",age1_ms="); Serial.print(uint32_t(now - anchor1.timeMs));
        Serial.print(",age2_ms="); Serial.print(uint32_t(now - anchor2.timeMs) + anchor2.sourceAgeMs);
        Serial.print(",crc_bad="); Serial.println(uartParser.badFrames);
    }
    if (!offsetCalibrated) {
        collectCalibrationPair(d1, d2Raw, now);
        return;
    }
    const float d2 = d2Raw + OFFSET_A2_M;
    if (!isfinite(d2) || d2 <= 0.0f || d2 > MAX_VALID_RANGE_M) return;
    if (ANCHOR_SPACING_M > 0.0f) calculateGeometry(d1, d2);
}

void calculateGeometry(float d1, float d2)
{
    const float L = ANCHOR_SPACING_M;

    if (L <= 0.0f)
    {
        return;
    }

    // Cho phep mot chut sai so do nhieu ranging.
    const float TRIANGLE_TOLERANCE_M = 0.03f;

    // Dieu kien ton tai tam giac.
    if ((d1 + d2) < (L - TRIANGLE_TOLERANCE_M) ||
        fabsf(d1 - d2) > (L + TRIANGLE_TOLERANCE_M))
    {
        if (DEBUG_LOG) Serial.print("GEOM_INVALID,d1=");
        if (DEBUG_LOG) Serial.print(d1, 3);
        if (DEBUG_LOG) Serial.print(",d2=");
        if (DEBUG_LOG) Serial.println(d2, 3);
        return;
    }

    // Toa do tinh tu A1 theo truc A1 -> A2.
    const float xFromA1 =
        (d1 * d1 - d2 * d2 + L * L) / (2.0f * L);

    float ySquared =
        d1 * d1 - xFromA1 * xFromA1;

    if (ySquared < -0.01f)
    {
        if (DEBUG_LOG) Serial.println("GEOM_INVALID,no_real_intersection");
        return;
    }

    if (ySquared < 0.0f)
    {
        ySquared = 0.0f;
    }

    // Chon nghiem y > 0 = Tag o phia truoc xe.
    const float y = sqrtf(ySquared);

    // Doi goc toa do ve tam A1-A2.
    const float x =
        xFromA1 - (L * 0.5f);

    // 0 do = thang truoc; duong = ve A2; am = ve A1.
    const float angleDeg =
        atan2f(x, y) * 180.0f / PI;

    // Day la noi DUY NHAT them Kalman vao ket qua hinh hoc.
    if (isfinite(angleDeg)) updateFilteredDirection(angleDeg, d1, d2);
}

void updateFilteredDirection(float angleDeg, float d1, float d2)
{
    const uint32_t now = millis();
    const bool first = !filterReady || uint32_t(now - lastAngleMs) > FILTER_RESET_MS;
    if (first) {
        filteredAngle = angleDeg;            // Mau dau: lay lam diem bat dau.
        angleVariance = KALMAN_R;
        filterReady = true;
        directionIndex = 3;
    } else {
        const float dt = uint32_t(now - lastAngleMs) / 1000.0f;
        angleVariance += KALMAN_Q * dt;      // Do bat dinh tang theo thoi gian.
        const float gain = angleVariance / (angleVariance + KALMAN_R);
        filteredAngle += gain * (angleDeg - filteredAngle); // Tron cu va moi.
        angleVariance *= (1.0f - gain);      // Cap nhat do bat dinh sau khi do.
    }
    lastAngleMs = now;

    // 0..10 do: THANG; 10..30: NHE; 30..60: CHEO; 60..90: BEN.
    // Dau am: ve A1 (trai), dau duong: ve A2 (phai).
    const float boundary[6] = {-60, -30, -10, 10, 30, 60};
    const float hysteresis = first ? 0.0f : 2.0f; // Vung tre de nhan do nhap nhay.
    while (directionIndex < 6 && filteredAngle > boundary[directionIndex] + hysteresis)
        ++directionIndex;
    while (directionIndex > 0 && filteredAngle < boundary[directionIndex - 1] - hysteresis)
        --directionIndex;

    // Moi cap range hop le chi bo 1 phieu cho nhan sau Kalman + hysteresis.
    addDirectionVote(directionIndex, d1, d2, filteredAngle, now);
}

void clearDirectionVotes()
{
    for (uint8_t i = 0; i < DIRECTION_COUNT; ++i) {
        directionVotes[i] = DirectionVote{};
    }
    voteSequence = 0;
}

void flushDirectionVotes(uint32_t now)
{
    if (!offsetCalibrated) return;
    const uint32_t elapsed = uint32_t(now - voteWindowStartMs);
    if (elapsed < VOTE_WINDOW_MS) return;

    // Neu loop bi dung qua ca mot cua so tiep theo, khong phat lai ket qua cu.
    if (elapsed >= 2 * VOTE_WINDOW_MS) clearDirectionVotes();

    int winner = -1;
    for (uint8_t i = 0; i < DIRECTION_COUNT; ++i) {
        if (directionVotes[i].count == 0) continue;
        if (winner < 0 ||
            directionVotes[i].count > directionVotes[winner].count ||
            (directionVotes[i].count == directionVotes[winner].count &&
             directionVotes[i].lastOrder > directionVotes[winner].lastOrder)) {
            winner = i;
        }
    }

    // Chi uu tien ket qua truoc neu no cung dang co so phieu cao nhat.
    if (winner >= 0 && lastVotedDirection >= 0 &&
        directionVotes[lastVotedDirection].count == directionVotes[winner].count) {
        winner = lastVotedDirection;
    }

    if (winner < 0) {
        Serial.println(PRINT_DETAILS ? "KHONG CO DU LIEU | valid=0" : "KHONG CO DU LIEU");
        lastVotedDirection = -1;
    } else {
        Serial.print(DIRECTION_NAMES[winner]);
        if (PRINT_DETAILS) {
            // Lay mau moi nhat CUA HUONG THANG VOTE, khong lay mau cua huong khac.
            Serial.print(" | dA1=");
            Serial.print(directionVotes[winner].d1, 3);
            Serial.print(" m | dA2=");
            Serial.print(directionVotes[winner].d2, 3);
            Serial.print(" m | goc=");
            Serial.print(directionVotes[winner].angleDeg, 1);
            Serial.print(" do | valid=1");
        }
        Serial.println();
        lastVotedDirection = winner;
    }

    clearDirectionVotes();
    // Giu moc cua so; moi phieu thuoc dung mot khoang [bat dau, bat dau + 500).
    voteWindowStartMs += (elapsed / VOTE_WINDOW_MS) * VOTE_WINDOW_MS;
}

void addDirectionVote(int direction, float d1, float d2, float angleDeg, uint32_t now)
{
    // Chot cua so cu TRUOC khi nhan mau tai moc 500 ms vao cua so moi.
    flushDirectionVotes(now);
    if (direction < 0 || direction >= DIRECTION_COUNT) return;

    DirectionVote &vote = directionVotes[direction];
    ++vote.count;
    vote.lastOrder = ++voteSequence;
    vote.d1 = d1;
    vote.d2 = d2;
    vote.angleDeg = angleDeg;
}

void resetStartupCalibration()
{
    calibrationStarted = false;
    calibrationCollecting = false;
    calibrationPairs = 0;
    calibrationMeanD1 = 0.0f;
    calibrationMeanD2 = 0.0f;
    calibrationM2D1 = 0.0f;
    calibrationM2D2 = 0.0f;
}

void collectCalibrationPair(float d1Raw, float d2Raw, uint32_t now)
{
    if (offsetCalibrated) return;
    if (!isfinite(d1Raw) || !isfinite(d2Raw) || d1Raw <= 0.0f || d2Raw <= 0.0f ||
        d1Raw > MAX_VALID_RANGE_M || d2Raw > MAX_VALID_RANGE_M) return;

    if (calibrationStarted &&
        (uint32_t(now - calibrationLastPairMs) > CALIB_MAX_PAIR_GAP_MS ||
         uint32_t(now - calibrationStartMs) >= CALIB_TIMEOUT_MS)) {
        resetStartupCalibration();
        Serial.println("CALIB_THU_LAI: NGAT MAU HOAC CHUA DU MAU, GIU TAG YEN.");
    }

    if (!calibrationStarted) {
        calibrationStarted = true;
        calibrationStartMs = now;
        calibrationLastPairMs = now;
    }
    calibrationLastPairMs = now;
    if (uint32_t(now - calibrationStartMs) < CALIB_SETTLE_MS) return;

    if (!calibrationCollecting) {
        calibrationCollecting = true;
        calibrationCollectStartMs = now;
    }

    // Trung binh va phuong sai Welford, khong can mang hay thu vien them.
    ++calibrationPairs;
    const float n = float(calibrationPairs);
    const float delta1 = d1Raw - calibrationMeanD1;
    calibrationMeanD1 += delta1 / n;
    calibrationM2D1 += delta1 * (d1Raw - calibrationMeanD1);
    const float delta2 = d2Raw - calibrationMeanD2;
    calibrationMeanD2 += delta2 / n;
    calibrationM2D2 += delta2 * (d2Raw - calibrationMeanD2);

    if (calibrationPairs < CALIB_MIN_PAIRS || calibrationPairs < 2 ||
        uint32_t(now - calibrationCollectStartMs) < CALIB_MIN_COLLECT_MS) return;

    const float variance1 = calibrationM2D1 / float(calibrationPairs - 1);
    const float variance2 = calibrationM2D2 / float(calibrationPairs - 1);
    const float std1 = sqrtf(variance1 > 0.0f ? variance1 : 0.0f);
    const float std2 = sqrtf(variance2 > 0.0f ? variance2 : 0.0f);
    if (std1 > CALIB_MAX_STD_M || std2 > CALIB_MAX_STD_M) {
        Serial.print("CALIB_CHUA_ON_DINH,sdA1=");
        Serial.print(std1, 3);
        Serial.print(",sdA2=");
        Serial.println(std2, 3);
        Serial.println("CALIB: GIU TAG YEN, TU LAY LAI MAU.");
        resetStartupCalibration();
        return;
    }

    // Khi d1 = d2 sau bu, moi canh phai lon hon nua khoang cach giua anchor.
    if (ANCHOR_SPACING_M > 0.0f && calibrationMeanD1 <= ANCHOR_SPACING_M * 0.5f) {
        Serial.println("CALIB_RANGE_A1_QUA_NHO: KIEM TRA RANGE, TU LAY LAI MAU.");
        resetStartupCalibration();
        return;
    }

    OFFSET_A2_M = calibrationMeanD1 - calibrationMeanD2;
    offsetCalibrated = true;
    // Xoa mau RAW va lich su goc/vote; chi dung cap MOI da bu tu day tro di.
    resetMeasurements();
    voteWindowStartMs = now;

    Serial.print("CALIB_OK | OFFSET_A2_M=");
    Serial.print(OFFSET_A2_M, 4);
    Serial.print(" m | dA1_raw_tb=");
    Serial.print(calibrationMeanD1, 3);
    Serial.print(" m | dA2_raw_tb=");
    Serial.print(calibrationMeanD2, 3);
    Serial.println(" m");
    Serial.println("RUN: DA CHOT OFFSET, CO THE DI CHUYEN.");
}

void serviceStartupCalibration(uint32_t now)
{
    if (offsetCalibrated) return;
    if (calibrationStarted &&
        (uint32_t(now - calibrationLastPairMs) > CALIB_MAX_PAIR_GAP_MS ||
         uint32_t(now - calibrationStartMs) >= CALIB_TIMEOUT_MS)) {
        resetStartupCalibration();
        clearFreshPair();
        Serial.println("CALIB_THU_LAI: MAT CAP MAU HOAC QUA THOI GIAN, GIU TAG YEN.");
    }

    if (uint32_t(now - calibrationLastStatusMs) < 1000) return;
    calibrationLastStatusMs = now;
    if (!hasRecentAnchorData(now)) {
        Serial.println("CALIB_CHO_A2_A1: CHUA CO CAP CUNG POLL CON MOI.");
    } else if (!calibrationStarted) {
        Serial.println("CALIB_CHO_CAP_RANGE_HOP_LE.");
    } else if (!calibrationCollecting) {
        Serial.println("CALIB_CHO_ON_DINH: GIU TAG YEN.");
    } else {
        Serial.print("CALIB_LAY_MAU,n=");
        Serial.print(calibrationPairs);
        Serial.print(",dA1_raw_tb=");
        Serial.print(calibrationMeanD1, 3);
        Serial.print(",dA2_raw_tb=");
        Serial.println(calibrationMeanD2, 3);
    }
}
