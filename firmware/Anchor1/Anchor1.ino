/*
  A1: nhan d1 tu BU01 va d2 RAW qua UART, ghep cung luot POLL.
  Luong xu ly: tu can offset -> tinh goc -> Kalman -> vote 500 ms.
  Khi bat/reset A1: giu Tag yen chinh giua phia truoc, cach tam 1 m,
  cho CALIB_OK roi di chuyen. Mat/noi lai Tag hoac A2 van giu offset.
  Doc setup() va loop() truoc; cac ham ben duoi theo tung cong viec.
*/
#include <SPI.h>
#include <math.h>
#include "DW1000.h"
#include "DW1000Ranging.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// 1. CAU HINH: giu chan, dia chi va thong so cua ban goc.
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
const uint16_t TAG_SHORT = 0x007D;
const uint64_t POLL_MASK = (uint64_t(1) << 40) - 1;
const uint32_t UART_TRANSFER_MS = 6; // Du phong truyen dong chu ngan o 115200 baud.
const uint32_t UART_LINE_TIMEOUT_MS = 20;
const uint8_t UART_LINE_SIZE = 64;

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

// 2. DU LIEU: mau do, UART, bo loc, phieu huong va mau calibration.
struct RangeSample {
    float meters = 0.0f;
    uint64_t pollStamp = 0;
    uint32_t timeMs = 0; // Thoi diem do d1 / nhan d2, deu tren dong ho A1.
    uint32_t sourceAgeMs = 0;
    bool fresh = false;
};
RangeSample anchor1, anchor2;
bool tagChanged = false;
char uartLine[UART_LINE_SIZE];
uint8_t uartLength = 0;
bool uartReading = false;
uint32_t uartLineStartMs = 0;
bool lastRemotePollKnown = false;
uint64_t lastRemotePoll = 0;
uint32_t lastUartServiceMs = 0;
bool discardUartBacklog = true;
bool lastPairKnown = false, lastConsumedPollKnown = false;
uint64_t lastConsumedPoll = 0;
uint32_t lastPairMs = 0;
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
uint32_t calibrationStartMs = 0;
uint32_t calibrationCollectStartMs = 0;
uint32_t calibrationLastPairMs = 0;
uint32_t calibrationLastStatusMs = 0;
uint32_t calibrationPairs = 0;
float calibrationMeanD1 = 0.0f;
float calibrationMeanD2 = 0.0f;
float calibrationM2D1 = 0.0f;
float calibrationM2D2 = 0.0f;

// 3. CHAY CHINH: UWB -> UART -> ghep cap -> calibration / xuat huong.
void setup() {
    Serial.begin(115200);
    AnchorUart.setRxBufferSize(256);
    AnchorUart.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    delay(1000);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
    DW1000.setAntennaDelay(ANTENNA_DELAY);
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachBlinkDevice(tagConnectionChanged);
    DW1000Ranging.attachInactiveDevice(tagConnectionChanged);
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
    if (tagChanged) {
        resetMeasurements();
        tagChanged = false;
    }
    serviceUart();
    tryMakePair(); // Tu bo mau cu, chi ghep hai mau cung POLL.

    const uint32_t now = millis();
    if (offsetCalibrated && lastPairKnown && uint32_t(now - lastPairMs) > TARGET_TIMEOUT_MS)
        resetMeasurements();
    if (!offsetCalibrated) serviceStartupCalibration(now);
    else flushDirectionVotes(now);
}

// 4. NHAN DU LIEU: callback chi luu mau; UART doc tung byte, khong cho.
void newRange() {
    DW1000Device *device = DW1000Ranging.getDistantDevice();
    if (!device || device->getShortAddress() != TAG_SHORT) return;
    // Callback chi luu d1; ghep cap va tinh goc o loop().
    anchor1.meters = device->getRange();
    anchor1.pollStamp = uint64_t(device->timePollSent.getTimestamp()) & POLL_MASK;
    anchor1.timeMs = millis();
    anchor1.sourceAgeMs = 0;
    anchor1.fresh = true;
}

void tagConnectionChanged(DW1000Device *device) {
    // Mat Tag va noi lai Tag deu phai bo ket qua cu.
    if (device && device->getShortAddress() == TAG_SHORT) tagChanged = true;
}

void serviceUart() {
    const uint32_t now = millis();
    // Bo byte cu khi loop ngung qua lau hoac UART bi don nhieu dong.
    if (uint32_t(now - lastUartServiceMs) > MAX_PAIR_SKEW_MS ||
        AnchorUart.available() > 2 * UART_LINE_SIZE) {
        discardUartBacklog = true;
        uartReading = false;
        resetMeasurements();
    }
    lastUartServiceMs = now;
    if (uartReading && uint32_t(now - uartLineStartMs) > UART_LINE_TIMEOUT_MS)
        uartReading = false;

    for (uint16_t i = 0; i < UART_BYTE_BUDGET && AnchorUart.available(); ++i) {
        const char c = char(AnchorUart.read());
        if (discardUartBacklog) continue;
        if (c == '$') { // Dau bat dau: tu tim lai dong sau khi thieu byte.
            uartReading = true;
            uartLength = 0;
            uartLineStartMs = now;
        } else if (uartReading && c == '\n') {
            uartLine[uartLength] = '\0';
            readUartLine(now);
            uartReading = false;
        } else if (uartReading) {
            if (uartLength < UART_LINE_SIZE - 1) uartLine[uartLength++] = c;
            else { uartReading = false; ++rejectedFrames; }
        }
    }
    if (discardUartBacklog && !AnchorUart.available()) discardUartBacklog = false;
}

uint8_t uartChecksum(const char *text) {
    uint8_t sum = 0;
    for (uint8_t i = 0; text[i] != '\0'; ++i) sum ^= uint8_t(text[i]);
    return sum;
}

void readUartLine(uint32_t now) {
    // Dong co dang A2,poll,range_mm,age_ms,valid*HH. HH la checksum XOR.
    char *star = strchr(uartLine, '*');
    if (!star || strlen(star + 1) != 2) { ++rejectedFrames; return; }
    char *end;
    const unsigned long receivedSum = strtoul(star + 1, &end, 16);
    *star = '\0';
    if (*end != '\0' || uartChecksum(uartLine) != receivedSum) { ++rejectedFrames; return; }

    unsigned long long poll;
    long rangeMm;
    unsigned long ageMs;
    int valid;
    char extra;
    // Gioi han so chu so va bo dong co du ky tu o cuoi.
    if (sscanf(uartLine, "A2,%13llu,%5ld,%2lu,%2d%c", &poll, &rangeMm, &ageMs, &valid, &extra) != 4 ||
        poll > POLL_MASK || ageMs > MAX_PAIR_SKEW_MS || (valid != -1 && valid != 0 && valid != 1)) {
        ++rejectedFrames;
        return;
    }
    if (valid == -1) { // A2 bao Tag mat/khoi dong lai.
        resetMeasurements();
        lastRemotePollKnown = false;
        return;
    }
    if (lastRemotePollKnown && uint64_t(poll) == lastRemotePoll) return;
    lastRemotePoll = uint64_t(poll);
    lastRemotePollKnown = true;
    const uint32_t age = uint32_t(ageMs) + UART_TRANSFER_MS + uint32_t(now - uartLineStartMs);
    if (valid != 1 || rangeMm <= 0 || rangeMm > long(MAX_VALID_RANGE_M * 1000) || age > MAX_PAIR_SKEW_MS) {
        ++rejectedFrames;
        clearFreshPair();
        return;
    }
    if (lastConsumedPollKnown && uint64_t(poll) == lastConsumedPoll) return;
    anchor2.meters = rangeMm / 1000.0f;
    anchor2.pollStamp = uint64_t(poll);
    anchor2.timeMs = now;
    anchor2.sourceAgeMs = age;
    anchor2.fresh = true;
}

// 5. GHEP CAP: cung POLL, tuoi <= 80 ms, moi cap dung mot lan.
void clearFreshPair() {
    anchor1.fresh = anchor2.fresh = false;
}

void resetMeasurements() {
    clearFreshPair();
    lastPairKnown = false;
    lastConsumedPollKnown = false;
    filterReady = false;
    clearDirectionVotes();
    lastVotedDirection = -1;
    // Da can xong thi giu offset trong RAM; chua xong thi lay lai mau.
    if (!offsetCalibrated) resetStartupCalibration();
}

void dropOldSingleSample() {
    const uint32_t now = millis();
    if (anchor1.fresh && uint32_t(now - anchor1.timeMs) > MAX_PAIR_SKEW_MS) anchor1.fresh = false;
    if (anchor2.fresh && uint32_t(now - anchor2.timeMs) + anchor2.sourceAgeMs > MAX_PAIR_SKEW_MS)
        anchor2.fresh = false;
}

void tryMakePair() {
    dropOldSingleSample();
    if (!anchor1.fresh || !anchor2.fresh || anchor1.pollStamp != anchor2.pollStamp) return;
    if (lastConsumedPollKnown && anchor1.pollStamp == lastConsumedPoll) {
        clearFreshPair();
        return;
    }
    const float d1 = anchor1.meters;
    const float d2Raw = anchor2.meters;
    clearFreshPair(); // Moi cap chi dung mot lan.
    if (!isfinite(d1) || !isfinite(d2Raw) || d1 <= 0 || d2Raw <= 0 ||
        d1 > MAX_VALID_RANGE_M || d2Raw > MAX_VALID_RANGE_M) return;

    const uint32_t now = millis();
    lastConsumedPoll = anchor1.pollStamp;
    lastConsumedPollKnown = lastPairKnown = true;
    lastPairMs = now;
    ++pairedCount;
    if (DEBUG_LOG) {
        Serial.printf("PAIR,n=%lu,d1_raw=%.3f,d2_raw=%.3f,age1_ms=%lu,age2_ms=%lu,uart_bad=%lu\n",
                      (unsigned long)pairedCount, d1, d2Raw,
                      (unsigned long)(now - anchor1.timeMs),
                      (unsigned long)(now - anchor2.timeMs + anchor2.sourceAgeMs),
                      (unsigned long)rejectedFrames);
    }
    if (!offsetCalibrated) {
        collectCalibrationPair(d1, d2Raw, now); // Khi can offset, chi dung RAW.
        return;
    }
    const float d2 = d2Raw + OFFSET_A2_M; // Bu A2 dung mot lan tai day.
    if (!isfinite(d2) || d2 <= 0 || d2 > MAX_VALID_RANGE_M) return;
    calculateGeometry(d1, d2);
}

// 6. GOC VA KALMAN: giu cong thuc va vung tre phan huong cua Tag cu.
void calculateGeometry(float d1, float d2) {
    const float L = ANCHOR_SPACING_M;
    if (L <= 0) return;
    const float TRIANGLE_TOLERANCE_M = 0.03f;
    if (d1 + d2 < L - TRIANGLE_TOLERANCE_M || fabsf(d1 - d2) > L + TRIANGLE_TOLERANCE_M) {
        if (DEBUG_LOG) Serial.printf("GEOM_INVALID,d1=%.3f,d2=%.3f\n", d1, d2);
        return;
    }
    // Tam giac A1-A2-Tag: tim x tu A1, roi doi ve trung diem hai Anchor.
    const float xFromA1 = (d1 * d1 - d2 * d2 + L * L) / (2.0f * L);
    float ySquared = d1 * d1 - xFromA1 * xFromA1;
    if (ySquared < -0.01f) {
        if (DEBUG_LOG) Serial.println("GEOM_INVALID,no_real_intersection");
        return;
    }
    if (ySquared < 0) ySquared = 0;
    const float y = sqrtf(ySquared); // Chi chon Tag o phia truoc xe.
    const float x = xFromA1 - L * 0.5f;
    const float angleDeg = atan2f(x, y) * 180.0f / PI;
    // Goc am: trai/A1. Goc duong: phai/A2. 0 do: thang.
    if (isfinite(angleDeg)) updateFilteredDirection(angleDeg, d1, d2);
}

void updateFilteredDirection(float angleDeg, float d1, float d2) {
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

// 7. VOTE: moi 500 ms chon huong nhieu phieu nhat.
void clearDirectionVotes() {
    for (uint8_t i = 0; i < DIRECTION_COUNT; ++i) {
        directionVotes[i] = DirectionVote{};
    }
    voteSequence = 0;
}

void flushDirectionVotes(uint32_t now) {
    if (!offsetCalibrated) return;
    const uint32_t elapsed = uint32_t(now - voteWindowStartMs);
    if (elapsed < VOTE_WINDOW_MS) return;
    if (elapsed >= 2 * VOTE_WINDOW_MS) clearDirectionVotes(); // Loop dung lau: bo phieu cu.

    int winner = -1;
    for (uint8_t i = 0; i < DIRECTION_COUNT; ++i) {
        if (directionVotes[i].count == 0) continue;
        if (winner < 0 || directionVotes[i].count > directionVotes[winner].count ||
            (directionVotes[i].count == directionVotes[winner].count &&
             directionVotes[i].lastOrder > directionVotes[winner].lastOrder)) winner = i;
    }
    // Hoa phieu: giu huong da xuat neu no cung co so phieu cao nhat.
    if (winner >= 0 && lastVotedDirection >= 0 &&
        directionVotes[lastVotedDirection].count == directionVotes[winner].count)
        winner = lastVotedDirection;

    if (winner < 0) Serial.println(PRINT_DETAILS ? "KHONG CO DU LIEU | valid=0" : "KHONG CO DU LIEU");
    else {
        Serial.print(DIRECTION_NAMES[winner]);
        if (PRINT_DETAILS) {
            const DirectionVote &vote = directionVotes[winner];
            Serial.printf(" | dA1=%.3f m | dA2=%.3f m | goc=%.1f do | valid=1",
                          vote.d1, vote.d2, vote.angleDeg);
        }
        Serial.println();
    }
    lastVotedDirection = winner;
    clearDirectionVotes();
    voteWindowStartMs += (elapsed / VOTE_WINDOW_MS) * VOTE_WINDOW_MS;
}

void addDirectionVote(int direction, float d1, float d2, float angleDeg, uint32_t now) {
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

// 8. CALIBRATION: on dinh 2 s, thu >= 30 cap trong >= 3 s.
void resetStartupCalibration() {
    calibrationStarted = false;
    calibrationPairs = 0;
    calibrationMeanD1 = calibrationMeanD2 = 0;
    calibrationM2D1 = calibrationM2D2 = 0;
}

void collectCalibrationPair(float d1Raw, float d2Raw, uint32_t now) {
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

    if (calibrationPairs == 0) calibrationCollectStartMs = now;

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
        Serial.printf("CALIB_CHUA_ON_DINH,sdA1=%.3f,sdA2=%.3f\n", std1, std2);
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

    Serial.printf("CALIB_OK | OFFSET_A2_M=%.4f m | dA1_raw_tb=%.3f m | dA2_raw_tb=%.3f m\n",
                  OFFSET_A2_M, calibrationMeanD1, calibrationMeanD2);
    Serial.println("RUN: DA CHOT OFFSET, CO THE DI CHUYEN.");
}

void serviceStartupCalibration(uint32_t now) {
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
    if (!lastPairKnown || uint32_t(now - lastPairMs) > CALIB_MAX_PAIR_GAP_MS) {
        Serial.println("CALIB_CHO_A2_A1: CHUA CO CAP CUNG POLL CON MOI.");
    } else if (!calibrationStarted) {
        Serial.println("CALIB_CHO_CAP_RANGE_HOP_LE.");
    } else if (calibrationPairs == 0) {
        Serial.println("CALIB_CHO_ON_DINH: GIU TAG YEN.");
    } else {
        Serial.printf("CALIB_LAY_MAU,n=%lu,dA1_raw_tb=%.3f,dA2_raw_tb=%.3f\n",
                      (unsigned long)calibrationPairs, calibrationMeanD1, calibrationMeanD2);
    }
}
