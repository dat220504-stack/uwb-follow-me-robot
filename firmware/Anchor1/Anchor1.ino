/* A1: nhan d1/d2 -> can offset -> tinh goc -> Kalman -> vote 500 ms.
   Khi bat A1, giu Tag yen o giua, phia truoc khoang 1 m; cho CALIB_OK.
   Dung nguyen Tag, A2 va thu vien DW1000 hien tai. */
#include <SPI.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "DW1000.h"
#include "DW1000Ranging.h"

char ADDRESS[] = "86:17:5B:D5:A9:9A:E2:9C";
const uint16_t TAG_SHORT = 0x007D, ANTENNA_DELAY = 16461;
const uint8_t SCK_PIN = 18, MISO_PIN = 19, MOSI_PIN = 23, CS_PIN = 4;
const uint8_t RST_PIN = 27, IRQ_PIN = 34, RX_PIN = 16, TX_PIN = 17;
const uint32_t MAX_AGE = 80, LOST_MS = 500, VOTE_MS = 500;
const uint32_t SETTLE_MS = 2000, COLLECT_MS = 3000, GAP_MS = 1500, CAL_TIMEOUT = 60000;
const uint16_t MIN_PAIRS = 30;
const float L = 0.5f, MAX_RANGE = 10.0f, MAX_STD = 0.10f;
const float KALMAN_Q = 36.0f, KALMAN_R = 25.0f;
const uint32_t FILTER_RESET_MS = 800;
const bool PRINT_DETAILS = true, DEBUG_LOG = false;
const uint64_t POLL_MASK = (uint64_t(1) << 40) - 1;
const uint64_t NO_POLL = UINT64_MAX; // Nam ngoai timestamp 40 bit: chua co mau.
HardwareSerial Uart(2);

struct Sample {
    float range = 0;
    uint64_t poll = 0;
    uint32_t time = 0; // Thoi diem mau quy ve dong ho A1, da tru tuoi A2 gui.
    bool fresh = false;
};
Sample a1, a2;
uint64_t remotePoll = NO_POLL, usedPoll = NO_POLL;
bool tagChanged = false, havePair = false, calibrated = false, filterReady = false;
float offset = 0, angle = 0, variance = 0;
int direction = 3, lastDirection = -1;
uint32_t lastPair = 0, lastAngle = 0, voteStart = 0, voteOrder = 0, lastStatus = 0;

struct Calibration {
    bool started = false;
    uint32_t start = 0, collect = 0, last = 0, count = 0;
    float mean[2] = {}, m2[2] = {}; // Trung binh va tong sai lech binh phuong.
};
Calibration cal;
struct Vote {
    uint32_t count = 0, order = 0;
    float d1 = 0, d2 = 0, angle = 0;
};
Vote votes[7];
const char *NAMES[] = {"BEN TRAI", "TRAI CHEO", "TRAI NHE", "THANG",
                       "PHAI NHE", "PHAI CHEO", "BEN PHAI"};
char line[64];
int lineLength = -1; // -1: chua gap dau '$'.
uint32_t lineStart = 0, lastUart = 0;
bool discardUart = true;

void setup() {
    Serial.begin(115200);
    Uart.setRxBufferSize(256);
    Uart.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    delay(1000);
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
    DW1000Ranging.initCommunication(RST_PIN, CS_PIN, IRQ_PIN);
    DW1000.setAntennaDelay(ANTENNA_DELAY);
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachBlinkDevice(connectionChanged);
    DW1000Ranging.attachInactiveDevice(connectionChanged);
    DW1000Ranging.startAsAnchor(ADDRESS, DW1000.MODE_SHORTDATA_FAST_LOWPOWER, false);
    lastUart = voteStart = lastStatus = millis();
    Serial.println("A1 READY | GIU TAG YEN O GIUA, PHIA TRUOC 1 m; CHO CALIB_OK.");
}

void loop() {
    DW1000Ranging.loop();
    const uint32_t now = millis();
    if (tagChanged) {
        resetMeasurements();
        tagChanged = false;
    }
    readUart(now);
    float d1 = 0, d2 = 0;
    const bool gotPair = getPair(d1, d2, now);
    if (calibrated && havePair && uint32_t(now - lastPair) > LOST_MS) resetMeasurements();
    if (!calibrated) calibrate(d1, d2, gotPair, now);
    else if (gotPair) calculateAngle(d1, d2 + offset, now);
    finishVote(now);
}

void newRange() {
    DW1000Device *device = DW1000Ranging.getDistantDevice();
    if (!device || device->getShortAddress() != TAG_SHORT) return;
    a1 = {device->getRange(), uint64_t(device->timePollSent.getTimestamp()) & POLL_MASK,
          millis(), true};
}

void connectionChanged(DW1000Device *device) {
    if (device && device->getShortAddress() == TAG_SHORT) tagChanged = true;
}

void clearVotes() {
    for (int i = 0; i < 7; ++i) votes[i] = {};
    voteOrder = 0;
}

void resetMeasurements() {
    a1.fresh = a2.fresh = false;
    usedPoll = NO_POLL;
    havePair = filterReady = false;
    lastDirection = -1;
    clearVotes();
    if (!calibrated) cal = {}; // Da chot offset thi giu lai khi mat/noi lai Tag.
}

// UART A2: $A2,poll,range_mm,age_ms,valid*HH\n. HH la checksum XOR.
void readUart(uint32_t now) {
    if (uint32_t(now - lastUart) > MAX_AGE || Uart.available() > 128) {
        discardUart = true;
        lineLength = -1;
        resetMeasurements();
    }
    lastUart = now;
    if (lineLength >= 0 && uint32_t(now - lineStart) > 20) lineLength = -1;
    for (int i = 0; i < 64 && Uart.available(); ++i) {
        const char c = char(Uart.read());
        if (discardUart) continue;
        if (c == '$') {
            lineLength = 0;
            lineStart = now;
        } else if (lineLength >= 0 && c == '\n') {
            line[lineLength] = '\0';
            readLine(now);
            lineLength = -1;
        } else if (lineLength >= 0) {
            if (lineLength < 63) line[lineLength++] = c;
            else lineLength = -1;
        }
    }
    if (!Uart.available()) discardUart = false;
}

void readLine(uint32_t now) {
    char *star = strchr(line, '*'), *end;
    if (!star || strlen(star + 1) != 2) return;
    const unsigned long received = strtoul(star + 1, &end, 16);
    *star = '\0';
    uint8_t checksum = 0;
    for (int i = 0; line[i]; ++i) checksum ^= uint8_t(line[i]);
    if (*end || received != checksum) return;
    unsigned long long poll;
    long mm;
    unsigned long age;
    int valid;
    char extra;
    if (sscanf(line, "A2,%13llu,%5ld,%2lu,%2d%c", &poll, &mm, &age, &valid, &extra) != 4 ||
        poll > POLL_MASK || age > MAX_AGE || valid < -1 || valid > 1) return;
    if (valid == -1) {
        resetMeasurements();
        remotePoll = NO_POLL;
        return;
    }
    if (poll == remotePoll) return; // Dong lap khong duoc lam moi tuoi mau.
    remotePoll = poll;
    age += 6 + uint32_t(now - lineStart); // Du phong truyen + thoi gian rap dong.
    if (valid != 1 || mm <= 0 || mm > MAX_RANGE * 1000 || age > MAX_AGE) {
        a1.fresh = a2.fresh = false;
        return;
    }
    a2 = {mm / 1000.0f, uint64_t(poll), now - uint32_t(age), true};
}

bool getPair(float &d1, float &d2, uint32_t now) {
    if (uint32_t(now - a1.time) > MAX_AGE) a1.fresh = false;
    if (uint32_t(now - a2.time) > MAX_AGE) a2.fresh = false;
    if (!a1.fresh || !a2.fresh || a1.poll != a2.poll) return false;
    a1.fresh = a2.fresh = false;
    d1 = a1.range;
    d2 = a2.range;
    if (a1.poll == usedPoll || !isfinite(d1) || !isfinite(d2) ||
        d1 <= 0 || d2 <= 0 || d1 > MAX_RANGE || d2 > MAX_RANGE) return false;
    usedPoll = a1.poll;
    havePair = true;
    lastPair = now;
    if (DEBUG_LOG) Serial.printf("PAIR,d1_raw=%.3f,d2_raw=%.3f\n", d1, d2);
    return true;
}

// Calibration chi dung RAW: cho 2 s, thu >=30 cap trong >=3 s, do lech chuan <=10 cm.
void calibrate(float d1, float d2, bool gotPair, uint32_t now) {
    if (cal.started && (uint32_t(now - cal.last) > GAP_MS || uint32_t(now - cal.start) >= CAL_TIMEOUT)) {
        cal = {};
        a1.fresh = a2.fresh = false;
        Serial.println("CALIB_THU_LAI: MAT CAP MAU HOAC QUA THOI GIAN.");
    }
    if (uint32_t(now - lastStatus) >= 1000) {
        lastStatus = now;
        Serial.printf("CALIB: GIU TAG YEN | so cap=%lu\n", (unsigned long)cal.count);
    }
    if (!gotPair) return;
    if (!cal.started) {
        cal.started = true;
        cal.start = now;
    }
    cal.last = now;
    if (uint32_t(now - cal.start) < SETTLE_MS) return;
    if (cal.count == 0) cal.collect = now;
    ++cal.count;
    const float ranges[2] = {d1, d2};
    for (int i = 0; i < 2; ++i) { // Cung mot cach tinh cho d1 va d2.
        const float delta = ranges[i] - cal.mean[i];
        cal.mean[i] += delta / cal.count;
        cal.m2[i] += delta * (ranges[i] - cal.mean[i]);
    }
    if (cal.count < MIN_PAIRS || uint32_t(now - cal.collect) < COLLECT_MS) return;
    const float sd1 = sqrtf(fmaxf(0, cal.m2[0] / (cal.count - 1)));
    const float sd2 = sqrtf(fmaxf(0, cal.m2[1] / (cal.count - 1)));
    if (sd1 > MAX_STD || sd2 > MAX_STD || (L > 0 && cal.mean[0] <= L * 0.5f)) {
        Serial.println("CALIB_CHUA_ON_DINH: GIU TAG YEN, KIEM TRA RANGE; TU LAY LAI MAU.");
        cal = {};
        return;
    }
    offset = cal.mean[0] - cal.mean[1];
    calibrated = true;
    resetMeasurements();
    voteStart = now;
    Serial.printf("CALIB_OK | OFFSET_A2_M=%.4f m\n", offset);
}

void calculateAngle(float d1, float d2, uint32_t now) {
    if (L <= 0 || !isfinite(d2) || d2 <= 0 || d2 > MAX_RANGE) return;
    if (d1 + d2 < L - 0.03f || fabsf(d1 - d2) > L + 0.03f) return;
    const float x1 = (d1 * d1 - d2 * d2 + L * L) / (2.0f * L);
    const float y2 = d1 * d1 - x1 * x1;
    if (y2 < -0.01f) return;
    const float measured = atan2f(x1 - L * 0.5f, sqrtf(fmaxf(0, y2))) * 180.0f / PI;
    if (isfinite(measured)) filterAngle(measured, d1, d2, now);
}

void filterAngle(float measured, float d1, float d2, uint32_t now) {
    const bool first = !filterReady || uint32_t(now - lastAngle) > FILTER_RESET_MS;
    if (first) {
        angle = measured;
        variance = KALMAN_R;
        direction = 3;
        filterReady = true;
    } else {
        variance += KALMAN_Q * (uint32_t(now - lastAngle) / 1000.0f);
        const float gain = variance / (variance + KALMAN_R);
        angle += gain * (measured - angle);
        variance *= 1.0f - gain;
    }
    lastAngle = now;
    const float limits[] = {-60, -30, -10, 10, 30, 60};
    const float margin = first ? 0 : 2; // Vung tre 2 do, tranh nhay nhan.
    while (direction < 6 && angle > limits[direction] + margin) ++direction;
    while (direction > 0 && angle < limits[direction - 1] - margin) --direction;
    finishVote(now); // Chot cua so cu truoc khi them phieu tai moc 500 ms.
    Vote &v = votes[direction];
    v = {v.count + 1, ++voteOrder, d1, d2, angle};
}

void finishVote(uint32_t now) {
    if (!calibrated || uint32_t(now - voteStart) < VOTE_MS) return;
    const uint32_t elapsed = now - voteStart;
    if (elapsed >= 2 * VOTE_MS) clearVotes();
    int winner = -1;
    for (int i = 0; i < 7; ++i) {
        if (!votes[i].count) continue;
        if (winner < 0 || votes[i].count > votes[winner].count ||
            (votes[i].count == votes[winner].count && votes[i].order > votes[winner].order)) winner = i;
    }
    if (winner >= 0 && lastDirection >= 0 && votes[lastDirection].count == votes[winner].count)
        winner = lastDirection; // Hoa phieu: uu tien huong da xuat, sau do huong moi nhat.
    if (winner < 0) Serial.println(PRINT_DETAILS ? "KHONG CO DU LIEU | valid=0" : "KHONG CO DU LIEU");
    else {
        Serial.print(NAMES[winner]);
        if (PRINT_DETAILS) Serial.printf(" | dA1=%.3f m | dA2=%.3f m | goc=%.1f do | valid=1",
                                        votes[winner].d1, votes[winner].d2, votes[winner].angle);
        Serial.println();
    }
    lastDirection = winner;
    clearVotes();
    voteStart += (elapsed / VOTE_MS) * VOTE_MS;
}
