/*
  TAG AUTO OFFSET A2 + KALMAN + VOTE 500 MS.
  GIU: chan noi, dia chi, SHORTDATA_FAST_LOWPOWER, L=ANCHOR_SPACING_M,
       antenna delay mac dinh, ghep cap 80 ms va hinh hoc.
  MOI LAN BAT/RESET: dat TAG dung yen chinh giua phia truoc A1/A2, cach tam 1 m.
  Cho du anchor dung thu tu, on dinh 2 s, lay >= 30 cap RAW trong >= 3 s.
  OFFSET_A2_M = trung binh(d1_raw) - trung binh(d2_raw), KHONG cong offset cu.
  Chi can bang hai ben; KHONG ep khoang cach ve 1 m hay thay antenna delay.
  CALIB_OK: da chot offset trong RAM, duoc phep di chuyen va bat dau vote.
  Mat/noi lai anchor sau CALIB_OK: van giu offset; khong tu cali khi dang di chuyen.
  THEM: A2 phai o index 0, A1 o index 1; Kalman GOC; vote huong moi 500 ms.
  KHONG sua file thu vien, khong them thu vien Kalman.
  Serial 115200: moi 500 ms in huong co nhieu phieu nhat.
  PRINT_DETAILS = true: kem dA1, dA2 va goc cua mau moi nhat thuoc huong thang vote.
  PRINT_DETAILS = false: chi in nhan huong, vi du "THANG".
  Hoa phieu: uu tien huong da xuat; neu khong, chon huong vua xuat hien gan nhat.
  Khong co mau hop le trong cua so: in "KHONG CO DU LIEU".
  Thu vien cu van co the in vai dong thong tin luc khoi dong.
  DEBUG_LOG = true de xem RAW / slot / ly do bo mau khi can.

  Quy uoc: A1 trai, A2 phai khi nhin tu robot ve phia truoc.
  Chi chon nghiem phia truoc: 2 anchor KHONG phan biet truoc/sau.
  Luu y: giu dung slot khong chung minh loi ranging da het.
  Chi quan ly danh sach tren TAG, khong chan song RANGING_INIT cua anchor.
  Chi cap range hop le moi duoc bo phieu. Doi danh sach anchor thi xoa phieu cu.
  Chi nap file nay cho TAG; giu hai anchor cung mode nhu ban dang chay.
  Logic duoc kiem tra bang C++ tren may tinh; CHUA bien dich ESP32/thu BU01 that.
  API tham chieu: thotro/arduino-dw1000/src/DW1000Ranging.{h,cpp}
*/

#include <SPI.h>
#include <math.h>
#include "DW1000.h"
#include "DW1000Ranging.h"

// ============================================================
// 1. DIA CHI
// ============================================================

char TAG_ADD[] = "7D:00:22:EA:82:60:3B:9C"; // Cung EUI, dung mang char cho API.

const uint16_t ANCHOR1_SHORT = 0x1786;
const uint16_t ANCHOR2_SHORT = 0x1787;

// ============================================================
// 2. ESP32 <-> BU01
// ============================================================

#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23
#define DW_CS      4

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;

// ============================================================
// 3. THONG SO UWB / DEBUG
// ============================================================

// JRemington dung Tag chung lam reference voi antenna delay MAC DINH cua library = 16384.
// Co y KHONG goi DW1000.setAntennaDelay() tren Tag de bam sat ESP32_UWB_setup_tag.ino.

// Day la cua so GHEP DU LIEU o cap sketch, KHONG phai reply slot cua DW1000.
// Bat dau voi 80 ms de tranh ghep range cua 2 vong qua xa nhau.
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
const bool DEBUG_LOG = false; // true: them RAW / PAIR / slot; false: chi ket qua vote.
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

// ============================================================
// 4. THONG SO BAN TU DIEN NEU MUON TINH GOC
// ============================================================
// Dien khoang cach THAT giua A1 va A2 (m).
// De 0.0f thi code VAN do d1/d2 va ghep PAIR,
// nhung CHUA tinh x/y/goc.
const float ANCHOR_SPACING_M = 0.5f;   // <-- DIEN L O DAY KHI CAN TINH GOC

// ============================================================
// 5. CAU TRUC LUU MOT MAU RANGE
// ============================================================

struct RangeSample
{
    float meters;
    float rxPower;
    uint32_t timeMs;
    uint16_t replyUs;
    int8_t deviceIndex;
    bool valid;
    bool fresh;
};

RangeSample anchor1 = {0.0f, 0.0f, 0, 0, -1, false, false};
RangeSample anchor2 = {0.0f, 0.0f, 0, 0, -1, false, false};

bool networkChanged = false;
bool rejectUnknown = false;
uint16_t unknownAddress = 0;
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

// ============================================================
// 6. KHAI BAO HAM
// ============================================================

void newRange();
void newDevice(DW1000Device *device);
void inactiveDevice(DW1000Device *device);

void tryMakePair();
void dropOldSingleSample();
void clearFreshPair();
void calculateGeometry(float d1, float d2);
DW1000Device *findAnchor(uint16_t address);
void removeAnchor(uint16_t address);
void enforceA2First();
bool correctAnchorOrder();
void resetMeasurements();
void updateFilteredDirection(float angleDeg, float d1, float d2);
void clearDirectionVotes();
void flushDirectionVotes(uint32_t now);
void addDirectionVote(int direction, float d1, float d2, float angleDeg, uint32_t now);
void resetStartupCalibration();
void collectCalibrationPair(float d1Raw, float d2Raw, uint32_t now);
void serviceStartupCalibration(uint32_t now);

// ============================================================
// 7. SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    if (DEBUG_LOG) Serial.println();
    if (DEBUG_LOG) Serial.println("==============================================");
    if (DEBUG_LOG) Serial.println(" UWB TAG - KALMAN ANGLE / A2 FIRST / STOCK LIBRARY");
    if (DEBUG_LOG) Serial.println(" A1 = 0x1786");
    if (DEBUG_LOG) Serial.println(" A2 = 0x1787");
    if (DEBUG_LOG) Serial.println("==============================================");

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    DW1000Ranging.initCommunication(
        PIN_RST,
        DW_CS,
        PIN_IRQ
    );

    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);

    // Giu nhu file goc: khong bat EMA cua DW1000Ranging.
    // Kalman se duoc ap dung SAU khi da tinh goc.

    DW1000Ranging.startAsTag(
        TAG_ADD,
        DW1000.MODE_SHORTDATA_FAST_LOWPOWER,
        false
    );

    if (DEBUG_LOG) Serial.println("TAG_ANTENNA_DELAY=library default (16384)");

    if (DEBUG_LOG) Serial.print("MAX_PAIR_SKEW_MS=");
    if (DEBUG_LOG) Serial.println(MAX_PAIR_SKEW_MS);

    if (ANCHOR_SPACING_M <= 0.0f)
    {
        if (DEBUG_LOG) Serial.println("GEOMETRY=OFF (chua dien ANCHOR_SPACING_M)");
    }
    else
    {
        if (DEBUG_LOG) Serial.print("GEOMETRY=ON,L=");
        if (DEBUG_LOG) Serial.println(ANCHOR_SPACING_M, 3);
    }

    voteWindowStartMs = millis();
    calibrationLastStatusMs = voteWindowStartMs;
    Serial.println("CALIB: DAT TAG DUNG YEN GIUA A1/A2, PHIA TRUOC CACH TAM 1 m.");
    Serial.println("CALIB: CHO CALIB_OK ROI MOI DI CHUYEN.");
}

// ============================================================
// 8. LOOP
// ============================================================

void loop()
{
    // Uu tien state machine cua DW1000Ranging.
    DW1000Ranging.loop();

    // Chi xoa khi thu vien da xu ly xong goi / vong kiem tra inactive.
    // Khong xoa truc tiep trong inactiveDevice(): thu vien se tu xoa tiep!
    if (networkChanged) enforceA2First();

    // Neu chi co 1 mau moi ma cho Anchor kia qua lau,
    // bo mau do. Khong mang sang vong ranging sau.
    dropOldSingleSample();

    const uint32_t now = millis();
    if (!offsetCalibrated) {
        serviceStartupCalibration(now);
    } else {
        // Chot cua so dung han, ke ca khi khong nhan them range.
        flushDirectionVotes(now);
    }
}

// ============================================================
// 9. NHAN RANGE MOI
// ============================================================

void newRange()
{
    // Khong xuat goc luc danh sach dang doi hoac chua dung A2 -> A1.
    if (networkChanged || !correctAnchorOrder()) return;
    DW1000Device *device = DW1000Ranging.getDistantDevice();

    if (device == nullptr)
    {
        return;
    }

    const uint16_t address = device->getShortAddress();
    float range = device->getRange();
    // Luc cali luon dung RAW. Chi bu A2 sau khi da chot offset.
    if (address == ANCHOR2_SHORT && offsetCalibrated)
        range += OFFSET_A2_M;

    const float rx = device->getRXPower();

    // Rat quan trong de debug chia luot cua DW1000Ranging.
    const int8_t deviceIndex = device->getIndex();
    const uint16_t replyUs = device->getReplyTime();

    // Reply time chi de debug; khong loai mau theo slot 7000/21000.

    // Chi quan tam A1 va A2.
    RangeSample *sample = nullptr;
    const char *name = nullptr;

    if (address == ANCHOR1_SHORT)
    {
        sample = &anchor1;
        name = "A1";
    }
    else if (address == ANCHOR2_SHORT)
    {
        sample = &anchor2;
        name = "A2";
    }
    else
    {
        if (DEBUG_LOG) Serial.print("UNKNOWN,addr=0x");
        if (DEBUG_LOG) Serial.println(address, HEX);
        return;
    }

    // In ca range loi de xem no dang nam o slot nao.
    if (!isfinite(range) || range <= 0.0f || range > MAX_VALID_RANGE_M)
    {
        if (DEBUG_LOG) Serial.print("BAD,");
        if (DEBUG_LOG) Serial.print(name);
        if (DEBUG_LOG) Serial.print(",addr=0x");
        if (DEBUG_LOG) Serial.print(address, HEX);
        if (DEBUG_LOG) Serial.print(",range=");
        if (DEBUG_LOG) Serial.print(range, 3);
        if (DEBUG_LOG) Serial.print(",rx=");
        if (DEBUG_LOG) Serial.print(rx, 1);
        if (DEBUG_LOG) Serial.print(",index=");
        if (DEBUG_LOG) Serial.print(deviceIndex);
        if (DEBUG_LOG) Serial.print(",reply_us=");
        if (DEBUG_LOG) Serial.println(replyUs);

        // Bo ca cap khi nhan range loi, tranh giu lai mau truoc do.
        clearFreshPair();
        return;
    }

    // Truoc CALIB_OK: RAW. Sau CALIB_OK: A2 da duoc cong offset mot lan.
    sample->meters = range;
    sample->rxPower = rx;
    sample->timeMs = millis();
    sample->replyUs = replyUs;
    sample->deviceIndex = deviceIndex;
    sample->valid = true;
    sample->fresh = true;

    // Log ngan gon, khong in qua dai de giam thoi gian Serial.
    if (DEBUG_LOG) Serial.print(offsetCalibrated ? "RANGE," : "RAW,");
    if (DEBUG_LOG) Serial.print(name);
    if (DEBUG_LOG) Serial.print(",range=");
    if (DEBUG_LOG) Serial.print(range, 3);
    if (DEBUG_LOG) Serial.print(",rx=");
    if (DEBUG_LOG) Serial.print(rx, 1);
    if (DEBUG_LOG) Serial.print(",index=");
    if (DEBUG_LOG) Serial.print(deviceIndex);
    if (DEBUG_LOG) Serial.print(",reply_us=");
    if (DEBUG_LOG) Serial.println(replyUs);

    // Thu ghep A1 + A2.
    tryMakePair();
}

// ============================================================
// 10. GHEP HAI RANGE A1/A2
// ============================================================

void tryMakePair()
{
    // Phai co 1 mau MOI tu moi Anchor.
    if (!anchor1.valid || !anchor2.valid ||
        !anchor1.fresh || !anchor2.fresh)
    {
        return;
    }

    const int32_t signedSkew =
        (int32_t)(anchor1.timeMs - anchor2.timeMs);

    const uint32_t skewMs =
        (signedSkew < 0)
            ? (uint32_t)(-signedSkew)
            : (uint32_t)signedSkew;

    // Neu 2 mau cach nhau qua xa:
    // BO CA HAI, khong giu mot mau cu de ghep voi vong tiep theo.
    if (skewMs > MAX_PAIR_SKEW_MS)
    {
        if (DEBUG_LOG) Serial.print("DROP_PAIR,skew_ms=");
        if (DEBUG_LOG) Serial.print(skewMs);
        if (DEBUG_LOG) Serial.print(",A1_reply_us=");
        if (DEBUG_LOG) Serial.print(anchor1.replyUs);
        if (DEBUG_LOG) Serial.print(",A2_reply_us=");
        if (DEBUG_LOG) Serial.println(anchor2.replyUs);

        clearFreshPair();
        return;
    }

    const float d1 = anchor1.meters;
    const float d2 = anchor2.meters;

    // Copy xong roi danh dau cap nay da duoc dung.
    clearFreshPair();

    if (DEBUG_LOG) Serial.print("PAIR,d1=");
    if (DEBUG_LOG) Serial.print(d1, 3);
    if (DEBUG_LOG) Serial.print(",d2=");
    if (DEBUG_LOG) Serial.print(d2, 3);
    if (DEBUG_LOG) Serial.print(",skew_ms=");
    if (DEBUG_LOG) Serial.print(skewMs);
    if (DEBUG_LOG) Serial.print(",A1_reply_us=");
    if (DEBUG_LOG) Serial.print(anchor1.replyUs);
    if (DEBUG_LOG) Serial.print(",A2_reply_us=");
    if (DEBUG_LOG) Serial.println(anchor2.replyUs);

    // Cali TRUOC phep kiem tra tam giac: RAW chua bu co the vi pham tam giac.
    if (!offsetCalibrated) {
        collectCalibrationPair(d1, d2, millis());
        return;
    }

    // Chi tinh goc khi ban da dien L.
    if (ANCHOR_SPACING_M > 0.0f)
    {
        calculateGeometry(d1, d2);
    }
}

// ============================================================
// 11. BO MAU LE QUA CU
// ============================================================

void dropOldSingleSample()
{
    const uint32_t now = millis();

    // Chi A1 dang fresh, A2 chua co mau moi.
    if (anchor1.fresh && !anchor2.fresh)
    {
        if ((uint32_t)(now - anchor1.timeMs) > MAX_PAIR_SKEW_MS)
        {
            if (DEBUG_LOG) Serial.println("DROP_SINGLE,A1");
            anchor1.fresh = false;
        }
    }

    // Chi A2 dang fresh, A1 chua co mau moi.
    if (anchor2.fresh && !anchor1.fresh)
    {
        if ((uint32_t)(now - anchor2.timeMs) > MAX_PAIR_SKEW_MS)
        {
            if (DEBUG_LOG) Serial.println("DROP_SINGLE,A2");
            anchor2.fresh = false;
        }
    }
}

// ============================================================
// 12. XOA CO FRESH CUA CA CAP
// ============================================================

void clearFreshPair()
{
    anchor1.fresh = false;
    anchor2.fresh = false;
}

// ============================================================
// 13. TINH VI TRI / GOC - CHI KHI DA DIEN L
// ============================================================

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

// ============================================================
// 14. PHAT HIEN ANCHOR MOI
// ============================================================

void newDevice(DW1000Device *device)
{
    if (device == nullptr) return;
    const uint16_t address = device->getShortAddress();
    networkChanged = true;
    if (address != ANCHOR1_SHORT && address != ANCHOR2_SHORT) {
        rejectUnknown = true;
        unknownAddress = address;
    }

    // device o callback nay co the la ban TAM, index chua duoc gan.
    // Chi ghi dia chi; ham enforceA2First() se tim ban trong danh sach that.
    if (DEBUG_LOG) Serial.print("ANCHOR_ADDED,addr=0x");
    if (DEBUG_LOG) Serial.println(address, HEX);
}

// ============================================================
// 15. ANCHOR MAT KET NOI
// ============================================================

void inactiveDevice(DW1000Device *device)
{
    if (device == nullptr)
    {
        return;
    }

    const uint16_t address = device->getShortAddress();
    networkChanged = true;
    resetMeasurements(); // Mat anchor: bo ket qua cu va khoi tao lai Kalman.

    if (address == ANCHOR1_SHORT)
    {
        anchor1.valid = false;
        anchor1.fresh = false;
    }
    else if (address == ANCHOR2_SHORT)
    {
        anchor2.valid = false;
        anchor2.fresh = false;
    }

    if (DEBUG_LOG) Serial.print("ANCHOR_INACTIVE,addr=0x");
    if (DEBUG_LOG) Serial.println(address, HEX);
}

// ============================================================
// 16. A2 VAO TRUOC - CHI DUNG API PUBLIC CUA THU VIEN CU
// ============================================================

DW1000Device *findAnchor(uint16_t address)
{
    // Thu vien luu short address theo thu tu byte thap -> byte cao.
    byte bytes[2] = {byte(address & 0xFF), byte(address >> 8)};
    return DW1000Ranging.searchDistantDevice(bytes);
}

void removeAnchor(uint16_t address)
{
    DW1000Device *stored = findAnchor(address);
    if (stored == nullptr) return;
    const int16_t index = stored->getIndex();
    if (index < 0 || index >= DW1000Ranging.getNetworkDevicesNumber()) return;

    if (DEBUG_LOG) Serial.print("REMOVE,addr=0x");
    if (DEBUG_LOG) Serial.println(address, HEX);
    DW1000Ranging.removeNetworkDevices(index);
    // Khong dung lai stored: xoa lam doi vi tri cac phan tu trong mang.
}

void resetMeasurements()
{
    anchor1.valid = false;
    anchor2.valid = false;
    clearFreshPair();
    filterReady = false;
    clearDirectionVotes();
    lastVotedDirection = -1;
    // Doi anchor trong luc cali: lay lai tu dau. Da chot thi giu nguyen offset.
    if (!offsetCalibrated) resetStartupCalibration();
}

void enforceA2First()
{
    networkChanged = false;
    if (rejectUnknown) {
        removeAnchor(unknownAddress);
        rejectUnknown = false;
    }

    DW1000Device *a2 = findAnchor(ANCHOR2_SHORT);
    // Chua co A2, hoac A2 khong o index 0: loai A1 va cho no vao lai.
    if (a2 == nullptr || a2->getIndex() != 0) {
        removeAnchor(ANCHOR1_SHORT);
    }
    // Neu A2 bi thu vien xoa do timeout, A1 cung bi loai o day.
    // A1 se duoc thu vien nhan lai qua thu tuc BLINK/RANGING_INIT.
    // Viec nhan lai co the mat vai giay; khong reset ESP32 lien tuc.
    resetMeasurements();
}

bool correctAnchorOrder()
{
    DW1000Device *a2 = findAnchor(ANCHOR2_SHORT);
    DW1000Device *a1 = findAnchor(ANCHOR1_SHORT);
    return a2 != nullptr && a1 != nullptr &&
           a2->getIndex() == 0 && a1->getIndex() == 1;
}

// ============================================================
// 17. KALMAN 1 CHIEU + NHAN HUONG (KHONG CAN THU VIEN THEM)
// ============================================================

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

// ============================================================
// 18. VOTE THEO CAC CUA SO 500 MS KHONG CHONG LEN NHAU
// ============================================================

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
        Serial.println("KHONG CO DU LIEU");
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
            Serial.print(" do");
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

// ============================================================
// 19. TU CAN OFFSET A2 MOT LAN LUC KHOI DONG - KHONG CHAN VONG DO
// ============================================================

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
    if (!correctAnchorOrder()) {
        Serial.println("CALIB_CHO_A2_A1: CHUA DU HAI ANCHOR DUNG THU TU.");
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
