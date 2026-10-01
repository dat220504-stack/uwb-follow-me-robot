/*
  Tag chi quan ly hai Anchor: A2 vao truoc, A1 vao sau.
  Tu can offset, tinh goc, Kalman va vote 500 ms da chuyen sang A1.
  Giu cau hinh UWB goc; khong sua thu vien hay ep reply slot.
*/
#include <SPI.h>
#include "DW1000.h"
#include "DW1000Ranging.h"

// Cau hinh goc cua Tag.
char TAG_ADD[] = "7D:00:22:EA:82:60:3B:9C";
const uint16_t ANCHOR1_SHORT = 0x1786;
const uint16_t ANCHOR2_SHORT = 0x1787;
constexpr uint8_t SPI_SCK = 18, SPI_MISO = 19, SPI_MOSI = 23;
constexpr uint8_t PIN_RST = 27, PIN_IRQ = 34, DW_CS = 4;
const bool DEBUG_LOG = false;
bool networkChanged = false, rejectUnknown = false;
uint16_t unknownAddress = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, DW_CS, PIN_IRQ);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    // Khong setAntennaDelay(): Tag giu mac dinh 16384 cua thu vien.
    DW1000Ranging.startAsTag(TAG_ADD, DW1000.MODE_SHORTDATA_FAST_LOWPOWER, false);
    Serial.println("TAG READY | A2 FIRST | PROCESSING AT A1");
}

void loop() {
    DW1000Ranging.loop();
    // Chi xoa Anchor sau khi thu vien xu ly xong, khong xoa trong callback.
    if (networkChanged) enforceA2First();
}

void newDevice(DW1000Device *device) {
    if (!device) return;
    const uint16_t address = device->getShortAddress();
    networkChanged = true;
    if (address != ANCHOR1_SHORT && address != ANCHOR2_SHORT) {
        rejectUnknown = true;
        unknownAddress = address;
    }
}

void inactiveDevice(DW1000Device *device) {
    if (device) networkChanged = true;
}

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
    // A1 se tu noi lai sau A2 qua BLINK/RANGING_INIT.
}
