/*
  UWB ANCHOR 2 - ESP32 + BU01 (DW1000)
  ------------------------------------------------------------
  Muc tieu:
  - Bo HOAN TOAN WiFi/OTA de tranh chen xu ly khi dang debug UWB.
  - Khong dung range filter trong giai doan kiem tra sai so.
  - Antenna delay la gia tri rieng cua Anchor 2, tu dien sau khi cali.
  - Giu cach khoi dong giong huong JRemington:
      MODE_LONGDATA_RANGE_LOWPOWER
      randomShortAddress = false

  Anchor 2:
    EUI          = 87:17:5B:D5:A9:9A:E2:9C
    short address du kien = 0x1787
*/

#include <SPI.h>
#include "DW1000.h"
#include "DW1000Ranging.h"

// ============================================================
// 1. THONG SO CO DINH
// ============================================================

#define ANCHOR_ADD "87:17:5B:D5:A9:9A:E2:9C"

#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS  = 4;

// ============================================================
// 2. THONG SO BAN TU DIEN
// ============================================================
// Dien antenna delay da cali RIENG cho Anchor 2.
// De 0 thi code se KHONG cho UWB chay, tranh quen dien sai gia tri.
const uint16_t ANTENNA_DELAY = 16450;   // <-- DIEN GIA TRI CALI A2 O DAY

// ============================================================
// 3. KHAI BAO HAM
// ============================================================

void newRange();
void newBlink(DW1000Device *device);
void inactiveDevice(DW1000Device *device);

// ============================================================
// 4. SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("====================================");
    Serial.println(" UWB ANCHOR 2 - CLEAN / NO OTA");
    Serial.println(" short address expected: 0x1787");
    Serial.println("====================================");

    // Bat buoc dien antenna delay cua chinh Anchor 2.
    if (ANTENNA_DELAY == 0)
    {
        Serial.println("STOP: Hay dien ANTENNA_DELAY cua Anchor 2.");
        while (true)
        {
            delay(1000);
        }
    }

    // SPI ESP32 <-> BU01
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    // Khoi tao DW1000: Reset, CS, IRQ
    DW1000Ranging.initCommunication(
        PIN_RST,
        PIN_SS,
        PIN_IRQ
    );

    // Antenna delay RIENG cua Anchor 2
    DW1000.setAntennaDelay(ANTENNA_DELAY);

    // Callback cua DW1000Ranging
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachBlinkDevice(newBlink);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);

    /*
      QUAN TRONG:
      Khong bat EMA/filter trong giai doan nay.
      Ta can xem RAW range de biet loi nam o ranging/timing hay khong.
    */

    // Khoi dong thanh Anchor.
    // false = short address co dinh tu EUI, khong random.
    DW1000Ranging.startAsAnchor(
        ANCHOR_ADD,
        DW1000.MODE_SHORTDATA_FAST_LOWPOWER,
        false
    );

    Serial.print("ANTENNA_DELAY A2 = ");
    Serial.println(ANTENNA_DELAY);
    Serial.println("ANCHOR 2 READY");
}

// ============================================================
// 5. LOOP
// ============================================================

void loop()
{
    // Khong WiFi, khong OTA, khong delay trong loop.
    DW1000Ranging.loop();
}

// ============================================================
// 6. RANGE MOI
// ============================================================

void newRange()
{
    DW1000Device *device = DW1000Ranging.getDistantDevice();

    if (device == nullptr)
    {
        return;
    }

    Serial.print("A2_RAW,tag=0x");
    Serial.print(device->getShortAddress(), HEX);

    Serial.print(",range=");
    Serial.print(device->getRange(), 3);

    Serial.print(",rx=");
    Serial.println(device->getRXPower(), 1);
}

// ============================================================
// 7. PHAT HIEN TAG
// ============================================================

void newBlink(DW1000Device *device)
{
    Serial.print("A2_TAG_ADDED,0x");
    Serial.println(device->getShortAddress(), HEX);
}

// ============================================================
// 8. TAG MAT KET NOI
// ============================================================

void inactiveDevice(DW1000Device *device)
{
    Serial.print("A2_TAG_INACTIVE,0x");
    Serial.println(device->getShortAddress(), HEX);
}
