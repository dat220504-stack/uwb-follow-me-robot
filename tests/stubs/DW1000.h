#pragma once
#include "Arduino.h"
struct MockDW1000 {
    inline static const byte MODE_SHORTDATA_FAST_LOWPOWER[] = {1, 2, 3};
    uint16_t antennaDelay = 16384;
    void setAntennaDelay(uint16_t value) { antennaDelay = value; }
};
inline MockDW1000 DW1000;
