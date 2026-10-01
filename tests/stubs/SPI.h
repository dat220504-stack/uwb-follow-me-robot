#pragma once
#include "Arduino.h"
struct SPIClass { void begin(int, int, int) {} };
inline SPIClass SPI;
