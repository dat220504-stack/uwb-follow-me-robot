#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cmath>
#include <deque>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <type_traits>
using byte = uint8_t;
constexpr int HEX = 16, SERIAL_8N1 = 0;
constexpr float PI = 3.14159265358979323846f;
inline uint32_t fakeNow = 0;
inline uint32_t millis() { return fakeNow; }
inline void delay(uint32_t ms) { fakeNow += ms; }
using std::isfinite;

class HardwareSerial {
public:
    explicit HardwareSerial(int = 0) {}
    std::deque<uint8_t> rx;
    std::vector<uint8_t> tx;
    std::string output;
    int txSpace = 128;
    void begin(uint32_t, int = 0, int = -1, int = -1) {}
    size_t setRxBufferSize(size_t size) { return size; }
    size_t setTxBufferSize(size_t size) { return size; }
    int available() { return int(rx.size()); }
    int availableForWrite() { return txSpace; }
    int read() { if (rx.empty()) return -1; auto value = rx.front(); rx.pop_front(); return value; }
    size_t write(const uint8_t *data, size_t size) {
        tx.insert(tx.end(), data, data + size); return size;
    }
    void print(const char *text) { output += text; }
    size_t printf(const char *format, ...) {
        char text[512];
        va_list args;
        va_start(args, format);
        const int size = std::vsnprintf(text, sizeof(text), format, args);
        va_end(args);
        if (size <= 0) return 0;
        output += text;
        return size_t(size);
    }
    template<class T> void print(T value, int format = 0) {
        std::ostringstream stream;
        if constexpr (std::is_floating_point_v<T>) stream << std::fixed << std::setprecision(format);
        else if (format == HEX) stream << std::hex;
        if constexpr (std::is_same_v<T, uint8_t> || std::is_same_v<T, int8_t>) stream << int(value);
        else stream << value;
        output += stream.str();
    }
    void println() { output += '\n'; }
    template<class T> void println(T value, int format = 0) { print(value, format); println(); }
};
inline HardwareSerial Serial;
