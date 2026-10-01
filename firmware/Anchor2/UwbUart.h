#ifndef UWB_UART_H
#define UWB_UART_H

#include <stdint.h>
#include <string.h>

// Identical copy in the two Anchor folders so each Arduino sketch opens alone.
namespace UwbLink {
constexpr uint16_t TAG_SHORT = 0x007D;
constexpr uint16_t A2_SHORT = 0x1787;
constexpr uint64_t POLL_MASK = (uint64_t(1) << 40) - 1;
constexpr uint8_t SAMPLE = 1;
constexpr uint8_t TAG_LOST = 2;
constexpr uint8_t FRAME_SIZE = 32;
constexpr uint32_t FRAME_TIMEOUT_MS = 20;
constexpr uint32_t WIRE_TIME_MS = 3; // 32 bytes, 8N1, 115200 baud, rounded up.

struct Frame {
    uint32_t bootId = 0;
    uint32_t sequence = 0;
    uint64_t pollStamp = 0;
    int32_t rangeMm = 0;
    int16_t rxDb10 = 0;
    uint16_t ageMs = 0; // A2 elapsed time from callback to enqueue, not millis().
    uint8_t kind = SAMPLE;
    bool valid = false;
};

inline bool newerSequence(uint32_t value, uint32_t previous) {
    const uint32_t delta = value - previous;
    return delta != 0 && delta < 0x80000000UL;
}

inline uint16_t crc16(const uint8_t *data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; ++i) {
        crc ^= uint16_t(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
    }
    return crc;
}

inline void putLE(uint8_t *dest, uint64_t value, uint8_t length) {
    for (uint8_t i = 0; i < length; ++i) dest[i] = uint8_t(value >> (8 * i));
}

inline uint64_t getLE(const uint8_t *src, uint8_t length) {
    uint64_t value = 0;
    for (uint8_t i = 0; i < length; ++i) value |= uint64_t(src[i]) << (8 * i);
    return value;
}

inline void encode(const Frame &frame, uint8_t *data) {
    data[0] = 0xB5;
    data[1] = 0x62;
    data[2] = 1;
    data[3] = frame.kind;
    putLE(data + 4, A2_SHORT, 2);
    putLE(data + 6, TAG_SHORT, 2);
    putLE(data + 8, frame.bootId, 4);
    putLE(data + 12, frame.sequence, 4);
    putLE(data + 16, frame.pollStamp & POLL_MASK, 5);
    data[21] = frame.valid ? 1 : 0;
    putLE(data + 22, uint32_t(frame.rangeMm), 4);
    putLE(data + 26, uint16_t(frame.rxDb10), 2);
    putLE(data + 28, frame.ageMs, 2);
    putLE(data + 30, crc16(data, 30), 2);
}

inline bool decode(const uint8_t *data, Frame &frame) {
    if (data[0] != 0xB5 || data[1] != 0x62 || data[2] != 1 ||
        (data[3] != SAMPLE && data[3] != TAG_LOST) ||
        getLE(data + 4, 2) != A2_SHORT || getLE(data + 6, 2) != TAG_SHORT ||
        data[21] > 1 || (data[3] == TAG_LOST && data[21] != 0) ||
        getLE(data + 30, 2) != crc16(data, 30)) return false;
    frame.kind = data[3];
    frame.bootId = uint32_t(getLE(data + 8, 4));
    frame.sequence = uint32_t(getLE(data + 12, 4));
    frame.pollStamp = getLE(data + 16, 5);
    frame.valid = data[21] != 0;
    frame.rangeMm = int32_t(uint32_t(getLE(data + 22, 4)));
    frame.rxDb10 = int16_t(uint16_t(getLE(data + 26, 2)));
    frame.ageMs = uint16_t(getLE(data + 28, 2));
    return true;
}

class Parser {
public:
    uint32_t badFrames = 0;
    uint32_t assemblyMs = 0;

    void reset() { used = 0; }

    bool feed(uint8_t value, uint32_t now, Frame &frame) {
        if (used && uint32_t(now - startedMs) > FRAME_TIMEOUT_MS) {
            ++badFrames;
            reset();
        }
        if (used == 0) {
            if (value == 0xB5) { buffer[used++] = value; startedMs = now; }
            return false;
        }
        if (used == 1 && value != 0x62) {
            if (value == 0xB5) startedMs = now;
            else reset();
            return false;
        }
        buffer[used++] = value;
        if (used != FRAME_SIZE) return false;
        if (decode(buffer, frame)) {
            assemblyMs = uint32_t(now - startedMs);
            reset();
            return true;
        }
        ++badFrames;
        // Recover after noise, a missing byte, or a false sync inside a payload.
        for (uint8_t i = 1; i < FRAME_SIZE - 1; ++i) {
            if (buffer[i] == 0xB5 && buffer[i + 1] == 0x62) {
                used = FRAME_SIZE - i;
                memmove(buffer, buffer + i, used);
                startedMs = now;
                return false;
            }
        }
        used = buffer[FRAME_SIZE - 1] == 0xB5 ? 1 : 0;
        if (used) { buffer[0] = 0xB5; startedMs = now; }
        return false;
    }

private:
    uint8_t buffer[FRAME_SIZE] = {};
    uint8_t used = 0;
    uint32_t startedMs = 0;
};
} // namespace UwbLink
#endif
