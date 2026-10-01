#pragma once
#include "DW1000.h"
struct MockTimestamp {
    int64_t value = 0;
    int64_t getTimestamp() { return value; }
};
struct DW1000Device {
    uint16_t address = 0x007D;
    float range = 1.0f, power = -70.0f;
    int8_t index = 0;
    MockTimestamp timePollSent;
    uint16_t getShortAddress() { return address; }
    float getRange() { return range; }
    float getRXPower() { return power; }
    int8_t getIndex() { return index; }
};
struct MockRanging {
    DW1000Device device;
    std::vector<DW1000Device> network;
    void initCommunication(uint8_t, uint8_t, uint8_t) {}
    void attachNewRange(void (*)()) {}
    void attachBlinkDevice(void (*)(DW1000Device *)) {}
    void attachInactiveDevice(void (*)(DW1000Device *)) {}
    void attachNewDevice(void (*)(DW1000Device *)) {}
    void startAsAnchor(char *, const byte *, bool) {}
    void startAsTag(char *, const byte *, bool) {}
    void loop() {}
    DW1000Device *getDistantDevice() { return &device; }
    DW1000Device *searchDistantDevice(byte *address) {
        const uint16_t value = address[0] | uint16_t(address[1]) << 8;
        for (auto &node : network) if (node.address == value) return &node;
        return nullptr;
    }
    uint8_t getNetworkDevicesNumber() { return uint8_t(network.size()); }
    void removeNetworkDevices(int16_t index) {
        network.erase(network.begin() + index);
        for (size_t i = 0; i < network.size(); ++i) network[i].index = int8_t(i);
    }
};
inline MockRanging DW1000Ranging;
