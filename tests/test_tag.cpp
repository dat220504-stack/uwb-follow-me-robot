#include <cassert>
#include <iostream>
#include "stubs/DW1000Ranging.h"
// Arduino tu khai bao ham; C++ tren may can khai bao truoc.
void newDevice(DW1000Device *device);
void inactiveDevice(DW1000Device *device);
DW1000Device *findAnchor(uint16_t address);
void removeAnchor(uint16_t address);
void enforceA2First();
#include "../firmware/Tag/Tag.ino"
bool correctAnchorOrder() {
    auto *a2 = findAnchor(ANCHOR2_SHORT);
    auto *a1 = findAnchor(ANCHOR1_SHORT);
    return a2 && a1 && a2->getIndex() == 0 && a1->getIndex() == 1;
}
void add(uint16_t address) {
    DW1000Device node; node.address = address;
    node.index = int8_t(DW1000Ranging.network.size());
    DW1000Ranging.network.push_back(node);
    newDevice(&DW1000Ranging.network.back());
}
int main() {
    add(ANCHOR1_SHORT); loop();
    assert(DW1000Ranging.network.empty());
    add(ANCHOR1_SHORT); add(ANCHOR2_SHORT);
    assert(DW1000Ranging.network.size() == 2); // No deletion in callbacks.
    loop(); assert(DW1000Ranging.network.size() == 1);
    assert(findAnchor(ANCHOR2_SHORT)->getIndex() == 0);
    add(ANCHOR1_SHORT); loop(); assert(correctAnchorOrder());
    add(0xCAFE); loop(); assert(DW1000Ranging.network.size() == 2 && correctAnchorOrder());
    inactiveDevice(findAnchor(ANCHOR2_SHORT));
    DW1000Ranging.removeNetworkDevices(0); // Simulate library's deferred deletion.
    loop(); assert(DW1000Ranging.network.empty());
    assert(DW1000.antennaDelay == 16384);
    std::cout << "Tag: A2 first, unknown rejection, reconnect, callback deferral PASS\n";
}
