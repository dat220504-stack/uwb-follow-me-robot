#include <cassert>
#include <iostream>
#include "../firmware/Tag/Tag.ino"
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
