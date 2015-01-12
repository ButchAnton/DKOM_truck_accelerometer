#include "OSEPPShield.h"

OSEPPShield::OSEPPShield() {
	_address = SHIELD_ADDR_OFF_OFF_OFF;
}

OSEPPShield::OSEPPShield(bool dip0, bool dip1, bool dip2) {
	_address = SHIELD_ADDR_OFF_OFF_OFF | (dip0 << 2) | (dip1 << 1) | dip2;
}

OSEPPShield::OSEPPShield(uint8_t addr) {
	_address = addr;
}

void OSEPPShield::select_port(int port) {
	_port = port;
    Wire.beginTransmission(_address);
    Wire.write(1 << _port);
    Wire.endTransmission();
}

int OSEPPShield::get_port() { return _port; }
