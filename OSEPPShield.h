// Based on example sketch at:
// http://osepp.com/learning-centre/start-here/i2c-expansion-shield/

#include "Wire.h"

#define SHIELD_ADDR_OFF_OFF_OFF  (0x70)
#define SHIELD_ADDR_OFF_OFF_ON   (0x74)
#define SHIELD_ADDR_OFF_ON_OFF   (0x72)
#define SHIELD_ADDR_OFF_ON_ON    (0x76)
#define SHIELD_ADDR_ON_OFF_OFF   (0x71)
#define SHIELD_ADDR_ON_OFF_ON    (0x75)
#define SHIELD_ADDR_ON_ON_OFF    (0x73)
#define SHIELD_ADDR_ON_ON_ON     (0x77)

#ifndef _OSHEPPSHIELD_H_
#define _OSHEPPSHIELD_H_

class OSEPPShield {
public:
	OSEPPShield();
	OSEPPShield(uint8_t address);
	OSEPPShield(bool dip0, bool dip1, bool dip2);

	void select_port(int port);
	int get_port();

private:
	int _port;
	uint8_t _address;
};

#endif
