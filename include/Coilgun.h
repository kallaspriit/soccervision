#ifndef COILGUN_H
#define COILGUN_H

#include "Config.h"

class Communication;

class Coilgun {

public:
	Coilgun(Communication* com);
	~Coilgun();

	void kick(int microseconds = Config::robotDefaultKickStrength);
	void charge();
	void discharge();
	bool isReady() { return false; } // TODO New communication
	void step(float dt);

private:
	Communication* com;
	double lastKickTime;
	double lastChargeRequestTime;

};

#endif //COILGUN_H