#ifndef COILGUN_H
#define COILGUN_H

#include "Config.h"

class AbstractCommunication;
class Command;

class Coilgun {

public:
	Coilgun(AbstractCommunication* com);
	~Coilgun();

	void kick(int microseconds = Config::robotDefaultKickStrength);
	void chipKick(float distance);
	void charge();
	void discharge();
	bool isReady() { return false; } // TODO New communication
	bool isLowVoltage() { return voltage < Config::robotCoilgunLowVoltageThreshold;  }
	float getChipKickDurationByDistance(float distance);
	float getVoltage() { return voltage; }
	bool handleCommand(const Command& cmd);
	void step(float dt);

private:
	void requestVoltageReading();

	AbstractCommunication* com;
	double lastKickTime;
	double lastChargeRequestTime;
	float timeSinceLastVoltageReading;
	float voltage;

};

#endif //COILGUN_H