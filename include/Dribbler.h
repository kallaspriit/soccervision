#ifndef DRIBBLER_H
#define DRIBBLER_H

#include "Config.h"
#include "Wheel.h"

class AbstractCommunication;

class Dribbler : public Wheel {

public:
	Dribbler(int id, AbstractCommunication* com);

	void prime();
	void start();
	void stop();
	void onKick();
	bool isActive() const { return targetOmega > 0; }
	bool gotBall(bool definitive = false) const;
	bool handleCommand(const Command& cmd);
	double getBallInDribblerTime() { return ballInDribblerTime; }
	double getBallLostTime() { return ballLostTime; }
	int getLowerLimit() { return lowerLimit; }
	int getUpperLimit() { return upperLimit; }
	void setLowerLimit(int limit);
	void setUpperLimit(int limit);
	void setLimits(int lower, int upper);
	void useNormalLimits();
	void useChipKickLimits();
	void step(float dt);

private:
	void applyLimits();
	
	AbstractCommunication* com;
	bool ballDetected;
	bool everDetectedBall;
	float ballInDribblerTime;
	float ballLostTime;
	double stopRequestedTime;
	int lowerLimit;
	int upperLimit;

};

#endif //DRIBBLER_H