#ifndef DRIBBLER_H
#define DRIBBLER_H

#include "Config.h"
#include "Wheel.h"

class Dribbler : public Wheel {

public:
	Dribbler(int id);

	void start() { setTargetOmega(Config::robotDribblerNormalOmega); }
	void stop() { setTargetOmega(0); }
	bool isActive() const { return targetOmega > 0; }
	bool gotBall() const;
	bool handleCommand(const Command& cmd);
	double getBallInDribblerTime() { return ballInDribblerTime; }
	double getBallLostTime() { return ballLostTime; }
	void step(float dt);

private:
	int id;
	float targetOmega;
    float realOmega;
	int stallCounter;
	bool ballDetected;
	float ballInDribblerTime;
	float ballLostTime;

};

#endif //DRIBBLER_H