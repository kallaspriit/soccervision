#ifndef DRIBBLER_H
#define DRIBBLER_H

#include "Config.h"
#include "Wheel.h"

class Dribbler : public Wheel {

public:
	Dribbler(int id);

	void start();
	void stop();
	void onKick();
	bool isActive() const { return targetOmega > 0; }
	bool gotBall() const;
	bool handleCommand(const Command& cmd);
	double getBallInDribblerTime() { return ballInDribblerTime; }
	double getBallLostTime() { return ballLostTime; }
	void step(float dt);

private:
	bool ballDetected;
	bool everDetectedBall;
	float ballInDribblerTime;
	float ballLostTime;
	double stopRequestedTime;

};

#endif //DRIBBLER_H