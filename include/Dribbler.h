#ifndef DRIBBLER_H
#define DRIBBLER_H

#include "Config.h"

class Dribbler {

public:
	Dribbler();
	~Dribbler();

	void setSpeed(int speed = 255);
	void start() { setSpeed(Config::robotDribblerSpeed); }
	void stop();
	bool isActive() const { return speed > 0; }
	bool gotBall() const;
	bool isReady() { return false; } // TODO Implement connection
	double getBallInDribblerTime() { return ballInDribblerTime; }
	double getBallLostTime() { return ballLostTime; }
	void step(float dt);

private:
	int speed;
	bool ballDetected;
	double ballInDribblerTime;
	double ballLostTime;

};

#endif //DRIBBLER_H