#ifndef DRIBBLER_H
#define DRIBBLER_H

#include "Config.h"
#include "Command.h"

class Dribbler : public Command::Listener {

public:
	Dribbler();
	~Dribbler();

	void setSpeed(int speed = 255);
	void start() { setSpeed(Config::robotDribblerSpeed); }
	void stop();
	bool isActive() const { return speed > 0; }
	bool gotBall() const;
	bool isReady() { return false; } // TODO Implement connection
	bool handleCommand(const Command& cmd);
	double getBallInDribblerTime() { return ballInDribblerTime; }
	double getBallLostTime() { return ballLostTime; }
	void step(float dt);

private:
	int speed;
	bool ballDetected;
	float ballInDribblerTime;
	float ballLostTime;

};

#endif //DRIBBLER_H