#include "Dribbler.h"
#include "Command.h"
#include "Util.h"

#include <iostream>

Dribbler::Dribbler(int id) : Wheel(id), ballDetected(false), ballInDribblerTime(0.0), ballLostTime(-1.0f), stopRequestedTime(-1.0) {

};

void Dribbler::start() {
	setTargetSpeed(-Config::robotDribblerSpeed);

	stopRequestedTime = -1.0;
}

void Dribbler::stop() {
	if (stopRequestedTime == -1.0) {
		stopRequestedTime = Util::millitime();
	}
}

void Dribbler::step(float dt) {
	Wheel::step(dt);

	double delayStopPeriod = 1.0;

	if (stopRequestedTime != -1.0 && Util::duration(stopRequestedTime) >= delayStopPeriod) {
		setTargetOmega(0);

		stopRequestedTime = -1.0;
	}

	if (ballDetected) {
		ballInDribblerTime += dt;

		if (ballInDribblerTime >= Config::ballInDribblerThreshold) {
			ballLostTime = 0.0;
		}
	} else {
		ballLostTime += dt;

		if (ballLostTime >= Config::dribblerBallLostThreshold) {
			ballInDribblerTime = 0.0;
		}
	}

	//std::cout << "ballInDribblerTime: " << ballInDribblerTime << ", ballLostTime: " << ballLostTime << ", got ball: " << (gotBall() ? "yes" : "no") << std::endl;
}

bool Dribbler::gotBall() const {
	if (!ballDetected && ballLostTime <= Config::dribblerBallLostThreshold) {
		return true;
	}

	return ballDetected;

	/*if (ballDetected && ballInDribblerTime >= Config::ballInDribblerThreshold) {
		return true;
	}

	if (!ballDetected && ballLostTime <= Config::dribblerBallLostThreshold) {
		return true;
	}

	return false;*/
}

bool Dribbler::handleCommand(const Command& cmd) {
	Wheel::handleCommand(cmd);

	if (cmd.name == "ball") {
		if (cmd.parameters[0] == "1") {
			ballDetected = true;
		} else {
			ballDetected = false;
		}

		return true;
	}

	return false;
}
