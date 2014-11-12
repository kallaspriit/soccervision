#include "Dribbler.h"
#include "AbstractCommunication.h"
#include "Command.h"
#include "Util.h"
#include "Maths.h"

#include <iostream>

Dribbler::Dribbler(int id, AbstractCommunication* com) : Wheel(id), com(com), ballDetected(false), everDetectedBall(false), ballInDribblerTime(0.0), ballLostTime(-1.0f), stopRequestedTime(-1.0), lowerLimit(Config::robotDribblerLowerLimit), upperLimit(Config::robotDribblerUpperLimit) {
	applyLimits();
};

void Dribbler::prime() {
	setTargetSpeed(-Config::robotDribblerSpeed);
	//setTargetSpeed(-Config::robotDribblerSpeed / 2);

	stopRequestedTime = -1.0;
}

void Dribbler::start() {
	setTargetSpeed(-Config::robotDribblerSpeed);

	stopRequestedTime = -1.0;
}

void Dribbler::stop() {
	if (stopRequestedTime == -1.0) {
		stopRequestedTime = Util::millitime();
	}
}

void Dribbler::onKick() {
	ballLostTime = Config::dribblerBallLostThreshold; // make it large so the ball is not faked after kick
	ballDetected = false;
}

void Dribbler::setLowerLimit(int limit) {
	lowerLimit = (int)Util::limit((float)limit, 0.0f, 100.0f);

	applyLimits();
}

void Dribbler::setUpperLimit(int limit) {
	upperLimit = (int)Util::limit((float)limit, 0.0f, 100.0f);

	applyLimits();
}

void Dribbler::setLimits(int lower, int upper) {
	lowerLimit = (int)Util::limit((float)lower, 0.0f, 100.0f);
	upperLimit = (int)Util::limit((float)upper, 0.0f, 100.0f);

	applyLimits();
}

void Dribbler::applyLimits() {
	float servoLimitLower = Math::map((float)lowerLimit, 0.0f, 100.0f, Config::robotDribblerLimitMin, Config::robotDribblerLimitMax);
	float servoLimitUpper = Config::robotDribblerLimitMin + Config::robotDribblerLimitMax - Math::map((float)upperLimit, 0.0f, 100.0f, Config::robotDribblerLimitMin, Config::robotDribblerLimitMax);

	std::cout << "! Setting servo limits to " << lowerLimit << "-" << upperLimit << " (" << servoLimitLower << "-" << servoLimitUpper << ")" << std::endl;

	com->send("servos:" + Util::toString(servoLimitLower) + ":" + Util::toString(servoLimitUpper));
}
void Dribbler::step(float dt) {
	Wheel::step(dt);

	double delayStopPeriod = 0.1;

	if (stopRequestedTime != -1.0 && Util::duration(stopRequestedTime) >= delayStopPeriod) {
		setTargetOmega(0);
		//setTargetOmega(-Config::robotDribblerSpeed / 5);

		stopRequestedTime = -1.0;
	}

	if (ballDetected) {
		ballInDribblerTime += dt;

		if (ballInDribblerTime >= Config::ballInDribblerThreshold) {
			ballLostTime = -1.0f;
		}
	} else {
		if (everDetectedBall) {
			if (ballLostTime == -1.0f) {
				ballLostTime = dt;
			} else {
				ballLostTime += dt;
			}
		}

		if (ballLostTime >= Config::dribblerBallLostThreshold) {
			ballInDribblerTime = 0.0f;
		}
	}

	//std::cout << "ballInDribblerTime: " << ballInDribblerTime << ", ballLostTime: " << ballLostTime << ", got ball: " << (gotBall() ? "yes" : "no") << std::endl;
}

bool Dribbler::gotBall(bool definitive) const {
	if (!definitive && !ballDetected && ballLostTime != -1.0f && ballLostTime < Config::dribblerBallLostThreshold) {
		//std::cout << "! Faking got ball, actually lost for: " << ballLostTime << std::endl;

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
			everDetectedBall = true;
		} else {
			ballDetected = false;
		}

		return true;
	}

	return false;
}
