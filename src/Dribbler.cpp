#include "Dribbler.h"
#include "Command.h"
#include "Util.h"

#include <iostream>

Dribbler::Dribbler() : speed(0), ballDetected(false), ballInDribblerTime(0.0), ballLostTime(-1.0) {

};

Dribbler::~Dribbler() {

}

void Dribbler::setSpeed(int newSpeed) {
	speed = newSpeed;
}

void Dribbler::stop() {
	speed = 0;
}

void Dribbler::step(float dt) {
	/*if (!serial->isOpen()) {
		return;
	}

	serial->writeln("gb\nwl" + Util::toString(speed));

	std::string message;

    while (serial->available() > 0) {
        message = serial->read();

        if (Command::isValid(message)) {
            Command cmd = Command::parse(message);

            if (cmd.name == "b") {
                ballDetected = cmd.params[0] == "1" ? true : false;
			}
		}
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
	}*/
}

bool Dribbler::gotBall() const {
	//return ballDetected;

	if (
		ballInDribblerTime >= Config::ballInDribblerThreshold
		&& ballLostTime <= Config::dribblerBallLostThreshold
	) {
		return true;
	}

	return false;
}
