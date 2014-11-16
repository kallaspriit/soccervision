#include "Coilgun.h"
#include "AbstractCommunication.h"
#include "Util.h"
#include "Maths.h"
#include "Config.h"

#include <iostream>

Coilgun::Coilgun(AbstractCommunication* com) : com(com), lastKickTime(0.0), lastChargeRequestTime(0.0) {

};

Coilgun::~Coilgun() {

}

void Coilgun::charge() {
	com->send("charge");

	lastChargeRequestTime = Util::millitime();
}

void Coilgun::discharge() {
	com->send("discharge");
}

void Coilgun::kick(int microseconds) {
	if (lastKickTime == 0.0 || Util::duration(lastKickTime) >= Config::minKickInterval) {
		std::cout << "! Kicking: " << microseconds << std::endl;

		com->send("kick:" + Util::toString(microseconds));
	} else {
		std::cout << "! Another kick requested too soon" << std::endl;
	}
}

void Coilgun::chipKick(float distanceMeters) {
	float microseconds = getChipKickDurationByDistance(distanceMeters);

	std::cout << "! Chip-kicking to distance of " << distanceMeters << " for " << microseconds << " microseconds" << std::endl;

	com->send("dkick:0:0:" + Util::toString(microseconds) + ":0");
}

float Coilgun::getChipKickDurationByDistance(float distanceMeters) {
	// https://docs.google.com/spreadsheets/d/1ARV-47Fz91zb0l3KWDKGF0UaeSvyiEqHsW1mHZ_QlIk/edit#gid=275766663
	// y = 1.103E-7x^4 - 9.764E-5x^3 + 0.014x^2

	float distanceCm = distanceMeters * 10;

	float a = 1.103E-7f;
	float b = 9.764E-5f;
	float c = 0.014f;

	return Math::pow(a * distanceCm, 4) - Math::pow(b * distanceCm, 3) + Math::pow(c * distanceCm, 2);
}

void Coilgun::step(float dt) {
	/*if (Util::duration(lastChargeRequestTime) >= 1.0) {
		charge();
	}*/
}