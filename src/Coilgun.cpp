#include "Coilgun.h"
#include "Communication.h"
#include "Util.h"
#include "Config.h"

#include <iostream>

Coilgun::Coilgun(Communication* com) : com(com), lastKickTime(0) {

};

Coilgun::~Coilgun() {

}

void Coilgun::charge() {
	com->send("charge");
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

void Coilgun::step(float dt) {
	
}