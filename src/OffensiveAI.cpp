#include "OffensiveAI.h"

#include "Robot.h"
#include "Command.h"
#include "Dribbler.h"
#include "Coilgun.h"
#include "Util.h"

OffensiveAI::OffensiveAI(Robot* robot, Communication* com) : Controller(robot, com), targetSide(Side::UNKNOWN), currentState(NULL), running(false), totalDuration(0.0f), currentStateDuration(0.0f) {
	setupStates();
};

OffensiveAI::~OffensiveAI() {
	for (StatesIt it = states.begin(); it != states.end(); it++) {
		delete it->second;
	}

	states.clear();
}

void OffensiveAI::reset() {
	com->send("reset");
	targetSide = Side::UNKNOWN;
	totalDuration = 0.0f;
	currentStateDuration = 0.0f;

	setState("idle");
}

void OffensiveAI::onEnter() {
	reset();
}

void OffensiveAI::onExit() {
	reset();
}

void OffensiveAI::setState(std::string state) {
	if (states.find(state) == states.end()) {
		std::cout << "- Invalid state '" << state << "' requested" << std::endl;

		return;
	}

	State* newState = states[state];

	if (currentState != NULL) {
		currentState->onExit();
	}

	currentStateDuration = 0.0f;

	newState->onEnter();

	currentState = newState;
}

bool OffensiveAI::handleRequest(std::string request) {
	return false;
}

bool OffensiveAI::handleCommand(const Command& cmd) {
	return false;
}

void OffensiveAI::handleCommunicationMessage(std::string message) {

}

void OffensiveAI::step(float dt, Vision::Results* visionResults) {
	currentStateDuration += dt;
	totalDuration += dt;

	if (currentState != NULL) {
		currentState->step(dt, totalDuration, currentStateDuration);
	}
}

void OffensiveAI::setupStates() {
	states["idle"] = new IdleState(this);

	setState("idle");
}

std::string OffensiveAI::getJSON() {
	return "null";
}
