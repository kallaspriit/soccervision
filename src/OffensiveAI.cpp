#include "OffensiveAI.h"

#include "Robot.h"
#include "Command.h"
#include "Dribbler.h"
#include "Coilgun.h"
#include "Util.h"

OffensiveAI::OffensiveAI(Robot* robot, Communication* com) : Controller(robot, com), targetSide(Side::UNKNOWN), currentState(NULL), running(false), totalDuration(0.0f), currentStateDuration(0.0f) {
	startStateName = "idle";
	
	setupStates();
};

OffensiveAI::~OffensiveAI() {
	for (StatesIt it = states.begin(); it != states.end(); it++) {
		delete it->second;
	}

	states.clear();
}

void OffensiveAI::reset() {
	std::cout << "! Reset offensive AI" << std::endl;

	com->send("reset");
	targetSide = Side::UNKNOWN;
	totalDuration = 0.0f;
	currentStateDuration = 0.0f;
	currentState = NULL;
	currentStateName = "";
}

void OffensiveAI::onEnter() {
	std::cout << "! Now using offensive AI algorithm" << std::endl;

	reset();
}

void OffensiveAI::onExit() {
	std::cout << "! Stopping offensive AI algorithm" << std::endl;

	reset();
}

void OffensiveAI::setState(std::string state) {
	if (states.find(state) == states.end()) {
		std::cout << "- Invalid state '" << state << "' requested" << std::endl;

		return;
	}

	State* newState = states[state];

	if (currentState != NULL) {
		std::cout << "! Switched offensive AI state from " << currentStateName << " to " << state << std::endl;

		currentState->onExit();
	} else {
		std::cout << "! Set initial offensive AI state to " << state << std::endl;
	}

	currentStateDuration = 0.0f;

	newState->onEnter();

	currentState = newState;
	currentStateName = state;
}

bool OffensiveAI::handleRequest(std::string request) {
	return false;
}

bool OffensiveAI::handleCommand(const Command& cmd) {
	if (cmd.name == "toggle-side") {
        handleToggleSideCommand();
    } else if (cmd.name == "toggle-go") {
        handleToggleGoCommand();
    } else {
		return false;
	}

	return true;
}

void OffensiveAI::handleToggleSideCommand() {
	if (!toggleSideBtn.toggle()) {
		return;
	}

	if (!running) {
		if (targetSide == Side::BLUE) {
			targetSide = Side::YELLOW;
		} else {
			targetSide = Side::BLUE;
		}

		std::cout << "! Now targeting " << (targetSide == Side::BLUE ? "blue" : "yellow") << " side" << std::endl;

		com->send("target:" + Util::toString(targetSide));
	} else {
		std::cout << "- Side can't be changed while running" << std::endl;
	}
}

void OffensiveAI::handleToggleGoCommand() {
	if (!toggleGoBtn.toggle()) {
		return;
	}

	if (running || targetSide != Side::UNKNOWN) {
		running = !running;
	} else {
		std::cout << "- Choose target side before requesting start" << std::endl;
	}
}

void OffensiveAI::handleCommunicationMessage(std::string message) {
	if (Command::isValid(message)) {
        Command command = Command::parse(message);

		handleCommand(command);
	}
}

void OffensiveAI::step(float dt, Vision::Results* visionResults) {
	if (!running && currentStateName != startStateName) {
		setState(startStateName);
	}

	currentStateDuration += dt;
	totalDuration += dt;

	if (currentState == NULL) {
		setState(startStateName);
	}

	if (currentState != NULL) {
		currentState->step(dt, totalDuration, currentStateDuration);
	}
}

void OffensiveAI::setupStates() {
	states["idle"] = new IdleState(this);
	states["find-ball"] = new FindBallState(this);
}

std::string OffensiveAI::getJSON() {
	std::stringstream stream;

	stream << "{";
	stream << "\"Target side\": \"" << (targetSide == Side::BLUE ? "blue" : targetSide == Side::YELLOW ? "yellow" : "not chosen") << "\",";
	stream << "\"Running\": " << (running ? "yes" : "no") << ",";
	stream << "\"Current state\": \"" << currentStateName << "\",";
	stream << "\"State duration\": \"" << currentStateDuration << "\",";
	stream << "\"Total duration\": \"" << totalDuration << "\"";
	stream << "}";

	return stream.str();
}

// idle
void OffensiveAI::IdleState::step(float dt, float totalDuration, float stateDuration) {
	if (ai->running) {
		ai->setState("find-ball");
	}
}


// find ball
void OffensiveAI::FindBallState::onEnter() {
	
}

void OffensiveAI::FindBallState::onExit() {
	
}

void OffensiveAI::FindBallState::step(float dt, float totalDuration, float stateDuration) {
	//std::cout << "! Step find ball state: " << dt << ", " << totalDuration << ", " << stateDuration << std::endl;
}
