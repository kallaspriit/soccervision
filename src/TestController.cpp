#include "TestController.h"

#include "Robot.h"
#include "Command.h"
#include "Util.h"

TestController::TestController(Robot* robot, Communication* com) : BaseAI(robot, com), running(false) {
	setupStates();
};

void TestController::setupStates() {
	states["idle"] = new IdleState(this);
	states["watch-ball"] = new WatchBallState(this);
}

void TestController::step(float dt, Vision::Results* visionResults) {
	currentStateDuration += dt;
	totalDuration += dt;

	if (currentState == NULL) {
		setState("idle");
	}

	if (currentState != NULL && running) {
		currentState->step(dt, totalDuration, currentStateDuration);
	}
}

bool TestController::handleCommand(const Command& cmd) {
    if (cmd.name == "toggle-go") {
        handleToggleGoCommand();
    } else {
		return false;
	}

    return true;
}

void TestController::handleToggleGoCommand() {
	if (!toggleGoBtn.toggle()) {
		return;
	}

	running = !running;
}

std::string TestController::getJSON() {
	std::stringstream stream;

	stream << "{";
	stream << "\"Running\": \"" << (running ? "yes" : "no") << "\",";
	stream << "\"Current state\": \"" << currentStateName << "\",";
	stream << "\"State duration\": \"" << currentStateDuration << "\",";
	stream << "\"Total duration\": \"" << totalDuration << "\"";
	stream << "}";

	return stream.str();
}

// watch ball
void TestController::WatchBallState::onEnter() {
	std::cout << "! Enter test watch ball" << std::endl;
}

void TestController::WatchBallState::onExit() {
	std::cout << "! Exit test watch ball" << std::endl;
}

void TestController::WatchBallState::step(float dt, float totalDuration, float stateDuration) {
	std::cout << "! Step test watch ball state: " << dt << ", " << totalDuration << ", " << stateDuration << std::endl;
}
