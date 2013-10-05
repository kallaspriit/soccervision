#include "TestController.h"

#include "Robot.h"
#include "Command.h"
#include "Util.h"

TestController::TestController(Robot* robot, Communication* com) : BaseAI(robot, com), running(false), manualSpeedX(0.0f), manualSpeedY(0.0f), manualOmega(0.0f), blueGoalDistance(0.0f), yellowGoalDistance(0.0f) {
	setupStates();
};

void TestController::setupStates() {
	states["idle"] = new IdleState(this);
	states["watch-ball"] = new WatchBallState(this);
	states["watch-goal"] = new WatchGoalState(this);
	states["spin-around-dribbler"] = new SpinAroundDribblerState(this);
	states["drive-to"] = new DriveToState(this);
}

void TestController::step(float dt, Vision::Results* visionResults) {
	updateGoalDistances(visionResults);
	
	if (currentState == NULL) {
		setState("idle");

		running = true;
	}

	if (running) {
		currentStateDuration += dt;
		totalDuration += dt;

		if (currentState != NULL) {
			currentState->step(dt, visionResults, robot, totalDuration, currentStateDuration);
		}
	} else {
		robot->stop();
	}
}

bool TestController::handleCommand(const Command& cmd) {
	if (cmd.name == "target-vector" && cmd.parameters.size() == 3) {
        handleTargetVectorCommand(cmd);
    } else if (cmd.name == "toggle-go") {
        handleToggleGoCommand();
    } else if (cmd.name == "stop" ||cmd.name == "toggle-side") {
        handleResetCommand();
    } else if (cmd.name == "drive-to" && cmd.parameters.size() == 3) {
        handleDriveToCommand(cmd);
    } else if (cmd.name.substr(0, 4) == "run-") {
        setState(cmd.name.substr(4));

		running = true;
    } else {
		return false;
	}

    return true;
}

void TestController::handleTargetVectorCommand(const Command& cmd) {
    manualSpeedX = Util::toFloat(cmd.parameters[0]);
    manualSpeedY = Util::toFloat(cmd.parameters[1]);
    manualOmega = Util::toFloat(cmd.parameters[2]);
}

void TestController::handleToggleGoCommand() {
	if (!toggleGoBtn.toggle()) {
		return;
	}

	running = !running;

	std::cout << "! " << (running ? "Starting test" : "Pausing test") << std::endl;
}

void TestController::handleResetCommand() {
	if (!resetBtn.toggle()) {
		return;
	}

	std::cout << "! Resetting test controller" << std::endl;

	running = false;
	totalDuration = 0.0f;
	currentStateDuration = 0.0f;

	setState(currentStateName);
}

void TestController::handleDriveToCommand(const Command& cmd) {
	DriveToState* state = (DriveToState*)states["drive-to"];

	state->x = Util::toFloat(cmd.parameters[0]);
	state->y = Util::toFloat(cmd.parameters[1]);
	state->orientation = Util::toFloat(cmd.parameters[2]);

	setState("drive-to");
}

void TestController::updateGoalDistances(Vision::Results* visionResults) {
	Object* blueGoal = visionResults->getLargestGoal(Side::BLUE);
	Object* yellowGoal = visionResults->getLargestGoal(Side::YELLOW);
	
	blueGoalDistance = blueGoal != NULL ? blueGoal->distance : 0.0f;
	yellowGoalDistance = yellowGoal != NULL ? yellowGoal->distance : 0.0f;
}

std::string TestController::getJSON() {
	std::stringstream stream;

	stream << "{";
	stream << "\"Running\": \"" << (running ? "yes" : "no") << "\",";
	stream << "\"Current state\": \"" << currentStateName << "\",";
	stream << "\"State duration\": \"" << currentStateDuration << "\",";
	stream << "\"Total duration\": \"" << totalDuration << "\",";
	stream << "\"blueGoalDistance\": " << blueGoalDistance << ",";
	stream << "\"yellowGoalDistance\": " << yellowGoalDistance;
	stream << "}";

	return stream.str();
}

void TestController::IdleState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	robot->setTargetDir(ai->manualSpeedX, ai->manualSpeedY, ai->manualOmega);
}

void TestController::WatchBallState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	Object* ball = visionResults->getClosestBall(true);

	if (ball == NULL) {
		robot->setTargetDir(ai->manualSpeedX, ai->manualSpeedY, ai->manualOmega);

		return;
	}

	robot->setTargetDir(ai->manualSpeedX, ai->manualSpeedY);
	robot->lookAt(ball);
}

void TestController::WatchGoalState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	Object* goal = visionResults->getLargestGoal(Side::BLUE, true);

	if (goal == NULL) {
		robot->setTargetDir(ai->manualSpeedX, ai->manualSpeedY, ai->manualOmega);

		return;
	}

	robot->setTargetDir(ai->manualSpeedX, ai->manualSpeedY);
	robot->lookAt(goal);
}

void TestController::SpinAroundDribblerState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	robot->spinAroundDribbler();
}

void TestController::DriveToState::onEnter(Robot* robot) {
	robot->driveTo(x, y, orientation);
}

void TestController::DriveToState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	
}
