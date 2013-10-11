#include "TestController.h"

#include "Robot.h"
#include "Dribbler.h"
#include "Command.h"

TestController::TestController(Robot* robot, Communication* com) : BaseAI(robot, com), manualSpeedX(0.0f), manualSpeedY(0.0f), manualOmega(0.0f), manualDribblerSpeed(0), manualKickStrength(0), blueGoalDistance(0.0f), yellowGoalDistance(0.0f), lastCommandTime(0.0) {
	setupStates();
};

TestController::~TestController() {
	
}

void TestController::setupStates() {
	states["manual-control"] = new ManualControlState(this);
	states["watch-ball"] = new WatchBallState(this);
	states["watch-goal"] = new WatchGoalState(this);
	states["spin-around-dribbler"] = new SpinAroundDribblerState(this);
	states["drive-to"] = new DriveToState(this);
	states["fetch-ball-infront"] = new FetchBallInfrontState(this);
	states["fetch-ball-behind"] = new FetchBallBehindState(this);
	states["aim"] = new AimState(this);
}

void TestController::step(float dt, Vision::Results* visionResults) {
	updateGoalDistances(visionResults);
	
	if (currentState == NULL) {
		setState("manual-control");
	}

	currentStateDuration += dt;
	totalDuration += dt;

	if (currentState != NULL) {
		currentState->step(dt, visionResults, robot, totalDuration, currentStateDuration);
	}
}

bool TestController::handleCommand(const Command& cmd) {
	if (cmd.name == "target-vector" && cmd.parameters.size() == 3) {
        handleTargetVectorCommand(cmd);
    } else if (cmd.name == "set-dribbler" && cmd.parameters.size() == 1) {
        handleDribblerCommand(cmd);
    } else if (cmd.name == "kick" && cmd.parameters.size() == 1) {
        handleKickCommand(cmd);
    } else if (cmd.name == "reset-position") {
		robot->setPosition(Config::fieldWidth / 2.0f, Config::fieldHeight / 2.0f, 0.0f);
    } else if (cmd.name == "stop") {
        handleResetCommand();
		setState("manual-control");
    } else if (cmd.name == "reset" || cmd.name == "toggle-side") {
        handleResetCommand();
    } else if (cmd.name == "drive-to" && cmd.parameters.size() == 3) {
        handleDriveToCommand(cmd);
    } else if (cmd.name.substr(0, 4) == "run-") {
        setState(cmd.name.substr(4));
    } else if (cmd.name == "parameter" && cmd.parameters.size() == 2) {
		handleParameterCommand(cmd);
	} else {
		return false;
	}

    return true;
}

void TestController::handleTargetVectorCommand(const Command& cmd) {
    manualSpeedX = Util::toFloat(cmd.parameters[0]);
    manualSpeedY = Util::toFloat(cmd.parameters[1]);
    manualOmega = Util::toFloat(cmd.parameters[2]);

	lastCommandTime = Util::millitime();
}

void TestController::handleDribblerCommand(const Command& cmd) {
    manualDribblerSpeed = Util::toInt(cmd.parameters[0]);

	lastCommandTime = Util::millitime();
}

void TestController::handleKickCommand(const Command& cmd) {
    manualKickStrength = Util::toInt(cmd.parameters[0]);

	lastCommandTime = Util::millitime();
}

void TestController::handleResetCommand() {
	if (!resetBtn.toggle()) {
		return;
	}

	std::cout << "! Resetting test controller" << std::endl;

	totalDuration = 0.0f;
	currentStateDuration = 0.0f;

	setState(currentStateName);
}

void TestController::handleParameterCommand(const Command& cmd) {
	int index = Util::toInt(cmd.parameters[0]);
	std::string value = cmd.parameters[1];

	parameters[index] = value;

	std::cout << "! Received parameter #" << index << ": " << value << std::endl;
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
	
	for (MessagesIt it = messages.begin(); it != messages.end(); it++) {
		stream << "\"" << (it->first) << "\": \"" << (it->second) << "\",";
	}

	messages.clear();

	stream << "\"currentState\": \"" << currentStateName << "\",";
	stream << "\"stateDuration\": \"" << currentStateDuration << "\",";
	stream << "\"totalDuration\": \"" << totalDuration << "\",";
	stream << "\"blueGoalDistance\": " << blueGoalDistance << ",";
	stream << "\"yellowGoalDistance\": " << yellowGoalDistance;

	stream << "}";

	return stream.str();
}

void TestController::ManualControlState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	double time = Util::millitime();

	if (time - ai->lastCommandTime < 0.5) {
		robot->setTargetDir(ai->manualSpeedX, ai->manualSpeedY, ai->manualOmega);
		robot->dribbler->setTargetSpeed(-ai->manualDribblerSpeed);

		if (ai->manualKickStrength != 0.0f) {
			robot->kick(ai->manualKickStrength);

			ai->manualKickStrength = 0;
		}
	} else {
		robot->stop();
		robot->dribbler->setTargetSpeed(0);
	}
}

void TestController::WatchBallState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	Object* ball = visionResults->getClosestBall(Dir::FRONT);

	if (ball == NULL) {
		robot->setTargetDir(ai->manualSpeedX, ai->manualSpeedY, ai->manualOmega);

		return;
	}

	robot->setTargetDir(ai->manualSpeedX, ai->manualSpeedY);
	robot->lookAt(ball);
}

void TestController::WatchGoalState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	Object* goal = visionResults->getLargestGoal(Side::BLUE, Dir::FRONT);

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

void TestController::FetchBallInfrontState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	if (robot->dribbler->gotBall()) {
		ai->dbg("gotBall", true);

		ai->setState("aim");

		return;
	}
	
	Object* ball = visionResults->getClosestBall(Dir::FRONT);
	Object* goal = visionResults->getLargestGoal(Side::BLUE, Dir::FRONT);

	ai->dbg("ballVisible", ball != NULL);
	ai->dbg("goalVisible", goal != NULL);

	if (ball == NULL || goal == NULL) {
		robot->stop();

		return;
	}

	float ballDistance = ball->getDribblerDistance();
	bool onLeft = ball->x < goal->x;
	int ballSideDistance = onLeft ? ball->x - ball->width / 2 : Config::cameraWidth - ball->x + ball->width / 2;

	// config
	//float sideP = 1.0f;
	//float forwardP = 3.0f;
	//float forwardP = 1.0f;
	float approachSpeed = 2.0f;
	float sideP = 1.0f;
	float zeroSpeedAngle = 35.0f;
	float nearDistance = 0.5f;
	float nearSpeed = 1.0f;
	float dribblerStartDistance = 0.5f;
	//int maxSideSpeedThreshold = 25; // side speed is maximal at this distance from side
	//int minSideSpeedThreshold = Config::cameraWidth / 4; // side speed is canceled starting from this distance from side
	int maxSideSpeedThreshold = 0; // side speed is maximal at this distance from side
	int minSideSpeedThreshold = Config::cameraWidth / 2; // side speed is canceled starting from this distance from side
	float sideSpeed, forwardSpeed;

	if (ai->parameters[0].length() > 0) approachSpeed = Util::toFloat(ai->parameters[0]);
	if (ai->parameters[1].length() > 0) nearDistance = Util::toFloat(ai->parameters[1]);
	if (ai->parameters[2].length() > 0) nearSpeed = Util::toFloat(ai->parameters[2]);
	
	//float sideSpeedMultiplier = Math::map((float)ballSideDistance, (float)sideMovementMaxThreshold, (float)cancelSideMovementThreshold, 1.0f, 0.0f);
	
	/*if (ballDistance <= nearDistance) {
		sideSpeedMultiplier = 1.0f;
	}*/
	
	//float sideSpeed = ball->distanceX * sideP * sideSpeedMultiplier;
	
	//float forwardSpeed = Math::max(Math::degToRad(zeroSpeedAngle) - Math::abs(ball->angle), 0.0f) * forwardP;
	//float forwardSpeed = forwardP * (1.0f - sideSpeedMultiplier);

	if (ballDistance > nearDistance) {
		//sideSpeed = approachSpeed * Math::sign(ball->distanceX) * Math::map((float)ballSideDistance, (float)maxSideSpeedThreshold, (float)minSideSpeedThreshold, 1.0f, 0.0f);
		sideSpeed = sideP * Math::sign(ball->distanceX) * Math::map((float)ballSideDistance, (float)maxSideSpeedThreshold, (float)minSideSpeedThreshold, 1.0f, 0.0f);
		forwardSpeed = approachSpeed - Math::abs(sideSpeed);
	} else {
		forwardSpeed = nearSpeed * Math::map(Math::abs(Math::radToDeg(ball->angle)), 0.0f, zeroSpeedAngle, 1.0f, 0.0f);
		sideSpeed = Math::sign(ball->distanceX) * (nearSpeed - forwardSpeed);
	}

	if (ballDistance < dribblerStartDistance) {
		robot->dribbler->start();
	} else {
		robot->dribbler->stop();
	}

	ai->dbg("ballDistance", ballDistance);
	ai->dbg("sideSpeed", sideSpeed);
	ai->dbg("forwardSpeed", forwardSpeed);
	ai->dbg("onLeft", onLeft);
	ai->dbg("ballDistanceFromSide", ballSideDistance);
	//ai->dbg("sideSpeedMultiplier", sideSpeedMultiplier);

	robot->setTargetDir(forwardSpeed, sideSpeed);
	//robot->setTargetDir(Math::Rad(ball->angle), 1.0f);
	robot->lookAt(goal);
}

void TestController::FetchBallBehindState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	Object* ball = visionResults->getClosestBall(Dir::REAR);
	Object* goal = visionResults->getLargestGoal(Side::BLUE, Dir::REAR);

	if (ball == NULL || goal == NULL) {
		return;
	}

}

void TestController::AimState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	robot->stop();
	
	if (!robot->dribbler->gotBall()) {
		return;
	}
	
	Object* goal = visionResults->getLargestGoal(Side::BLUE, Dir::FRONT);

	if (goal == NULL) {
		return;
	}

	robot->setTargetDir(0.0f, 0.0f, 0.0f);
	robot->dribbler->start();

	int halfWidth = Config::cameraWidth / 2;
	int leftEdge = goal->x - goal->width / 2;
	int rightEdge = goal->x + goal->width / 2;
	int goalKickThresholdPixels = (int)((float)goal->width * Config::goalKickThreshold);
	bool shouldKick = false;

	if (!goal->behind) {
		if (
			leftEdge + goalKickThresholdPixels < halfWidth
			&& rightEdge - goalKickThresholdPixels > halfWidth
		) {
			shouldKick = true;
		}
	}

	if (shouldKick && lastKickTime == 0.0 || Util::duration(lastKickTime) >= 1) {
		robot->kick();

		lastKickTime = Util::millitime();
	} else {
		robot->lookAt(goal);
	}
}