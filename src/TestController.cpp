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
	states["fetch-ball-straight"] = new FetchBallStraightState(this);
	states["fetch-ball-near"] = new FetchBallNearState(this);
	states["aim"] = new AimState(this);
	states["drive-circle"] = new DriveCircleState(this);
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

	// TODO Useful for testing, first focuses on goal
	if (stateDuration < 2.0f) {
		robot->lookAt(goal);

		return;
	}

	// config
	float approachP = 2.0f;
	float sideP = 1.0f;
	float nearDistance = Math::map(robot->getVelocity(), 0.0f, 2.0f, 0.25f, 1.0f);
	int maxSideSpeedThreshold = 0; // side speed is maximal at this distance from side
	int minSideSpeedThreshold = Config::cameraWidth / 2; // side speed is canceled starting from this distance from side

	float ballDistance = ball->getDribblerDistance();
	bool onLeft = ball->x < goal->x;
	int ballSideDistance = onLeft ? ball->x - ball->width / 2 : Config::cameraWidth - ball->x + ball->width / 2;

	if (ballDistance < nearDistance) {
		ai->setState("fetch-ball-near");

		return;
	}

	if (ai->parameters[0].length() > 0) approachP = Util::toFloat(ai->parameters[0]);
	if (ai->parameters[1].length() > 0) approachP = Util::toFloat(ai->parameters[1]);
	if (ai->parameters[2].length() > 0) nearDistance = Util::toFloat(ai->parameters[2]);

	float forwardSideRatio = Math::map((float)ballSideDistance, (float)maxSideSpeedThreshold, (float)minSideSpeedThreshold, 0.0f, 1.0f);
	float forwardSpeed = approachP * forwardSideRatio;
	float sideSpeed = (1.0f - forwardSideRatio) * Math::sign(ball->distanceX) * sideP;

	ai->dbg("ballDistance", ballDistance);
	ai->dbg("ballDistanceX", ball->distanceX);
	ai->dbg("nearDistance", nearDistance);
	ai->dbg("robotVelocity", robot->getVelocity());
	ai->dbg("ballAngle", Math::radToDeg(ball->angle));
	ai->dbg("forwardSideRatio", forwardSideRatio);
	ai->dbg("sideSpeed", sideSpeed);
	ai->dbg("forwardSpeed", forwardSpeed);
	ai->dbg("onLeft", onLeft);
	ai->dbg("ballDistanceFromSide", ballSideDistance);

	robot->setTargetDir(forwardSpeed, sideSpeed);
	//robot->lookAt(goal); // TODO Try focusing between the two
	robot->lookAt(Math::Rad((goal->angle + ball->angle) / 2.0f));
}

void TestController::FetchBallBehindState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	// TODO
}

void TestController::FetchBallStraightState::onEnter(Robot* robot) {
	startBrakingDistance = -1.0f;
}

void TestController::FetchBallStraightState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	if (robot->dribbler->gotBall()) {
		ai->dbg("gotBall", true);

		ai->setState("aim");

		return;
	}
	
	Object* ball = visionResults->getClosestBall(Dir::FRONT);
	Object* goal = visionResults->getLargestGoal(Side::BLUE, Dir::FRONT);

	ai->dbg("ballVisible", ball != NULL);
	ai->dbg("goalVisible", goal != NULL);

	// TODO Drive to ball and search for goal when lost goal (new state)
	if (ball == NULL || goal == NULL) {
		robot->stop();

		return;
	}

	float approachSpeed = 2.0f;
	float maxNearSpeed = 0.5f;
	float startAccelerationDuration = 0.5f;
	float maxOffsetDistanceAngleDiff = 45.0f;
	float minAngleDiffDistance = 0.2f;
	float maxAngleDiffDistance = 0.6f;
	float focusBetweenBallGoalAngle = 15.0f;
	float maxAngleBrakingAngle = 60.0f;
	float minApproachSpeed = 0.4f;
	float brakeP = 3.0f;
	float nearDistance = 0.15f;

	float adaptiveBrakingDistance = Math::map(robot->getVelocity(), 0.0f, 2.0, 0.5f, 1.5f);
	float ballDistance = ball->getDribblerDistance();
	float ballAngle = ball->angle;
	float goalAngle = goal->angle;
	float angleDiff = Math::abs(goalAngle - ballAngle);
	float offsetDistance = Math::map(Math::radToDeg(angleDiff), 0.0f, maxOffsetDistanceAngleDiff, minAngleDiffDistance, maxAngleDiffDistance);

	if (ai->parameters[0].length() > 0) approachSpeed = Util::toFloat(ai->parameters[0]);
	if (ai->parameters[1].length() > 0) offsetDistance = Util::toFloat(ai->parameters[1]);
	if (ai->parameters[2].length() > 0) startAccelerationDuration = Util::toFloat(ai->parameters[2]);
	if (ai->parameters[3].length() > 0) brakeP = Util::toFloat(ai->parameters[3]);

	//if (ballDistance < offsetDistance) {
	if (ballDistance < nearDistance) {
		ai->setState("fetch-ball-near");
		
		return;
	}
	
	float targetAngle = getTargetAngle(goal->distanceX, goal->distanceY, ball->distanceX, ball->distanceY, offsetDistance);

	// accelerate in the beginning
	float acceleratedSpeed = approachSpeed * Math::map(stateDuration, 0.0f, startAccelerationDuration, 0.0f, 1.0f);

	// only choose the braking distance once
	if (startBrakingDistance == -1.0f && ballDistance < adaptiveBrakingDistance) {
		startBrakingDistance = adaptiveBrakingDistance;
	}

	if (startBrakingDistance != -1.0f) {
		// brake as getting close and large target angle
		float distanceBraking = Math::map(ballDistance, 0.0f, startBrakingDistance, 1.0, 0.0f);
		float angleBreaking = Math::map(Math::abs(targetAngle), 0.0f, Math::degToRad(maxAngleBrakingAngle), 0.0f, 1.0f);
		float combinedBrakeFactor = brakeP * distanceBraking * angleBreaking;
		
		// limit max speed near the ball
		float maxSpeed = Math::map(ballDistance, 0.0f, startBrakingDistance, maxNearSpeed, approachSpeed);
		//float combinedBrakeFactor = brakeP * (distanceBraking + angleBreaking);

		acceleratedSpeed = Math::min(Math::max(acceleratedSpeed * (1.0f - combinedBrakeFactor), minApproachSpeed), maxSpeed);

		ai->dbg("distanceBraking", distanceBraking);
		ai->dbg("angleBreaking", angleBreaking);
		ai->dbg("combinedBrakeFactor", combinedBrakeFactor);
		ai->dbg("maxSpeed", maxSpeed);
	}

	robot->setTargetDir(Math::Rad(targetAngle), acceleratedSpeed);

	float lookAngle = Math::map(angleDiff, 0.0f, Math::degToRad(focusBetweenBallGoalAngle), goal->angle, (goal->angle + ball->angle) / 2.0f);

	robot->lookAt(Math::Rad(lookAngle));

	ai->dbg("acceleratedSpeed", acceleratedSpeed);
	ai->dbg("startBrakingDistance", startBrakingDistance);
	ai->dbg("ballDistance", ballDistance);
	ai->dbg("targetAngle", Math::radToDeg(targetAngle));
	ai->dbg("angleDiff", Math::radToDeg(angleDiff));
	ai->dbg("offsetDistance", offsetDistance);
	ai->dbg("lookAngle", Math::radToDeg(lookAngle));
}

float TestController::FetchBallStraightState::getTargetAngle(float goalX, float goalY, float ballX, float ballY, float D) {
	float targetX1;
	float targetX2;
	float targetY1;
	float targetY2;
	
	if(Math::abs(ballX - goalX) < 0.001){
		//ai->dbg("case", 1);

		//kui vahe on alla millimeetri, arvutame otse
		targetX1 = ballX;
		targetX2 = ballX;
		targetY1 = ballY - D;
		targetY2 = ballY + D;
	}
	else{
		//ai->dbg("case", 2);

		//Line connecting ball and goal
		float a = (ballY - goalY)/(ballX - goalX);
		float b = goalY - a * goalX;

		float underSqrt = sqrt(
			- pow(a, 2) * pow(ballX, 2)
			+ pow(a, 2) * pow(D, 2)
			- 2 * a * b * ballX
			+ 2 * a * ballX * ballY
			- pow(b, 2)
			+ 2 * b * ballY
			+ pow(D, 2)
			- pow(ballY, 2)
			);
		float rest = - a * b + a * ballY + ballX;
		float divisor = pow(a,2) + 1;

		targetX1 = ( + underSqrt + rest) / divisor;
		targetX2 = ( - underSqrt + rest) / divisor;
		targetY1 = a * targetX1 + b;
		targetY2 = a * targetX2 + b;
	}

	//Target's distance from goal (squared)
	float target1Dist = pow(goalX - targetX1, 2) + pow(goalY - targetY1, 2);
	float target2Dist = pow(goalX - targetX2, 2) + pow(goalY - targetY2, 2);

	//Choose target which is farther away from goal
	float targetX;
	float targetY;
	if(target1Dist > target2Dist){
		targetX = targetX1;
		targetY = targetY1;
	}
	else{
		targetX = targetX2;
		targetY = targetY2;
	}

	/*ai->dbg("targetX", targetX);
	ai->dbg("targetY", targetY);
	ai->dbg("target1Dist", target1Dist);
	ai->dbg("target2Dist", target2Dist);*/

	float targetAngle = atan2(targetX, targetY);
	return targetAngle;
}

void TestController::FetchBallNearState::onEnter(Robot* robot) {
	float minAllowedApproachSpeed = 0.25f;

	enterVelocity = Math::max(robot->getVelocity(), minAllowedApproachSpeed);
}

void TestController::FetchBallNearState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
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

	float approachP = 1.5f;
	float sideP = 0.75f;
	float nearZeroSpeedAngle = 10.0f;
	float nearMaxSideSpeedAngle = 45.0f;
	float ballDistance = ball->getDribblerDistance();

	if (ai->parameters[0].length() > 0) approachP = Util::toFloat(ai->parameters[0]);
	if (ai->parameters[1].length() > 0) sideP = Util::toFloat(ai->parameters[1]);
	if (ai->parameters[2].length() > 0) nearZeroSpeedAngle = Util::toFloat(ai->parameters[2]);

	float forwardSpeed = Math::min(approachP * Math::map(Math::abs(Math::radToDeg(ball->angle)), 0.0f, nearZeroSpeedAngle, 1.0f, 0.0f), enterVelocity);
	float sideSpeed = sideP * Math::sign(ball->distanceX) * Math::map(Math::abs(Math::radToDeg(ball->angle)), 0.0f, nearMaxSideSpeedAngle, 0.0f, 1.0f);

	ai->dbg("ballDistance", ballDistance);
	ai->dbg("forwardSpeed", forwardSpeed);
	ai->dbg("sideSpeed", sideSpeed);
	ai->dbg("ballAngle", (Math::radToDeg(ball->angle)));

	robot->dribbler->start();
	robot->setTargetDir(forwardSpeed, sideSpeed);
	robot->lookAt(goal);
}

void TestController::AimState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	robot->stop();

	ai->dbg("gotBall", robot->dribbler->gotBall());
	
	if (!robot->dribbler->gotBall()) {
		return;
	}
	
	Object* goal = visionResults->getLargestGoal(Side::BLUE, Dir::FRONT);

	ai->dbg("goalVisible", goal != NULL);

	if (goal == NULL) {
		return;
	}

	ai->dbg("goalVisible", true);

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

	ai->dbg("shouldKick", shouldKick);
	ai->dbg("leftEdge", leftEdge);
	ai->dbg("rightEdge", rightEdge);
	ai->dbg("goalKickThresholdPixels", goalKickThresholdPixels);
	ai->dbg("sinceLastKick", lastKickTime != 0.0 ? Util::duration(lastKickTime) : -1.0);

	if (shouldKick && lastKickTime == 0.0 || Util::duration(lastKickTime) >= 1) {
		robot->kick();

		lastKickTime = Util::millitime();
	} else {
		robot->lookAt(goal);
	}
}

void TestController::DriveCircleState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	Object* goal = visionResults->getLargestGoal(Side::BLUE, Dir::FRONT);

	ai->dbg("goalVisible", goal != NULL);

	if (goal == NULL) {
		return;
	}

	float targetAngle = Math::circleAround(0.0f, stateDuration, 5.0f);

	robot->setTargetDir(Math::Rad(targetAngle), 0.2f);
	robot->lookAt(goal);
}