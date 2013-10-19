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
	//states["fetch-ball-infront"] = new FetchBallInfrontState(this);
	states["fetch-ball-behind"] = new FetchBallBehindState(this);
	states["fetch-ball-front"] = new FetchBallFrontState(this);
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

float TestController::getTargetAngle(float goalX, float goalY, float ballX, float ballY, float D, TestController::TargetMode targetMode) {
	float targetX1;
	float targetX2;
	float targetY1;
	float targetY2;

	if (Math::abs(ballX - goalX) < 0.001f){
		// special case of small x difference
		if (targetMode == TargetMode::INLINE){
			// let's not divide by zero
			targetX1 = ballX;
			targetX2 = ballX;
			targetY1 = ballY + D;
			targetY2 = ballY - D;
		} else if (targetMode == TargetMode::LEFT || targetMode == TargetMode::RIGHT){
			targetX1 = ballX + D;
			targetX2 = ballX - D;
			targetY1 = ballY;
			targetY2 = ballY;
		}
	} else if(Math::abs(ballY - goalY) < 0.001f){
		// special case of small y difference
		if (targetMode == TargetMode::INLINE){
			targetX1 = ballX + D;
			targetX2 = ballX - D;
			targetY1 = ballY;
			targetY2 = ballY;
		} else if (targetMode == TargetMode::LEFT || targetMode == TargetMode::RIGHT) {
			targetX1 = ballX;
			targetX2 = ballX;
			targetY1 = ballY + D;
			targetY2 = ballY - D;
		}
	} else {
		// normal case
		float a = (ballY - goalY) / (ballX - goalX);

		if (targetMode == TargetMode::LEFT || targetMode == TargetMode::RIGHT){
			// perpendicular to the line from goal to ball
			a = -(1.0f / a);
		}

		float b = ballY - a * ballX;
		float underSqrt = sqrt(
			- pow(a, 2) * pow(ballX, 2)
			+ pow(a, 2) * pow(D, 2)
			- 2.0f * a * b * ballX
			+ 2.0f * a * ballX * ballY
			- pow(b, 2)
			+ 2.0f * b * ballY
			+ pow(D, 2)
			- pow(ballY, 2)
		);
		float rest = - a * b + a * ballY + ballX;
		float divisor = pow(a, 2) + 1.0f;

		targetX1 = ( + underSqrt + rest) / divisor;
		targetX2 = ( - underSqrt + rest) / divisor;
		targetY1 = a * targetX1 + b;
		targetY2 = a * targetX2 + b;
	}

	// target's distance from goal (squared)
	float target1Dist = pow(goalX - targetX1, 2) + pow(goalY - targetY1, 2);
	float target2Dist = pow(goalX - targetX2, 2) + pow(goalY - targetY2, 2);

	float targetX;
	float targetY;

	if (targetMode == TargetMode::INLINE) {
		// choose target which is farther away from goal
		if (target1Dist > target2Dist){
			targetX = targetX1;
			targetY = targetY1;
		} else{
			targetX = targetX2;
			targetY = targetY2;
		}
	} else if (targetMode == TargetMode::LEFT) {
		// choose one on left
		targetX = targetX2;
		targetY = targetY2;
	} else if (targetMode == TargetMode::RIGHT) {
		// choose one on right
		targetX = targetX1;
		targetY = targetY1;
	}

	float targetAngle = atan2(targetX, targetY);

	return targetAngle;
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

/*void TestController::FetchBallInfrontState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
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
}*/

void TestController::FetchBallFrontState::onEnter(Robot* robot) {
	startBrakingDistance = -1.0f;
	startBrakingVelocity = -1.0f;
}

void TestController::FetchBallFrontState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	robot->stop();
	
	if (robot->dribbler->gotBall()) {
		ai->dbg("gotBall", true);
		ai->dbgs("action", "Switch to aim");

		ai->setState("aim");

		return;
	}

	double minSearchFrontDuration = 1.0;
	
	// prefer balls in the front camera
	Object* ball = visionResults->getClosestBall(Dir::FRONT);
	Object* goal = visionResults->getLargestGoal(Side::BLUE, Dir::FRONT);

	if (ball == NULL && stateDuration > minSearchFrontDuration) {
		ball = visionResults->getClosestBall(Dir::ANY);
	}

	ai->dbg("ballVisible", ball != NULL);
	ai->dbg("goalVisible", goal != NULL);

	if (ball != NULL && ball->behind) {
		ai->dbgs("action", "Switch to fetch behind");

		ai->setState("fetch-ball-behind");

		return;
	}

	// TODO Drive to ball and search for goal when lost goal (new state)
	if (ball == NULL || goal == NULL) {
		return;
	}

	float approachSpeed = 2.0f;
	float maxNearSpeed = 1.0f;
	float startAccelerationDuration = 0.75f;
	float maxOffsetDistanceAngleDiff = 45.0f;
	float maxAngleDiffDistance = 0.6f;
	float focusBetweenBallGoalAngle = 15.0f;
	float maxAngleBrakingAngle = 40.0f;
	float maxBrakingDistanceVelocity = 2.0f;
	float minVelocityBrakeDistance = 0.5f;
	float maxVelocityBrakingDistance = 1.5f;
	float minApproachSpeed = 0.4f;
	float nearDistance = 0.2f;

	float ballDistance = ball->getDribblerDistance();
	float ballAngle = ball->angle;
	float goalAngle = goal->angle;
	float angleDiff = Math::abs(goalAngle - ballAngle);
	float offsetDistance = Math::map(Math::radToDeg(angleDiff), 0.0f, maxOffsetDistanceAngleDiff, nearDistance, maxAngleDiffDistance);

	if (ai->parameters[0].length() > 0) approachSpeed = Util::toFloat(ai->parameters[0]);
	if (ai->parameters[1].length() > 0) offsetDistance = Util::toFloat(ai->parameters[1]);
	if (ai->parameters[2].length() > 0) startAccelerationDuration = Util::toFloat(ai->parameters[2]);

	//if (ballDistance < offsetDistance) {
	if (ballDistance < nearDistance) {
		ai->dbgs("action", "Switch to fetch ball near");
		ai->setState("fetch-ball-near"); // TODO Add back
		
		return;
	}
	
	float adaptiveBrakingDistance = Math::map(robot->getVelocity(), 0.0f, maxBrakingDistanceVelocity, minVelocityBrakeDistance, maxVelocityBrakingDistance);
	float targetAngle = ai->getTargetAngle(goal->distanceX, goal->distanceY, ball->distanceX, ball->distanceY, offsetDistance);

	// accelerate in the beginning
	float acceleratedSpeed = approachSpeed * Math::map(stateDuration, 0.0f, startAccelerationDuration, 0.0f, 1.0f);

	// only choose the braking distance once
	if (startBrakingDistance == -1.0f && ballDistance < adaptiveBrakingDistance) {
		startBrakingDistance = adaptiveBrakingDistance;
		startBrakingVelocity = robot->getVelocity();
	}

	if (startBrakingDistance != -1.0f) {
		// brake as getting close and large target angle
		float distanceBraking = Math::map(ballDistance, nearDistance, startBrakingDistance, 1.0, 0.0f);
		float angleBreaking = Math::map(Math::abs(targetAngle), 0.0f, Math::degToRad(maxAngleBrakingAngle), 0.0f, 1.0f);
		float combinedBrakeFactor = distanceBraking * angleBreaking;
		//float combinedBrakeFactor = brakeP * (distanceBraking + angleBreaking);
		
		// limit max speed near the ball
		float maxSpeed = Math::map(ballDistance, nearDistance, startBrakingDistance, maxNearSpeed, startBrakingVelocity);
		
		acceleratedSpeed = acceleratedSpeed * (1.0f - combinedBrakeFactor);
		acceleratedSpeed = Math::max(acceleratedSpeed, minApproachSpeed);
		acceleratedSpeed = Math::min(acceleratedSpeed, maxSpeed);

		ai->dbg("distanceBraking", distanceBraking);
		ai->dbg("angleBreaking", angleBreaking);
		ai->dbg("combinedBrakeFactor", combinedBrakeFactor);
		ai->dbg("maxSpeed", maxSpeed);
	}
	
	float lookAngle = Math::map(angleDiff, 0.0f, Math::degToRad(focusBetweenBallGoalAngle), goal->angle, (goal->angle + ball->angle) / 2.0f);

	robot->setTargetDir(Math::Rad(targetAngle), acceleratedSpeed);
	robot->lookAt(Math::Rad(lookAngle));

	ai->dbg("acceleratedSpeed", acceleratedSpeed);
	ai->dbg("startBrakingDistance", startBrakingDistance);
	ai->dbg("startBrakingVelocity", startBrakingVelocity);
	ai->dbg("ballDistance", ballDistance);
	ai->dbg("targetAngle", Math::radToDeg(targetAngle));
	ai->dbg("angleDiff", Math::radToDeg(angleDiff));
	ai->dbg("offsetDistance", offsetDistance);
	ai->dbg("nearDistance", nearDistance);
	ai->dbg("lookAngle", Math::radToDeg(lookAngle));
}

void TestController::FetchBallBehindState::onEnter(Robot* robot) {
	hadBall = false;
	lastTargetAngle = 0.0f;
	lostBallTime = -1.0;
	timeSinceLostBall = 0.0;
	lostBallVelocity = 0.0f;
	startBallDistance = -1.0f;
	targetMode = TargetMode::UNDECIDED;
}

void TestController::FetchBallBehindState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	robot->stop();

	if (robot->dribbler->gotBall()) {
		ai->dbg("gotBall", true);

		ai->setState("aim");

		return;
	}

	double maxBlindReverseDuration = 1.5;
	double minSearchBehindDuration = 1.0;

	Object* ball = visionResults->getClosestBall(Dir::REAR);
	Object* goal = visionResults->getLargestGoal(Side::BLUE, Dir::FRONT);

	if (ball == NULL && stateDuration > minSearchBehindDuration) {
		ball = visionResults->getClosestBall(Dir::ANY);
	}

	ai->dbg("ballVisible", ball != NULL);
	ai->dbg("goalVisible", goal != NULL);
	ai->dbg("timeSinceLostBall", timeSinceLostBall);

	if (goal == NULL) {
		return; // TODO What now?
	}

	if (ball != NULL) {
		ai->dbg("ballDistance", ball->getDribblerDistance());
		ai->dbg("ball->behind", ball->behind);
	}

	ai->dbg("startBallDistance", startBallDistance);
	ai->dbg("hadBall", hadBall);

	// only revert to fetch front if not fetching behind blind
	if (
		ball != NULL
		&& !ball->behind
		&& (
			ball->getDribblerDistance() <= startBallDistance
			|| timeSinceLostBall >= maxBlindReverseDuration
			|| !hadBall
		)
	) {
		ai->setState("fetch-ball-front");

		return;
	}

	if (ball == NULL || !ball->behind) {
		if (!hadBall) {
			return; // TODO Never had the ball, what now?
		}

		if (lostBallTime == -1.0) {
			lostBallTime = Util::millitime();
			lostBallVelocity = robot->getVelocity();
		}
		
		timeSinceLostBall = Util::duration(lostBallTime);

		if (timeSinceLostBall > maxBlindReverseDuration) {
			if (ball != NULL) {
				ai->setState("fetch-ball-front");
			}

			return; // TODO Start searching for new ball
		}

		float fetchBlindSpeed = 0.5f;
		//float sideP = 0.4f;
		float sideAccelerationDuration = 0.5f;
		float deaccelerationDuration = 0.5f;
		double sideSpeedDelay = 0.5;
		float deacceleratedSpeed = Math::map((float)timeSinceLostBall, 0.0f, deaccelerationDuration, lostBallVelocity, fetchBlindSpeed);
		float targetModeSide = targetMode == TargetMode::LEFT ? 1.0f : -1.0f;
		float sideSpeed = targetModeSide * Math::map((float)(timeSinceLostBall - sideSpeedDelay), 0.0f, sideAccelerationDuration, 0.0f, deacceleratedSpeed);

		Math::Vector dirVector = Math::Vector::createForwardVec(lastTargetAngle, deacceleratedSpeed);

		dirVector.y += sideSpeed;

		robot->setTargetDir(dirVector.x, dirVector.y);
		robot->lookAt(goal);

		ai->dbgs("mode", "blind");
		ai->dbg("sideSpeed", sideSpeed);
		ai->dbg("deacceleratedSpeed", deacceleratedSpeed);

		return;
	}

	float ballDistance = ball->getDribblerDistance();

	hadBall = true;
	timeSinceLostBall = 0.0;
	lostBallTime = -1.0;

	if (startBallDistance == -1.0f) {
		startBallDistance = ballDistance;
	}

	ai->dbgs("mode", "visible");

	float offsetDistance = 0.2f;
	float approachP = 2.0f;
	float startAccelerationDuration = 0.5f;

	if (targetMode == TargetMode::UNDECIDED) {
		// TODO Select best target mode
		targetMode = TargetMode::LEFT;
	}

	if (ai->parameters[0].length() > 0) offsetDistance = Util::toFloat(ai->parameters[0]);
	if (ai->parameters[1].length() > 0) approachP = Util::toFloat(ai->parameters[1]);
	if (ai->parameters[2].length() > 0) {
		int targetModeVal = Util::toInt(ai->parameters[2]);

		if (targetModeVal == -1) {
			targetMode = TargetMode::LEFT;
		} else if (targetModeVal == 1) {
			targetMode = TargetMode::RIGHT;
		} else {
			targetMode = TargetMode::INLINE;
		}
	}

	//float targetAngle = ai->getTargetAngle(goal->distanceX, goal->distanceY * (goal->behind ? -1.0f : 1.0f), ball->distanceX, ball->distanceY * (ball->behind ? -1.0f : 1.0f), offsetDistance, TargetMode::RIGHT);
	float targetAngle = ai->getTargetAngle(goal->distanceX * (goal->behind ? -1.0f : 1.0f), goal->distanceY * (goal->behind ? -1.0f : 1.0f), ball->distanceX * (ball->behind ? -1.0f : 1.0f), ball->distanceY * (ball->behind ? -1.0f : 1.0f), offsetDistance, targetMode);
	float approachSpeed = approachP * Math::map(stateDuration, 0.0f, startAccelerationDuration, 0.0f, 1.0f);
	float deacceleratedSpeed = Math::map(ballDistance, 0.3f, 1.0f, 0.5f, approachSpeed);

	ai->dbg("offsetDistance", offsetDistance);
	ai->dbg("approachSpeed", approachSpeed);
	ai->dbg("deacceleratedSpeed", deacceleratedSpeed);
	ai->dbg("targetAngle", Math::radToDeg(targetAngle));
	/*ai->dbg("goal->distanceX", goal->distanceX * (goal->behind ? -1.0f : 1.0f));
	ai->dbg("goal->distanceY", goal->distanceY * (goal->behind ? -1.0f : 1.0f));
	ai->dbg("ball->distanceX", ball->distanceX * (ball->behind ? -1.0f : 1.0f));
	ai->dbg("ball->distanceY", ball->distanceY * (ball->behind ? -1.0f : 1.0f));*/

	robot->setTargetDir(Math::Rad(targetAngle), deacceleratedSpeed);
	robot->lookAt(goal);

	lastTargetAngle = targetAngle;
}

void TestController::FetchBallNearState::onEnter(Robot* robot) {
	float minAllowedApproachSpeed = 0.25f;

	enterVelocity = Math::max(robot->getVelocity(), minAllowedApproachSpeed);
	enterBallDistance = -1.0f;
}

void TestController::FetchBallNearState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	robot->stop();
	
	if (robot->dribbler->gotBall()) {
		ai->dbg("gotBall", true);

		ai->setState("aim");

		return;
	}
	
	Object* ball = visionResults->getClosestBall();
	Object* goal = visionResults->getLargestGoal(Side::BLUE, Dir::FRONT);

	ai->dbg("ballVisible", ball != NULL);
	ai->dbg("goalVisible", goal != NULL);
	ai->dbg("enterBallDistance", enterBallDistance);

	if (goal == NULL) {
		return; // TODO Start searching
	}

	if (ball != NULL && ball->behind) {
		ai->setState("fetch-ball-behind");

		return;
	}

	if (ball == NULL) {
		return; // TODO Start searching
	}

	float ballDistance = ball->getDribblerDistance();

	ai->dbg("ballDistance", ballDistance);
	
	float approachP = 1.5f;
	float sideP = 0.75f;
	//float sideP = 0.5f;
	//float nearZeroSpeedAngle = 10.0f;
	float nearZeroSpeedAngle = Math::map(ballDistance, 0.0f, 0.75f, 5.0f, 25.0f);
	float nearMaxSideSpeedAngle = 45.0f;
	//float nearMaxSideSpeedAngle = nearZeroSpeedAngle * 2.0f;

	if (enterBallDistance == -1.0f) {
		enterBallDistance = ballDistance;
	} else if (ballDistance > enterBallDistance * 1.25f) {
		// ball has gotten further than when started, probably messed it up, switch to faster fetch
		ai->setState("fetch-ball-front");

		return;
	}

	if (ai->parameters[0].length() > 0) approachP = Util::toFloat(ai->parameters[0]);
	if (ai->parameters[1].length() > 0) sideP = Util::toFloat(ai->parameters[1]);
	if (ai->parameters[2].length() > 0) nearZeroSpeedAngle = Util::toFloat(ai->parameters[2]);

	float forwardSpeed = Math::min(approachP * Math::map(Math::abs(Math::radToDeg(ball->angle)), 0.0f, nearZeroSpeedAngle, 1.0f, 0.0f), enterVelocity);
	float sideSpeed = sideP * Math::sign(ball->distanceX) * Math::map(Math::abs(Math::radToDeg(ball->angle)), 0.0f, nearMaxSideSpeedAngle, 0.0f, 1.0f);
	
	ai->dbg("forwardSpeed", forwardSpeed);
	ai->dbg("sideSpeed", sideSpeed);
	ai->dbg("enterVelocity", enterVelocity);
	ai->dbg("ballAngle", (Math::radToDeg(ball->angle)));
	ai->dbg("nearZeroSpeedAngle", nearZeroSpeedAngle);
	ai->dbg("ball->distanceX", ball->distanceX);

	robot->dribbler->start();
	robot->setTargetDir(forwardSpeed, sideSpeed);
	robot->lookAt(goal);
}

void TestController::AimState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {
	robot->stop();

	ai->dbg("gotBall", robot->dribbler->gotBall());
	
	if (!robot->dribbler->gotBall()) {
		ai->setState("fetch-ball-front");

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

	float speed = 0.5f;
	float period = 5.0f;

	if (ai->parameters[0].length() > 0) speed = Util::toFloat(ai->parameters[0]);
	if (ai->parameters[1].length() > 0) period = Util::toFloat(ai->parameters[1]);

	float targetAngle = getCircleTargetAngle(0.0f, stateDuration, period);

	robot->setTargetDir(Math::Rad(targetAngle), speed);
	robot->lookAt(goal);
}

float TestController::DriveCircleState::getCircleTargetAngle(float start, float time, float period){
	float omega = 2.0f * Math::PI / period;
	float targetX = sin(omega * time + start);
	float targetY = cos(omega * time + start);
	float targetAngle = atan2(targetY, targetX);

	return targetAngle;
}