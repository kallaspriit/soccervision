#include "TestController.h"

#include "Robot.h"
#include "Dribbler.h"
#include "Command.h"

/**
 * TODO
 * + fetch ball straight and search for goal if lost goal at large angle
 * + search for goal state
 * + search for ball state
 * + avoid kicking through another ball
 * + improve aiming while spinning around dribbler
 * + make it brake less/smarter for fetch front
 * + improve fetch behind not to reconsider so much (complete maneuver?)
 * + decrease fetch direct speed if white-black close (nearby line)
 * + provide cached ball info for some frames if can't see one any more
 * + move while aiming if other robot in the way (can be improved)
 * + do something if searching for a ball for a while (drive the center)
 * + deal with not seeing a ball at distance when starting to fetch it quickly (remembers it for a while)
 * + check if sees ball is out driving in reverse
 * + check whether adaptive fetch front distance is good
 * + can drive out of the field when avoiding to kick through balls
 * + don't drive forward while avoiding balls if next ball is close by
 * + detect that the robot has gone out of the field (both cameras)
 * + create a way to read the actual distance the robot has travelled at any time
 * + use robot distance to calculate how long to drive blind behind the ball
 * + more reliable "ball in goal", check distances?
 * + better reversing out of the corner
 * + differentiate between near line and in corner
 * + reverse only a little near a line, more in corner, approach with care in both cases
 * + don't fake ball in dribbler after kicking
 * + can fetch behind be made faster?
 * + when aiming, turn around dribbler with acceleration and don't move forward or event slightly reverse at the beginning
 * / reverse towards own goal while aiming based on travelledRotation not time
 * + come home state, drives to corner based on localization, white lines (eq side distance approach)
 * + make sure robot doesn't drive into own goal if balls close to it, over line
 * + thinks it's near the line too often when actually not
 * + apply max distance on escape from corner
 * + search: drive straigh until near line white-black, turn 45 degrees, repeat, turn rotating
 * + check turnBy -150.4 degrees
 * + need better fetch ball near
 * + kicks through other balls if other balls very close
 * - make localizer use angle to goal
 * - try accelerating dribbler
 * + better drive-into-goal avoidance
 * / avoid goal collision, sample top quarter area pixels for unsegmented/goal colors, drive left from them
 * + check why fetch behind sometimes messes it up (render in web UI)
 * / cancel fetch ball behind and escape corner if robot out is detected
 * + fake ball in dribbler for more time if escaping corner / always complete it (if not out)
 * - ignore camera image if fetching frame takes a long time
 * + rotate around its axis when was near any goals lately
 * / find ball drive between goals
 * - read and check all logic code
 * + make returning to field seperate state, use in find ball, fetch ball direct
 * + fix find goal, drive towards own when aiming
 * - check if kick window calculaton is reasonable
 *
 *
 * DEMO
 * + fetch string of balls in front
 * + fetch string of balls behind
 * + fetch diagonal set of balls
 * + fetch ball behind near own goal (turn around)
 * + fetch ball from own/opponent goal corner (near both lines)
 * + fetch ball on the line
 * + fetch ball at large angle from own goal, facing own goal
 * - fetch balls from 4 corners (probably includes search)
 * - empty entire field with timer
 * - maximum speed fetching forward/behind
 * - kick ball avoiding a set of other balls
 * - find ball move to center
 * - kick into very small goal
 * - show robot and ball localization in the web UI
 * - show "back in time" in the web UI
 * - show configuring colors in the web UI
 * - avoid opponent
 * - on top of robot
 */

TestController::TestController(Robot* robot, Communication* com) : BaseAI(robot, com), targetSide(Side::BLUE), manualSpeedX(0.0f), manualSpeedY(0.0f), manualOmega(0.0f), manualDribblerSpeed(0), manualKickStrength(0), blueGoalDistance(0.0f), yellowGoalDistance(0.0f), lastCommandTime(-1.0), lastBallTime(-1.0), lastNearLineTime(-1.0), lastInCornerTime(-1.0), lastTargetGoalAngle(0.0f), lastBall(NULL), lastTurnAroundTime(-1.0), lastClosestGoalDistance(-1.0f), lastTargetGoalDistance(-1.0f), framesRobotOutFront(0), framesRobotOutRear(0), isRobotOutFront(false), isRobotOutRear(false), isNearLine(false), isInCorner(false), inCornerFrames(0) {
	setupStates();
};

TestController::~TestController() {
	
}

void TestController::reset() {
	std::cout << "! Reset test-controller" << std::endl;

	com->send("reset");
	targetSide = Side::YELLOW; // will be switched to blue in handleToggleSideCommand()
	totalDuration = 0.0f;
	currentStateDuration = 0.0f;
	currentState = NULL;
	currentStateName = "";

	setState("manual-control");
	handleToggleSideCommand();
}

void TestController::onEnter() {
	reset();
}

void TestController::onExit() {
	reset();
}

void TestController::setupStates() {
	states["manual-control"] = new ManualControlState(this);
	states["watch-ball"] = new WatchBallState(this);
	states["watch-goal"] = new WatchGoalState(this);
	states["watch-goal-behind"] = new WatchGoalBehindState(this);
	states["spin-around-dribbler"] = new SpinAroundDribblerState(this);
	states["drive-to"] = new DriveToState(this);
	states["turn-by"] = new TurnByState(this);
	states["find-ball"] = new FindBallState(this);
	states["fetch-ball-front"] = new FetchBallFrontState(this);
	states["fetch-ball-direct"] = new FetchBallDirectState(this);
	states["fetch-ball-behind"] = new FetchBallBehindState(this);
	states["fetch-ball-near"] = new FetchBallNearState(this);
	states["aim"] = new AimState(this);
	states["drive-circle"] = new DriveCircleState(this);
	states["accelerate"] = new AccelerateState(this);
	states["return-field"] = new ReturnFieldState(this);
	states["escape-corner"] = new EscapeCornerState(this);
	states["drive-home"] = new DriveHomeState(this);
}

void TestController::step(float dt, Vision::Results* visionResults) {
	messages.clear();

	updateVisionInfo(visionResults);

	currentStateDuration += dt;
	combinedStateDuration += dt;
	totalDuration += dt;

	if (currentState != NULL) {
		currentState->step(dt, visionResults, robot, totalDuration, currentStateDuration, combinedStateDuration);
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
    } else if (cmd.name == "reset") {
        handleResetCommand();
    } else if (cmd.name == "toggle-go") {
        handleToggleGoCommand();
    } else if (cmd.name == "toggle-side") {
        handleToggleSideCommand();
    } else if (cmd.name == "drive-to" && cmd.parameters.size() == 3) {
        handleDriveToCommand(cmd);
    } else if (cmd.name == "turn-by" && cmd.parameters.size() == 1) {
        handleTurnByCommand(cmd);
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

void TestController::handleToggleGoCommand() {
	if (!toggleGoBtn.toggle() || currentStateName != "manual-control") {
		return;
	}

	setState("find-ball");
}

void TestController::handleToggleSideCommand() {
	if (!toggleSideBtn.toggle() || currentStateName != "manual-control") {
		return;
	}

	if (targetSide == Side::BLUE) {
		targetSide = Side::YELLOW;

		robot->setPosition(Config::fieldWidth - Config::robotRadius, Config::robotRadius, Math::PI - Math::PI / 8.0f);
	} else {
		targetSide = Side::BLUE;

		robot->setPosition(Config::robotRadius, Config::fieldHeight - Config::robotRadius, -Math::PI / 8.0f);
	}

	std::cout << "! Now targeting " << (targetSide == Side::BLUE ? "blue" : "yellow") << " side" << std::endl;

	com->send("target:" + Util::toString(targetSide));

	lastTurnAroundTime = -1.0;
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

void TestController::handleTurnByCommand(const Command& cmd) {
	TurnByState* state = (TurnByState*)states["turn-by"];

	state->angle = Math::degToRad(Util::toFloat(cmd.parameters[0]));

	setState("turn-by");
}

void TestController::updateVisionInfo(Vision::Results* visionResults) {
	Object* blueGoal = visionResults->getLargestGoal(Side::BLUE);
	Object* yellowGoal = visionResults->getLargestGoal(Side::YELLOW);

	// update the closest goal distance
	if (blueGoal != NULL || yellowGoal != NULL) {
		float currentClosestGoalDistance = -1.0f;

		if (blueGoal != NULL && blueGoal->distanceY > 0.0f && blueGoal->distanceY < Config::fieldWidth && blueGoal->distanceY < 2.0f && (currentClosestGoalDistance == -1.0f || blueGoal->distanceY < currentClosestGoalDistance)) {
			currentClosestGoalDistance = getObjectClosestDistance(visionResults, blueGoal);
		}

		if (yellowGoal != NULL && yellowGoal->distanceY > 0.0f && yellowGoal->distanceY < Config::fieldWidth && yellowGoal->distanceY < 2.0f && (currentClosestGoalDistance == -1.0f || yellowGoal->distanceY < currentClosestGoalDistance)) {
			currentClosestGoalDistance = getObjectClosestDistance(visionResults, yellowGoal);
		}

		if (currentClosestGoalDistance != -1.0f) {
			lastClosestGoalDistanceAvg.add(currentClosestGoalDistance);

			// take some averages, single frame distances flicker
			if (lastClosestGoalDistanceAvg.full()) {
				lastClosestGoalDistance = lastClosestGoalDistanceAvg.value();
			}
		}
	}

	// used just for debugging
	blueGoalDistance = blueGoal != NULL ? blueGoal->distance : 0.0f;
	yellowGoalDistance = yellowGoal != NULL ? yellowGoal->distance : 0.0f;
	whiteDistance = visionResults->front->whiteDistance;
	blackDistance = visionResults->front->blackDistance;

	// update target goal distance
	float currentTargetGoalDistance = -1.0f;

	if (targetSide == Side::BLUE && blueGoal != NULL) {
		// filter goalsdetected very far away
		if (blueGoal->distance > 0.0f && blueGoal->distance <= 6.0f) {
			lastTargetGoalAngle = blueGoal->angle;
		}

		currentTargetGoalDistance = getObjectClosestDistance(visionResults, blueGoal);
	} else if (targetSide == Side::YELLOW && yellowGoal != NULL) {
		if (yellowGoal->distance > 0.0f && yellowGoal->distance <= 6.0f) {
			lastTargetGoalAngle = yellowGoal->angle;
		}

		currentTargetGoalDistance = getObjectClosestDistance(visionResults, yellowGoal);
	}

	if (currentTargetGoalDistance != -1.0f && currentTargetGoalDistance > 0.0f && currentTargetGoalDistance < Config::fieldWidth) {
		lastTargetGoalDistanceAvg.add(currentTargetGoalDistance);

		if (lastTargetGoalDistanceAvg.full()) {
			lastTargetGoalDistance = lastTargetGoalDistanceAvg.value();
		}
	}

	int robotOutFramesThreshold = 10;

	// check whether robot is detected to be out for some frames
	if (visionResults->isRobotOut(Dir::FRONT)) {
		framesRobotOutFront = (int)Math::min((float)(framesRobotOutFront + 1), (float)robotOutFramesThreshold);
	} else {
		framesRobotOutFront = (int)Math::max((float)(framesRobotOutFront - 1), 0.0f);
	}

	if (visionResults->isRobotOut(Dir::REAR)) {
		framesRobotOutRear = (int)Math::min((float)(framesRobotOutRear + 1), (float)robotOutFramesThreshold);
	} else {
		framesRobotOutRear = (int)Math::max((float)(framesRobotOutRear - 1), 0);
	}

	isRobotOutFront = framesRobotOutFront >= robotOutFramesThreshold;
	isRobotOutRear = framesRobotOutRear >= robotOutFramesThreshold;

	// store whether robot is near line or in corner
	isNearLine = isRobotNearLine(visionResults);
	isInCorner = isRobotInCorner(visionResults);

	if (isNearLine) {
		lastNearLineTime = Util::millitime();
	}

	int robotInCornerFramesThreshold = 3;

	if (isInCorner) {
		inCornerFrames = (int)Math::min((float)(inCornerFrames + 1), (float)robotInCornerFramesThreshold);
	} else {
		inCornerFrames = (int)Math::max((float)(inCornerFrames - 1), 0);
	}

	if (inCornerFrames >= 3) {
		lastInCornerTime = Util::millitime();
	}
}

bool TestController::isRobotNearLine(Vision::Results* visionResults, bool ignoreCenterSample) {
	float nearLineDistance = 0.6f;
	float whiteMin = visionResults->front->whiteDistance.min;
	float blackMin = visionResults->front->blackDistance.min;

	if (ignoreCenterSample) {
		// recalculate minimum distances without the center sample which may be a white spot on the ball
		whiteMin = -1.0f;
		blackMin = -1.0f;

		if (visionResults->front->whiteDistance.left != -1.0f && (whiteMin == -1.0f || visionResults->front->whiteDistance.left < whiteMin)) whiteMin = visionResults->front->whiteDistance.left;
		if (visionResults->front->whiteDistance.leftMiddle != -1.0f && (whiteMin == -1.0f || visionResults->front->whiteDistance.leftMiddle < whiteMin)) whiteMin = visionResults->front->whiteDistance.leftMiddle;
		if (visionResults->front->whiteDistance.rightMiddle != -1.0f && (whiteMin == -1.0f || visionResults->front->whiteDistance.rightMiddle < whiteMin)) whiteMin = visionResults->front->whiteDistance.rightMiddle;
		if (visionResults->front->whiteDistance.right != -1.0f && (whiteMin == -1.0f || visionResults->front->whiteDistance.right < whiteMin)) whiteMin = visionResults->front->whiteDistance.right;

		if (visionResults->front->blackDistance.left != -1.0f && (blackMin == -1.0f || visionResults->front->blackDistance.left < blackMin)) blackMin = visionResults->front->blackDistance.left;
		if (visionResults->front->blackDistance.leftMiddle != -1.0f && (blackMin == -1.0f || visionResults->front->blackDistance.leftMiddle < blackMin)) blackMin = visionResults->front->blackDistance.leftMiddle;
		if (visionResults->front->blackDistance.rightMiddle != -1.0f && (blackMin == -1.0f || visionResults->front->blackDistance.rightMiddle < blackMin)) blackMin = visionResults->front->blackDistance.rightMiddle;
		if (visionResults->front->blackDistance.right != -1.0f && (blackMin == -1.0f || visionResults->front->blackDistance.right < blackMin)) blackMin = visionResults->front->blackDistance.right;
	}

	// the last two conditions may not be true if there's a ball in the way
	return whiteMin != -1.0f && whiteMin < nearLineDistance
		&& blackMin != -1.0f && blackMin < nearLineDistance;
		//&& visionResults->front->whiteDistance.min < visionResults->front->blackDistance.min
		//&& visionResults->front->blackDistance.min - visionResults->front->whiteDistance.min <= 0.1f;
};

bool TestController::isRobotInCorner(Vision::Results* visionResults) {
	float tooCloseDistance = 0.3f;
	
	if (!isRobotNearLine(visionResults)) {
		return false;
	}

	// too close to the line, it's impossible to make the difference
	if (visionResults->front->whiteDistance.min < tooCloseDistance) {
		return false;
	}

	// probably a straight line if the difference is very small
	if (visionResults->front->whiteDistance.max - visionResults->front->whiteDistance.min < 0.02f) {
		return false;
	}

	// if the maximum is far, we can be looking at the corner near the line and large distance, dont consider this valid
	if (visionResults->front->whiteDistance.max > tooCloseDistance * 3.0f) {
		return false;
	}

	// dont perform corner maneuvers when the robot is out
	if (isRobotOutFront || isRobotOutRear) {
		return false;
	}

	// not in corner if black is closer than black
	// cant be trusted because black is found on ball
	/*if (
		visionResults->front->whiteDistance.min != -1.0f
		&& visionResults->front->blackDistance.min != -1.0f
		&& visionResults->front->blackDistance.min < visionResults->front->whiteDistance.min
	) {
		return false;
	}*/

	// we need left and right samples
	if (visionResults->front->whiteDistance.left == -1.0f || visionResults->front->whiteDistance.right == -1.0f) {
		return false;
	}

	// calculate maximum color distances at corners
	float maxCornerDistanceWhite = Math::max(visionResults->front->whiteDistance.left, visionResults->front->whiteDistance.right);
	float maxCornerDistanceBlack = Math::max(visionResults->front->blackDistance.left, visionResults->front->blackDistance.right);

	// robot is in corner if any of the center 3 samples are further than the furthest side samples (check both white and black)
	if (
		(
			(visionResults->front->whiteDistance.leftMiddle != -1.0f && visionResults->front->whiteDistance.leftMiddle > maxCornerDistanceWhite)
			|| (visionResults->front->whiteDistance.center != -1.0f && visionResults->front->whiteDistance.center > maxCornerDistanceWhite)
			|| (visionResults->front->whiteDistance.rightMiddle != -1.0f && visionResults->front->whiteDistance.rightMiddle > maxCornerDistanceWhite)
		) && (
			(visionResults->front->blackDistance.leftMiddle != -1.0f && visionResults->front->blackDistance.leftMiddle > maxCornerDistanceBlack)
			|| (visionResults->front->blackDistance.center != -1.0f && visionResults->front->blackDistance.center > maxCornerDistanceBlack)
			|| (visionResults->front->blackDistance.rightMiddle != -1.0f && visionResults->front->blackDistance.rightMiddle > maxCornerDistanceBlack)
		)
	) {
		return true;
	}

	return false;
};

bool TestController::isRobotNearGoal(float threshold) {
	return lastClosestGoalDistance != -1.0f && lastClosestGoalDistance < threshold;
}

bool TestController::isRobotNearTargetGoal(float threshold) {
	return lastTargetGoalDistance != -1.0f && lastTargetGoalDistance < threshold;
}

bool TestController::wasNearLineLately(double threshold) {
	return lastNearLineTime != -1.0 && Util::duration(lastNearLineTime) < threshold;
}

bool TestController::wasInCornerLately(double threshold) {
	return lastInCornerTime != -1.0 && Util::duration(lastInCornerTime) < threshold;
}

void TestController::resetLastBall() {
	if (lastBall != NULL) {
		delete lastBall;

		lastBall = NULL;
		lastBallTime = -1.0;
	}
}

void TestController::setLastBall(Object* ball) {
	resetLastBall();

	lastBall = new Object();
	lastBall->copyFrom(ball);

	lastBallTime = Util::millitime();
}

Object* TestController::getLastBall(Dir dir) {
	// only return last seen ball if its fresh enough
	if (lastBall == NULL || lastBallTime == -1.0 || Util::duration(lastBallTime) > 0.25) {
		return NULL;
	}

	// make sure the ball is on the right side
	if (dir != Dir::ANY && (lastBall->behind && dir == Dir::FRONT) || (!lastBall->behind && dir == Dir::REAR)) {
		return NULL;
	}

	// use this only for balls far away as they're likely to be badly visible
	if (lastBall->distance < 1.0f) {
		return NULL;
	}

	return lastBall;
}

float TestController::getObjectClosestDistance(Vision::Results* visionResults, Object* object) {
	// calculate the closest object bottom distance, corners might be closer than center
	Vision::Distance leftDistance = visionResults->front->vision->getDistance(object->x - object->width / 2, object->y + object->height / 2);
	Vision::Distance rightDistance = visionResults->front->vision->getDistance(object->x + object->width / 2, object->y + object->height / 2);
		
	return Math::min(Math::min(leftDistance.y, rightDistance.y), object->distanceY);
}

std::string TestController::getJSON() {
	std::stringstream stream;

	stream << "{";
	
	for (MessagesIt it = messages.begin(); it != messages.end(); it++) {
		stream << "\"" << (it->first) << "\": \"" << (it->second) << "\",";
	}

	//send some debug information to the client
	stream << "\"currentState\": \"" << currentStateName << "\",";
	stream << "\"stateDuration\": \"" << currentStateDuration << "\",";
	stream << "\"combinedDuration\": \"" << combinedStateDuration << "\",";
	stream << "\"totalDuration\": \"" << totalDuration << "\",";
	stream << "\"realSpeed\": \"" << robot->getVelocity() << "\",";
	stream << "\"travelledDistance\": \"" << robot->getTravelledDistance() << "\",";
	stream << "\"travelledTurns\": \"" << (robot->getTravelledRotation() / Math::TWO_PI) << "\",";
	stream << "\"targetSide\": \"" << (targetSide == Side::BLUE ? "blue" : targetSide == Side::YELLOW ? "yellow" : "not chosen") << "\",";
	stream << "\"whiteDistance\": " << whiteDistance.min << ",";
	stream << "\"blackDistance\": " << blackDistance.min << ",";
	stream << "\"blueGoalDistance\": " << blueGoalDistance << ",";
	stream << "\"yellowGoalDistance\": " << yellowGoalDistance << ",";
	stream << "\"lastClosestGoalDistance\": " << lastClosestGoalDistance << ",";
	stream << "\"lastTargetGoalDistance\": " << lastTargetGoalDistance << ",";
	stream << "\"isRobotOutFront\": " << (isRobotOutFront ? "true" : "false") << ",";
	stream << "\"isRobotOutRear\": " << (isRobotOutRear ? "true" : "false") << ",";
	stream << "\"isInCorner\": " << (isInCorner ? "true" : "false") << ",";
	stream << "\"wasInCornerLately\": " << (wasInCornerLately() ? "\"true: " + Util::toString(Util::duration(lastInCornerTime)) + "\"" : "false") << ",";
	stream << "\"isNearLine\": " << (isNearLine ? "true" : "false") << ",";
	stream << "\"lastTargetGoalAngle\": " << Math::radToDeg(lastTargetGoalAngle);

	stream << "}";

	return stream.str();
}

// calculates the angle to approach the ball at to get behind it or to the side of it
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

	return atan2(targetX, targetY);
}

void TestController::ManualControlState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	double time = Util::millitime();

	// failsafe stops movement if no new commands received for some time
	if (ai->lastCommandTime == -1.0 || time - ai->lastCommandTime < 0.5) {
		robot->setTargetDir(ai->manualSpeedX, ai->manualSpeedY, ai->manualOmega);
		robot->dribbler->setTargetSpeed(-ai->manualDribblerSpeed);

		if (ai->manualKickStrength != 0.0f) {
			robot->kick(ai->manualKickStrength);

			ai->manualKickStrength = 0;
		}
	} else {
		robot->stop();
		robot->dribbler->stop();
	}
}

void TestController::WatchBallState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	Object* ball = visionResults->getClosestBall(Dir::FRONT);

	if (ball == NULL) {
		robot->setTargetDir(ai->manualSpeedX, ai->manualSpeedY, ai->manualOmega);

		return;
	}

	robot->setTargetDir(ai->manualSpeedX, ai->manualSpeedY);
	robot->lookAt(ball);
}

void TestController::WatchGoalState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	Object* goal = visionResults->getLargestGoal(ai->targetSide, Dir::FRONT);

	if (goal == NULL) {
		robot->setTargetDir(ai->manualSpeedX, ai->manualSpeedY, ai->manualOmega);

		return;
	}

	robot->setTargetDir(ai->manualSpeedX, ai->manualSpeedY);
	robot->lookAt(goal);
}

void TestController::WatchGoalBehindState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	Object* goal = visionResults->getLargestGoal(ai->targetSide, Dir::REAR);

	if (goal == NULL) {
		robot->setTargetDir(ai->manualSpeedX, ai->manualSpeedY, ai->manualOmega);

		return;
	}

	robot->setTargetDir(ai->manualSpeedX, ai->manualSpeedY);
	robot->lookAtBehind(goal);
}

void TestController::SpinAroundDribblerState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	robot->spinAroundDribbler();
}

void TestController::DriveToState::onEnter(Robot* robot, Parameters parameters) {
	robot->driveTo(x, y, orientation);
}

void TestController::DriveToState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	if (!robot->hasTasks()) {
		ai->setState("manual-control");
	}
}

void TestController::TurnByState::onEnter(Robot* robot, Parameters parameters) {
	robot->turnBy(angle, Math::PI);
}

void TestController::TurnByState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	if (!robot->hasTasks()) {
		ai->setState("manual-control");
	}
}

void TestController::FindBallState::onEnter(Robot* robot, Parameters parameters) {
	// accepts parameter to set which direction to start searching at
	if (parameters.find("search-dir") != parameters.end()) {
		searchDir = Util::toFloat(parameters["search-dir"]);
	} else {
		// otherwise the search direction is based on last seen target goal angle to turn the shortest path
		if (ai->lastTargetGoalAngle > 0.0f) {
			searchDir = 1.0f;
		} else {
			searchDir = -1.0f;
		}
	}

	// reset runtime parameters
	nearBothFrames = 0;
	wasSearchingRecently = false;
	focusedOnGoal = false;
	queuedApproachGoal = false;
	timeSinceLastSearch = -1.0;

	if (lastSearchTime != -1.0) {
		timeSinceLastSearch = Util::duration(lastSearchTime);

		if (timeSinceLastSearch < 0.2) {
			wasSearchingRecently = true;
		}
	}
}

void TestController::FindBallState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	robot->stop();
	
	lastSearchTime = Util::millitime();
	
	// switch to aim if got ball
	if (robot->dribbler->gotBall()) {
		robot->dribbler->start();

		ai->setState("aim");

		return;
	}

	Dir ballSearchDir = Dir::ANY;

	// can introduce ignoring balls nearby
	/*if (ai->lastTurnAroundTime != -1.0 && Util::duration(ai->lastTurnAroundTime) < 2.0) {
		// search from front if just performed turn around towards ball behind
		ballSearchDir = Dir::FRONT;

		ai->dbgs("search", "front due to recent turn around");
	} else if (wasSearchingRecently) {
		if (ai->lastStateName == "fetch-ball-behind") {
			// search from front if recently fetching from behind failed
			ballSearchDir = Dir::FRONT;

			ai->dbgs("search", "front due recent search from rear");
		} else {
			ballSearchDir = Dir::REAR;

			ai->dbgs("search", "rear due recent search from front");
		}
	} else {
		ai->dbgs("search", "any closest");
	}*/

	Object* ball = visionResults->getClosestBall(ballSearchDir);
	Object* goal = visionResults->getLargestGoal(ai->targetSide, Dir::FRONT);

	if (ball == NULL && ballSearchDir != Dir::ANY && stateDuration > 1.0f) {
		ball = visionResults->getClosestBall(Dir::ANY);
	}

	// store last seen ball
	if (ball != NULL) {
		ai->setLastBall(ball);
	}

	ai->dbg("ballSearchDir", ballSearchDir);
	ai->dbg("wasSearchingRecently", wasSearchingRecently);
	ai->dbg("timeSinceLastTurnAround", ai->lastTurnAroundTime != -1.0 ? Util::duration(ai->lastTurnAroundTime) : -1.0);
	ai->dbg("hasTasks", robot->hasTasks());
	ai->dbg("timeSinceLastSearch", timeSinceLastSearch);
	ai->dbg("ballVisible", ball != NULL);
	ai->dbg("goalVisible", goal != NULL);
	ai->dbg("searchDir", searchDir);
	ai->dbg("focusedOnGoal", focusedOnGoal);

	double minTurnBreak = 2.0;
	float searchPeriod = 2.0f;
	float searchOmega = Math::TWO_PI / searchPeriod;

	if (ball != NULL) {
		if (robot->hasTasks()) {
			robot->clearTasks();
		}

		ai->dbg("ballBehind", ball->behind);
		ai->dbg("ballDistance", ball->getDribblerDistance());
		ai->dbg("ballAngle", Math::radToDeg(ball->angle));

		if (!ball->behind) {
			// switch to fetching ball is ball found in front
			if (goal != NULL) {
				ai->setState("fetch-ball-front");
			} else {
				ai->setState("fetch-ball-direct");
			}
		} else if (ai->lastTurnAroundTime == -1.0 || Util::duration(ai->lastTurnAroundTime) > minTurnBreak) {
			if (goal != NULL) {
				// ball behind and target goal in front, fetch it from behind
				ai->setState("fetch-ball-behind");
			} else {
				// ball found behind but no goal visible, turn around
				float turnAngle = ball->angle;
				float underturnAngle = Math::degToRad(15.0f);
				float turnSpeed = Math::TWO_PI;

				// turn slightly less than actual ball angle, inertia can take further
				if (turnAngle < 0.0f) {
					turnAngle += underturnAngle;
					searchDir = -1.0f;
				} else {
					turnAngle -= underturnAngle;
					searchDir = 1.0f;
				}

				ai->dbg("turnAngle", Math::radToDeg(turnAngle));
				ai->dbg("turnSpeed", turnSpeed);
				ai->dbg("searchDir", searchDir);

				// start turn by angle task
				robot->turnBy(turnAngle, turnSpeed);

				ai->lastTurnAroundTime = Util::millitime();
			}

			return;
		} else {
			// ball is behind but turn around already executed lately, keep on turning
			robot->setTargetOmega(searchOmega * searchDir);
		}
	} else {
		// robot is out, return to the playing area
		if (ai->isRobotOutFront || ai->isRobotOutRear) {
			ai->setState("return-field");

			return;
		}

		// wait until tasks complete
		if (robot->hasTasks()) {
			return;
		}

		// just spin for half a search period, two cameras should cover the whole field
		if (stateDuration < searchPeriod / 2.0f) {
			robot->setTargetOmega(searchOmega * searchDir);

			return;
		}

		// first turn towards one of the goals to get started in the right direction
		if (!focusedOnGoal) {
			Object* goal = visionResults->getLargestGoal(Side::UNKNOWN, Dir::FRONT);
			
			// only consider goals far away not to drive into own goal if next to it
			if (goal != NULL && goal->distance > Config::fieldWidth / 3.0f) {
				ai->dbg("focusGoalType", goal->type);

				robot->lookAt(goal);

				if (goal->angle > 0.0f) {
					searchDir = 1.0f;
				} else {
					searchDir = -1.0f;
				}

				// consider maneuver complete if angle to the goal is small enough
				if (Math::abs(goal->angle) < Math::degToRad(5.0f)) {
					focusedOnGoal = true;
				}
			} else {
				// no goal found, spin
				robot->setTargetOmega(searchDir * Math::PI);

				return;
			}
		}

		// drive forwards until near a line, then bounce off it, covering most of the field in some time
		float nearLineDistance = 0.5f;
		float omegaP = Math::PI;
		float forwardP = 1.5f;
		float omegaPower = 0.0f;
		float omega = 0.0f;

		float leftLine = -1.0f;
		float rightLine = -1.0f;

		// calculate distances to the lines on the sides
		if (
			visionResults->front->whiteDistance.left != -1.0f
			&& visionResults->front->blackDistance.left != -1.0f
		) {
			leftLine = (visionResults->front->whiteDistance.left + visionResults->front->blackDistance.left) / 2.0f;
		}

		if (
			visionResults->front->whiteDistance.right != -1.0f
			&& visionResults->front->blackDistance.right != -1.0f
		) {
			rightLine = (visionResults->front->whiteDistance.right + visionResults->front->blackDistance.right) / 2.0f;
		}

		if (leftLine != -1.0f || rightLine != -1.0f) {
			if (
				leftLine != -1.0f && leftLine < nearLineDistance
				&& rightLine != -1.0f && rightLine < nearLineDistance
			) {
				nearBothFrames++;
			} else {
				nearBothFrames = 0;
			}

			if (nearBothFrames > 10) {
				// we're probably in a corner, turn around (almost)
				robot->turnBy(Math::degToRad(searchDir * 135.0f), Math::TWO_PI);

				nearBothFrames = 0;

				return;
			} else {
				// calculate the turning power based on how close we're to the lines
				if (leftLine != -1.0f && (rightLine == -1.0f || leftLine < rightLine)) {
					omegaPower = Math::map(leftLine, nearLineDistance, nearLineDistance * 2.0f, 1.0f, 0.0f);
				} else {
					omegaPower = Math::map(rightLine, nearLineDistance, nearLineDistance * 2.0f, 1.0f, 0.0f);
				}

				omega = searchDir * omegaPower * omegaP;
			}
		}

		// slow down a bit when turning
		float forwardSpeed = forwardP * (1.0f - omegaPower);

		robot->setTargetDir(forwardSpeed, 0.0f, omega);

		ai->dbg("leftLine", leftLine);
		ai->dbg("rightLine", rightLine);
		ai->dbg("omegaPower", omegaPower);
		ai->dbg("nearBothFrames", nearBothFrames);
		ai->dbg("forwardSpeed", forwardSpeed);
		ai->dbg("omega", omega);
	}
}

void TestController::FetchBallFrontState::onEnter(Robot* robot, Parameters parameters) {
	reset(robot);
}

// resets decision properties
void TestController::FetchBallFrontState::reset(Robot* robot) {
	forwardSpeed = robot->getVelocity();
	startBrakingDistance = -1.0f;
	startBrakingVelocity = -1.0f;
	lastBallDistance = -1.0f;
	lastTargetAngle = 0.0f;
}

void TestController::FetchBallFrontState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	robot->stop();
	
	if (robot->dribbler->gotBall()) {
		robot->dribbler->start();

		ai->setState("aim");

		return;
	}

	// only consider balls in the front camera
	Object* ball = visionResults->getClosestBall(Dir::FRONT);
	Object* goal = visionResults->getLargestGoal(ai->targetSide, Dir::FRONT);

	bool usingGhost = false;

	if (ball != NULL) {
		ai->setLastBall(ball);
	} else {
		// use ghost ball if available
		ball = ai->getLastBall(Dir::FRONT);

		if (ball != NULL) {
			usingGhost = true;
		}
	}

	ai->dbg("ballVisible", ball != NULL);
	ai->dbg("goalVisible", goal != NULL);
	ai->dbg("usingGhost", usingGhost);

	// we can see a ball but not the goal, switch to direct fetch
	if (goal == NULL && ball != NULL) {
		ai->setState("fetch-ball-direct");

		return;
	}

	// can't see the ball any more, switch to searching for it
	if (ball == NULL) {
		Parameters parameters;

		// start searching the ball on the side that it was lost at
		if (lastTargetAngle > 0.0f) {
			parameters["search-dir"] = "1.0f";
		} else {
			parameters["search-dir"] = "-1.0f";
		}

		ai->setState("find-ball", parameters);

		return;
	}

	// configuration parameters
	float targetApproachSpeed = 3.5f;
	float brakingApproachSpeed = 1.5f;
	float maxNearSpeed = 0.75f;
	float maxOffsetDistanceAngleDiff = 45.0f;
	float maxAngleDiffDistance = 0.6f;
	float focusBetweenBallGoalAngle = 15.0f;
	float maxAngleBrakingAngle = 40.0f;
	float maxBallBrakingAngle = 10.0f;
	float minApproachSpeed = 0.75f;
	float nearDistance = 0.35f;
	float accelerateAcceleration = 3.5f;
	float brakeAcceleration = 3.0f;
	float retratingBallDistanceDiff = 0.2f;
	float offsetDistance = 0.25f;

	float ballDistance = ball->getDribblerDistance();
	float realSpeed = robot->getVelocity();
	float ballAngle = ball->angle;
	float goalAngle = goal->angle;
	float angleDiff = Math::abs(goalAngle - ballAngle);
	float adaptiveBrakingDistance = Math::getAccelerationDistance(realSpeed, 0.0f, brakeAcceleration);

	// reset if we probably started to watch a ball we just kicked
	if (lastBallDistance != -1.0f && ballDistance - lastBallDistance > retratingBallDistanceDiff) {
		reset(robot);
	}

	// ball is close, switch to fetching ball from near state
	if (ballDistance < nearDistance) {
		ai->setState("fetch-ball-near");
		
		return;
	}
	
	// only choose the braking distance once
	if (startBrakingDistance == -1.0f && ballDistance < adaptiveBrakingDistance) {
		startBrakingDistance = adaptiveBrakingDistance;
		startBrakingVelocity = robot->getVelocity();
	}

	// calculate the target angle to drive behind the ball, facing the target goal
	float targetAngle = ai->getTargetAngle(goal->distanceX, goal->distanceY, ball->distanceX, ball->distanceY, offsetDistance);
	bool braking = startBrakingDistance != -1.0f && ballDistance < startBrakingDistance;

	// accelerate in the beginning
	float limitedSpeed = forwardSpeed = Math::getAcceleratedSpeed(forwardSpeed, braking ? brakingApproachSpeed : targetApproachSpeed, dt, accelerateAcceleration);

	// brake as getting close and when using large target angle
	if (braking) {
		float distanceBraking = Math::map(ballDistance, nearDistance, startBrakingDistance, 1.0, 0.0f);
		float targetAngleBreaking = Math::map(Math::abs(targetAngle), 0.0f, Math::degToRad(maxAngleBrakingAngle), 0.0f, 1.0f);
		float ballAngleBreaking = Math::map(Math::abs(ballAngle), 0.0f, Math::degToRad(maxBallBrakingAngle), 0.0f, 1.0f);
		float combinedBrakeFactor = distanceBraking * ((targetAngleBreaking + ballAngleBreaking) / 2.0f);
		
		// limit max speed near the ball
		float maxSpeed = Math::map(ballDistance, nearDistance, startBrakingDistance, maxNearSpeed, startBrakingVelocity);
		
		// apply the braking components
		limitedSpeed = limitedSpeed * (1.0f - combinedBrakeFactor);
		forwardSpeed = Math::min(forwardSpeed, maxSpeed);
		limitedSpeed = Math::max(limitedSpeed, minApproachSpeed);

		ai->dbg("distanceBraking", distanceBraking);
		ai->dbg("targetAngleBreaking", targetAngleBreaking);
		ai->dbg("ballAngleBreaking", ballAngleBreaking);
		ai->dbg("combinedBrakeFactor", combinedBrakeFactor);
	}
	
	// focus between the ball and goal at first and more on the goal as getting closer to it so there wouldn't be a sudden switch in the end
	float lookAngle = Math::map(ballDistance, nearDistance, maxAngleDiffDistance, goal->angle, (goal->angle + ball->angle) / 2.0f);

	// don't really need the dribbler before actually got the ball
	/*if (ballDistance < nearDistance) {
		robot->dribbler->start();
	}*/

	// move at calculated target angle, focusing on the ball and goal
	robot->setTargetDir(Math::Rad(targetAngle), limitedSpeed);
	robot->lookAt(Math::Rad(lookAngle));

	lastBallDistance = ballDistance;
	lastTargetAngle = targetAngle;

	ai->dbg("forwardSpeed", forwardSpeed);
	ai->dbg("limitedSpeed", limitedSpeed);
	ai->dbg("realSpeed", robot->getVelocity());
	ai->dbg("startBrakingDistance", startBrakingDistance);
	ai->dbg("startBrakingVelocity", startBrakingVelocity);
	ai->dbg("ballDistance", ballDistance);
	ai->dbg("ballAngle", Math::radToDeg(ballAngle));
	ai->dbg("goalAngle", Math::radToDeg(goalAngle));
	ai->dbg("targetAngle", Math::radToDeg(targetAngle));
	ai->dbg("angleDiff", Math::radToDeg(angleDiff));
	ai->dbg("offsetDistance", offsetDistance);
	ai->dbg("nearDistance", nearDistance);
	ai->dbg("lookAngle", Math::radToDeg(lookAngle));
}

void TestController::FetchBallDirectState::onEnter(Robot* robot, Parameters parameters) {
	forwardSpeed = robot->getVelocity();
	nearLineFrames = 0;
	nearLine = false;
}

void TestController::FetchBallDirectState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	robot->stop();
	
	if (robot->dribbler->gotBall()) {
		ai->dbg("gotBall", true);

		robot->dribbler->start();

		// tell the aim state that we were near the line when approaching, might not see that close by
		Parameters parameters;

		if (nearLine) {
			parameters["near-line"] = "1";
		}

		ai->setState("aim", parameters);

		return;
	}

	// return to the field if robot is out
	if (ai->isRobotOutFront || ai->isRobotOutRear) {
		ai->setState("return-field");

		return;
	}

	// find ball and goal in front
	Object* ball = visionResults->getClosestBall(Dir::FRONT);
	Object* goal = visionResults->getLargestGoal(ai->targetSide, Dir::FRONT);

	// use ghost ball if available
	if (ball != NULL) {
		ai->setLastBall(ball);
	} else {
		ball = ai->getLastBall(Dir::FRONT);
	}

	ai->dbg("ballVisible", ball != NULL);
	ai->dbg("goalVisible", goal != NULL);

	float minFetchDirectDuration = 0.5f;

	// switch to fetching from front using goal if both are visible, don't switch too quickly
	if (ball != NULL && goal != NULL && !ball->behind && !goal->behind && stateDuration > minFetchDirectDuration) {
		ai->setState("fetch-ball-front");

		return;
	}

	// don't have ball, find it
	if (ball == NULL) {
		ai->setState("find-ball");

		return;
	}

	// configuration parameters
	float targetApproachSpeed = 3.5f;
	float minApproachSpeed = 0.3f;
	float accelerateAcceleration = 2.8f;
	float brakeAcceleration = 2.5f;
	float nearLineSpeed = 0.25f;
	float nearBallDistance = 0.3f;
	float realSpeed = robot->getVelocity();
	float ballDistance = ball->getDribblerDistance();
	float brakeDistance = Math::getAccelerationDistance(forwardSpeed, 0.0f, brakeAcceleration);

	// apply braking near the ball
	if (ballDistance < brakeDistance) {
		float brakingAcceleration = Math::getAcceleration(forwardSpeed, 0.0f, brakeDistance);

		targetApproachSpeed = forwardSpeed + brakingAcceleration * dt;

		ai->dbg("brakingAcceleration", brakingAcceleration);

		// don't really need the dribbler
		//robot->dribbler->start();
	}

	// accelerate towards the ball, apply minimum speed
	forwardSpeed = Math::max(Math::getAcceleratedSpeed(forwardSpeed, targetApproachSpeed, dt, accelerateAcceleration), minApproachSpeed);

	// limit the speed low near the white-black line to avoid driving the ball out
	if (nearLine || ai->isRobotNearLine(visionResults, true)) {
		nearLineFrames++;

		// the robot needs to be detected near the line for several frames to be considered true
		if (nearLineFrames >= 10) {
			nearLine = true;
		}

		// limit ball near line
		if (nearLine && ballDistance < nearBallDistance) {
			forwardSpeed = nearLineSpeed;

			ai->dbg("lineLimited", true);
		}
	}

	robot->setTargetDir(forwardSpeed, 0.0f);
	robot->lookAt(ball);

	if (ballDistance < nearBallDistance) {
		robot->dribbler->prime();
	}

	ai->dbg("realSpeed", realSpeed);
	ai->dbg("nearLineFrames", nearLineFrames);
	ai->dbg("nearLine", nearLine);
	ai->dbg("ballDistance", ballDistance);
	ai->dbg("brakeDistance", brakeDistance);
	ai->dbg("targetApproachSpeed", targetApproachSpeed);
	ai->dbg("forwardSpeed", forwardSpeed);
	ai->dbg("brakeAcceleration", brakeAcceleration);
	ai->dbg("accelerateAcceleration", accelerateAcceleration);
	ai->dbg("dt", dt);
}

void TestController::FetchBallBehindState::onEnter(Robot* robot, Parameters parameters) {
	hadBall = false;
	reversePerformed = false;
	turnAroundPerformed = false;
	lastTargetAngle = 0.0f;
	lastBallDistance = -1.0f;
	lostBallTime = -1.0;
	timeSinceLostBall = 0.0;
	lostBallVelocity = 0.0f;
	startBallDistance = -1.0f;
	searchDir = 0.0f;
	targetMode = TargetMode::UNDECIDED;
	forwardSpeed = robot->getVelocity();
	avgBallGoalDistance.clear();
}

void TestController::FetchBallBehindState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	robot->stop();

	if (robot->dribbler->gotBall()) {
		ai->dbg("gotBall", true);

		robot->dribbler->start();

		ai->setState("aim");

		return;
	}

	if (robot->hasTasks()) {
		// even when executing driving behind ball, make sure we don't reverse into our own goal
		Side ownSide = ai->targetSide == Side::YELLOW ? Side::BLUE : Side::YELLOW;
		Object* ownGoal = visionResults->getLargestGoal(ownSide, Dir::REAR);

		if (ownGoal != NULL && ownGoal->distance < 0.3f) {
			robot->clearTasks();

			// own goal was close, start searching for new ball
			ai->setState("find-ball");
		}

		ai->dbg("ownGoalDistance", ownGoal != NULL ? ownGoal->distance : -1.0f);

		return;
	}

	// switch to searching for ball after reverse maneuver has been performed
	if (reversePerformed) {
		// start searching for ball
		ai->setState("find-ball");

		return;
	}

	float reverseBlindSpeed = 1.25f;
	float offsetDistance = 0.2f;

	// one possibility would be to fetch the ball furthest away
	//Object* ball = visionResults->getFurthestBall(Dir::REAR);

	// watch for ball behind and target side goal in front
	Object* ball = visionResults->getClosestBall(Dir::REAR);
	Object* goal = visionResults->getLargestGoal(ai->targetSide, Dir::FRONT);

	ai->dbg("ballVisible", ball != NULL);
	ai->dbg("goalVisible", goal != NULL);
	ai->dbg("hasTasks", robot->hasTasks());
	ai->dbg("timeSinceLostBall", timeSinceLostBall);
	ai->dbg("startBallDistance", startBallDistance);
	ai->dbg("lastBallDistance", lastBallDistance);
	ai->dbg("hadBall", hadBall);
	ai->dbg("reversePerformed", reversePerformed);

	// don't continue if target goal not visible
	if (goal == NULL) {
		if (ball != NULL && !ball->behind) {
			ai->setState("fetch-ball-direct");
		} else {
			// switch to searching for balls, search in the same direction as approached ball behind
			Parameters parameters;

			if (searchDir != 0.0f) {
				parameters["search-dir"] = Util::toString(searchDir);
			}

			ai->setState("find-ball", parameters);
		}

		return;
	}

	// perform reverse if ball is not visible or suddenly seeing another ball further away
	if (
		ball == NULL
		|| (lastBallDistance != -1.0f && ball->getDribblerDistance() > lastBallDistance * 1.25f)
	) {
		// don't perform the blind reverse if the ball was lost at too great of a distance
		if (!hadBall || lastBallDistance > 0.8f) {
			// switch to searching for balls, search in the same direction as approached ball behind
			Parameters parameters;

			if (searchDir != 0.0f) {
				parameters["search-dir"] = Util::toString(searchDir);
			}

			ai->setState("find-ball", parameters);
		} else {
			// perform the blind reverse maneuver
			reversePerformed = true;

			robot->driveBehindBall(lastBallDistance, lastTargetAngle, reverseBlindSpeed, offsetDistance * 1.0f, targetMode == TargetMode::LEFT ? 1.0f : -1.0f);
		}

		return;
	}

	float ballDistance = ball->getDribblerDistance();

	hadBall = true;
	timeSinceLostBall = 0.0;
	lostBallTime = -1.0;
	lastBallDistance = ballDistance;

	// store ball initial distance
	if (startBallDistance == -1.0f) {
		startBallDistance = ballDistance;
	}

	// decide on which side to approach it, use the side already more direct
	// TODO Maybe not go outside near line?
	if (targetMode == TargetMode::UNDECIDED) {
		if (ball->angle + goal->angle > 0.0f) {
			targetMode = TargetMode::LEFT;
			searchDir = 1.0f;
		} else {
			targetMode = TargetMode::RIGHT;
			searchDir = -1.0f;
		}
	}

	ai->dbg("ballDistance", ballDistance);
	ai->dbg("targetMode", targetMode);

	// find our own goal
	Side ownSide = ai->targetSide == Side::YELLOW ? Side::BLUE : Side::YELLOW;
	Object* ownGoal = visionResults->getLargestGoal(ownSide, Dir::REAR);

	// make sure we don't reverse into our own goal
	if (ownGoal != NULL) {
		// calculate distance between the ball and our own goal, including edges of the goal
		float minFetchBehindGoalBallDistance = 0.6f;

		Vision::Distance goalLeftDistance = visionResults->front->vision->getDistance(ownGoal->x - ownGoal->width / 2, ownGoal->y + ownGoal->height / 2);
		Vision::Distance goalRightDistance = visionResults->front->vision->getDistance(ownGoal->x + ownGoal->width / 2, ownGoal->y + ownGoal->height / 2);
		
		Math::Point goalCenterPos = Math::Point(ownGoal->distanceX, ownGoal->distanceY);
		Math::Point goalLeftPos = Math::Point(goalLeftDistance.x, goalLeftDistance.y);
		Math::Point goalRightPos = Math::Point(goalRightDistance.x, goalRightDistance.y);
		Math::Point ballPos = Math::Point(ball->distanceX, ball->distanceY);

		float goalBallDistanceCenter = goalCenterPos.getDistanceTo(ballPos);
		float goalBallDistanceLeft = goalLeftPos.getDistanceTo(ballPos);
		float goalBallDistanceRight = goalRightPos.getDistanceTo(ballPos);
		float goalBallDistance = Math::min(Math::min(goalBallDistanceCenter, goalBallDistanceLeft), goalBallDistanceRight);

		avgBallGoalDistance.add(goalBallDistance);

		ai->dbg("goalBallDistanceCenter", goalBallDistanceCenter);
		ai->dbg("goalBallDistanceLeft", goalBallDistanceLeft);
		ai->dbg("goalBallDistanceRight", goalBallDistanceRight);
		ai->dbg("goalBallDistance", goalBallDistance);
		ai->dbg("avgBallGoalDistance", avgBallGoalDistance.value());
		ai->dbg("avgBallGoalDistanceSize", avgBallGoalDistance.size());
		ai->dbg("avgBallGoalDistanceFull", avgBallGoalDistance.full());

		double minTurnBreak = 2.0;

		// perform turn around if ball is too close to own goal, might reverse into it
		if (
			avgBallGoalDistance.full()
			&& avgBallGoalDistance.value() < minFetchBehindGoalBallDistance
			//&& goalBallDistance < minFetchBehindGoalBallDistance
			&& ownGoal->distance <= 1.5f
			&& (ai->lastTurnAroundTime == -1.0 || Util::duration(ai->lastTurnAroundTime) > minTurnBreak)
		) {
			float turnAngle = ball->angle;
			float underturnAngle = Math::degToRad(15.0f);
			float turnSpeed = Math::TWO_PI;

			if (turnAngle < 0.0f) {
				turnAngle += underturnAngle;
				searchDir = -1.0f;
			} else {
				turnAngle -= underturnAngle;
				searchDir = 1.0f;
			}

			ai->dbg("turnAngle", Math::radToDeg(turnAngle));
			ai->dbg("turnSpeed", turnSpeed);

			turnAroundPerformed = true;

			robot->turnBy(turnAngle, turnSpeed);

			ai->lastTurnAroundTime = Util::millitime();

			return;
		}
	}

	float targetApproachSpeed = 3.0f;
	float accelerateAcceleration = 2.5f;
	float probableBallLostDistance = 0.75f;
	float targetAngle = ai->getTargetAngle(goal->distanceX * (goal->behind ? -1.0f : 1.0f), goal->distanceY * (goal->behind ? -1.0f : 1.0f), ball->distanceX * (ball->behind ? -1.0f : 1.0f), ball->distanceY * (ball->behind ? -1.0f : 1.0f), offsetDistance, targetMode);
	forwardSpeed = Math::getAcceleratedSpeed(forwardSpeed, targetApproachSpeed, dt, accelerateAcceleration);
	float deacceleratedSpeed = Math::map(ballDistance, probableBallLostDistance, probableBallLostDistance * 2.0f, reverseBlindSpeed, forwardSpeed);

	robot->setTargetDir(Math::Rad(targetAngle), deacceleratedSpeed);
	robot->lookAt(goal);

	lastTargetAngle = targetAngle;

	ai->dbg("ballAngle", Math::radToDeg(ball->angle));
	ai->dbg("ballDistanceX", ball->distanceX);
	ai->dbg("offsetDistance", offsetDistance);
	ai->dbg("forwardSpeed", forwardSpeed);
	ai->dbg("deacceleratedSpeed", deacceleratedSpeed);
	ai->dbg("targetAngle", Math::radToDeg(targetAngle));
}

void TestController::FetchBallNearState::onEnter(Robot* robot, Parameters parameters) {
	enterVelocity = robot->getVelocity();
	enterDistance = -1.0f;
	smallestForwardSpeed = -1.0f;
}

void TestController::FetchBallNearState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	robot->stop();
	
	if (robot->dribbler->gotBall()) {
		robot->dribbler->start();

		ai->setState("aim");

		return;
	}
	
	// fetches a nearby ball, focusing on the target goal
	Object* ball = visionResults->getClosestBall(Dir::FRONT);
	Object* goal = visionResults->getLargestGoal(ai->targetSide, Dir::FRONT);

	ai->dbg("ballVisible", ball != NULL);
	ai->dbg("goalVisible", goal != NULL);

	if (goal == NULL) {
		// can't see the goal, switch to direct fetch if ball available, otherwise start searching for ball
		if (ball != NULL) {
			ai->setState("fetch-ball-direct");
		} else {
			ai->setState("find-ball");
		}

		return;
	}

	// switch to searching for ball if not visible any more
	if (ball == NULL) {
		ai->setState("find-ball");

		return;
	}

	// configuration parameters
	float ballDistance = ball->getDribblerDistance();
	float minAllowedApproachSpeed = 0.5f;
	float nearDistance = 0.35f;
	float maxSideSpeedDistance = 0.1f;
	float sideP = 0.5f;

	// store ball first sighting distance
	if (enterDistance == -1.0f) {
		enterDistance = ballDistance;
	} else if (
		(ballDistance > enterDistance + 0.2f || ballDistance > nearDistance)
		&& stateDuration >= 0.5f
	) {
		// ball has gotten further than when started, probably messed it up, switch to faster fetch
		ai->setState("fetch-ball-front");

		return;
	}

	// calculate movement
	float approachP = Math::map(ballDistance, 0.0f, enterDistance, 0.5f, Math::max(enterVelocity, minAllowedApproachSpeed));
	float sidePower = Math::map(Math::abs(ball->distanceX), 0.0f, maxSideSpeedDistance, 0.0f, 1.0f);
	float sideSpeed = sideP * Math::sign(ball->distanceX) * sidePower;
	float forwardSpeed = approachP * (1.0f - sidePower);
	float limitedForwardSpeed = Math::min(forwardSpeed, Math::max(enterVelocity, minAllowedApproachSpeed));
	
	// don't really need the dribbler before capturing ball
	//robot->dribbler->start();

	robot->setTargetDir(limitedForwardSpeed, sideSpeed);
	robot->lookAt(goal);
	robot->dribbler->prime();

	ai->dbg("approachP", approachP);
	ai->dbg("sideP", sideP);
	ai->dbg("ballDistance", ballDistance);
	ai->dbg("forwardSpeed", forwardSpeed);
	ai->dbg("limitedForwardSpeed", limitedForwardSpeed);
	ai->dbg("sidePower", sidePower);
	ai->dbg("sideSpeed", sideSpeed);
	ai->dbg("enterVelocity", enterVelocity);
	ai->dbg("enterDistance", enterDistance);
	ai->dbg("ballAngle", (Math::radToDeg(ball->angle)));
	ai->dbg("goalAngle", (Math::radToDeg(goal->angle)));
	ai->dbg("ball->distanceX", ball->distanceX);
}

void TestController::AimState::onEnter(Robot* robot, Parameters parameters) {
	avoidBallSide = TargetMode::UNDECIDED;
	searchGoalDir = 0.0f;
	spinDuration = 0.0f;
	reverseDuration = 0.0f;
	avoidBallDuration = 0.0f;
	validKickFrames = 0;
	nearLine = false;
	escapeCornerPerformed = false;

	nearLine = parameters.find("near-line") != parameters.end();
	escapeCornerPerformed = parameters.find("escape-corner-performed") != parameters.end();
}

void TestController::AimState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	robot->stop();
	robot->dribbler->start();

	ai->dbg("gotBall", robot->dribbler->gotBall());
	ai->dbg("escapeCornerPerformed", escapeCornerPerformed);
	
	// switch to searchinf for ball if lost it
	if (!robot->dribbler->gotBall()) {
		ai->setState("find-ball");

		return;
	}

	// attempt to get back on the field if out
	if (ai->isRobotOutFront || ai->isRobotOutRear) {
		// no point to aim if robot is out
		std::cout << "! Robot is out, kicking" << std::endl;

		robot->kick();

		ai->setState("return-field");

		return;
	}

	if (robot->hasTasks()) {
		return;
	}
	
	Object* goal = visionResults->getLargestGoal(ai->targetSide, Dir::FRONT);
	Object* rearGoal = visionResults->getLargestGoal(Side::UNKNOWN, Dir::REAR);

	ai->dbg("goalVisible", goal != NULL);
	ai->dbg("nearLine", nearLine);
	ai->dbg("wasInCornerLately", ai->wasInCornerLately());
	ai->dbg("lastTargetGoalAngle", ai->lastTargetGoalAngle);
	ai->dbg("timeSinceEscapeCorner", lastEscapeCornerTime == -1.0 ? -1.0 : Util::duration(lastEscapeCornerTime));

	// configuration parameters
	float searchPeriod = Config::robotSpinAroundDribblerPeriod;
	float reversePeriod = 1.0f;
	float reverseSpeed = 1.5f;
	float maxAimDuration = 6.0f;
	int weakKickStrength = 2250; // kicks it a bit, but might stay on the field in new place

	if (goal == NULL) {
		// no goal visible, start searching for it
		if (searchGoalDir == 0.0f) {
			if (ai->lastTargetGoalAngle > 0.0f) {
				searchGoalDir = 1.0f;
			} else {
				searchGoalDir = -1.0f;
			}
		}

		// if aiming has taken too long, perform a weak kick and give up
		if (combinedDuration > maxAimDuration) {
			Side ownSide = ai->targetSide == Side::YELLOW ? Side::BLUE : Side::YELLOW;
			Object* ownGoalFront = visionResults->getLargestGoal(ownSide, Dir::FRONT);

			// only perform the give-up weak kick if not looking towards own goal
			if (ownGoalFront == NULL) {
				robot->kick(weakKickStrength);

				ai->setState("fetch-ball-behind");

				return;
			}
		}

		// escape from corner if near one lately, only do it near own goal and also when near line and target goal, only do it once and when there is no goal neaby at the rear
		if (
			(ai->wasInCornerLately() || (ai->wasNearLineLately() && ai->isRobotNearTargetGoal(1.2f)))
			&& !escapeCornerPerformed
			&& ai->isRobotNearTargetGoal()
			&& (rearGoal == NULL || ai->getObjectClosestDistance(visionResults, rearGoal) > 1.5f)
		) {
			ai->setState("escape-corner");

			lastEscapeCornerTime = Util::millitime();

			return;
		}

		spinDuration += dt;

		if (ai->isRobotNearGoal()) {
			robot->setTargetOmega(searchGoalDir * Math::PI);
		} else {
			/*if (nearLine) {
				// spin around dribbler with acceleration and initial reverse
				float spinAccelerationTime = 1.5f;
				float spinReverseDuration = 0.75f;
				float spinSlowdownMultiplier = 4.0f;
				float spinReverseSpeed = 0.4f;
				float spinRadius = Config::robotSpinAroundDribblerRadius;
				float spinPeriod = Math::map(spinDuration, 0.0f, spinAccelerationTime, Config::robotSpinAroundDribblerPeriod * spinSlowdownMultiplier, Config::robotSpinAroundDribblerPeriod);
		
				float spinSpeed = (2.0f * Math::PI * spinRadius) / spinPeriod;
				float spinOmega = (2.0f * Math::PI) / spinPeriod;
				float spinForwardSpeed = 0.0f;

				if (spinDuration < spinReverseDuration) {
					spinForwardSpeed = Math::map(spinDuration, 0.0f, spinReverseDuration, 0.0f, -spinReverseSpeed);
				} else {
					spinForwardSpeed = Math::map(spinDuration, 0.0f, spinAccelerationTime, 0.0f, Config::robotSpinAroundDribblerForwardSpeed);
				}

				if (searchGoalDir == -1.0f) {
					spinSpeed *= -1.0f;
					spinOmega *= -1.0f;
				}

				robot->setTargetDir(spinForwardSpeed, -spinSpeed, spinOmega);
			} else {
				robot->spinAroundDribbler(searchGoalDir == -1.0f, searchPeriod);
			}*/

			// spin around robot axis near goal to avoid moving goal spinning off-axis
			if (!ai->wasNearLineLately() && stateDuration < 0.25f) {
				// move forwards a bit, getting better grip of the ball
				robot->setTargetDir(0.25f, 0.0f);
				robot->dribbler->prime();
			} else {
				// TODO Accelerate to speed
				robot->spinAroundDribbler(searchGoalDir == -1.0f, searchPeriod);
			}
		}

		// start searching for own goal after almost full rotation
		if (spinDuration > searchPeriod / 1.25f) {
			float reverseTime = 1.5f;
			float approachOwnGoalMinDistance = 1.5f;
			float accelerationPeriod = 1.5f;
			float reverseSpeed = 1.0f;

			// jump back to the beginning of aim state after reversing for some time, performing the sping again
			if (reverseDuration > reverseTime) {
				ai->setState("aim");

				return;
			}

			// didn't find our goal in time, search for opponent goal and drive towards it instead
			Side ownSide = ai->targetSide == Side::YELLOW ? Side::BLUE : Side::YELLOW;
			Object* ownGoal = visionResults->getLargestGoal(ownSide, Dir::REAR);

			// make sure we don't get too close to our own goal
			if (ownGoal != NULL && ownGoal->distance > approachOwnGoalMinDistance) {
				float accelerationMultiplier = Math::map(reverseDuration, 0, accelerationPeriod, 0.0f, 1.0f);
				float acceleratedReverseSpeed = -reverseSpeed * accelerationMultiplier;

				// reverse towards own goal, looking at it
				robot->setTargetDir(
					acceleratedReverseSpeed,
					0.0f,
					0.0f
				);

				robot->lookAtBehind(ownGoal);

				reverseDuration += dt;

				ai->dbg("accelerationMultiplier", accelerationMultiplier);
				ai->dbg("acceleratedReverseSpeed", acceleratedReverseSpeed);
			}
		}

		return;
	}

	ai->dbg("goalVisible", true);

	// configuration
	float avoidBallSpeed = 0.65f;
	float minForwardSpeed = 0.1f;
	float minBallAvoidSideSpeed = 0.25f;
	float maxRobotKickOmega = Math::PI / 4.0f;
	float maxBallAvoidTime = 1.5f;
	double minKickInterval = 1.0;

	// need to decide whether kicking is a good idea
	int halfWidth = Config::cameraWidth / 2;
	int leftEdge = goal->x - goal->width / 2;
	int rightEdge = goal->x + goal->width / 2;
	int goalKickThresholdPixels = (int)((float)goal->width * Config::goalKickThreshold);
	double timeSinceLastKick = lastKickTime != 0.0 ? Util::duration(lastKickTime) : -1.0;
	bool isBallInWay = visionResults->isBallInWay(visionResults->front->balls, goal->y + goal->height / 2);
	Obstruction goalPathObstruction = visionResults->front->goalPathObstruction;
	bool isGoalPathObstructed = goalPathObstruction != Obstruction::NONE;
	float forwardSpeed = 0.0f;
	float sideSpeed = 0.0f;
	bool validWindow = false;
	bool isKickTooSoon = lastKickTime != -1.0 && timeSinceLastKick < minKickInterval;

	// limit ball avoidance time
	if (avoidBallDuration > maxBallAvoidTime) {
		isBallInWay = false;
		isGoalPathObstructed = false;
	}

	// drive sideways if there's a ball in the way
	if (isBallInWay || isGoalPathObstructed) {
		// check whether there's another ball close by
		float anotherBallCloseDistance = 0.3f;
		Object* nextClosestBall = visionResults->getNextClosestBall(Dir::FRONT);
		bool nearbyAnotherBall = nextClosestBall != NULL && nextClosestBall->getDribblerDistance() < anotherBallCloseDistance;

		// decide which way to avoid the balls once
		if (avoidBallSide == TargetMode::UNDECIDED) {
			if (isGoalPathObstructed) {
				if (goalPathObstruction == Obstruction::LEFT) {
					avoidBallSide = TargetMode::RIGHT;
				} else {
					avoidBallSide = TargetMode::LEFT;
				}
			} else {
				// make sure to drive near the centerline of the field not out further
				if (robot->getPosition().y < Config::fieldHeight / 2.0f) {
					avoidBallSide = ai->targetSide == Side::BLUE ? TargetMode::RIGHT : TargetMode::LEFT;
				} else {
					avoidBallSide = ai->targetSide == Side::BLUE ? TargetMode::LEFT : TargetMode::RIGHT;
				}
			}
		}

		// don't move forwards near another ball not to drive into it
		if (nearbyAnotherBall) {
			forwardSpeed = 0.0f;
		}

		avoidBallDuration += dt;

		sideSpeed = (avoidBallSide == TargetMode::LEFT ? -1.0f : 1.0f) * Math::map(avoidBallDuration, 0.0f, 1.0f, 0.0f, avoidBallSpeed);
	
		// not sure if this is good after all
		forwardSpeed = Math::map(goal->distance, 0.5f, 1.0f, 0.0f, Math::abs(sideSpeed));

		ai->dbg("nearbyAnotherBall", nearbyAnotherBall);
	}

	// check whether the aiming is precise enough
	if (!goal->behind) {
		if (
			leftEdge + goalKickThresholdPixels < halfWidth
			&& rightEdge - goalKickThresholdPixels > halfWidth
		) {
			validWindow = true;
		}
	}

	// always apply some forward speed (can think that has ball when really doesn't)
	forwardSpeed = Math::max(forwardSpeed, minForwardSpeed);

	bool isRobotOmegaLowEnough = Math::abs(robot->getOmega()) <= maxRobotKickOmega;
	bool isFrameValid = validWindow && !isKickTooSoon && !isBallInWay && !isGoalPathObstructed && isRobotOmegaLowEnough && robot->dribbler->gotBall(true);

	if (isFrameValid) {
		validKickFrames++;
	} else {
		validKickFrames = 0;
	}

	bool performKick = validKickFrames >= 4;

	// only perform the kick if valid view has been observed for a couple of frames
	if (performKick) {
		robot->kick();

		ai->resetLastBall();

		lastKickTime = Util::millitime();
	} else {
		robot->setTargetDir(forwardSpeed, sideSpeed);
		robot->lookAt(goal);
	}

	ai->dbg("performKick", performKick);
	ai->dbg("validWindow", validWindow);
	ai->dbg("isFrameValid", isFrameValid);
	ai->dbg("isKickTooSoon", isKickTooSoon);
	ai->dbg("isBallInWay", isBallInWay);
	ai->dbg("isGoalPathObstructed", isGoalPathObstructed);
	ai->dbg("isRobotOmegaLowEnough", isRobotOmegaLowEnough);
	ai->dbg("avoidBallSide", avoidBallSide);
	ai->dbg("validKickFrames", validKickFrames);
	ai->dbg("leftEdge", leftEdge);
	ai->dbg("rightEdge", rightEdge);
	ai->dbg("halfWidth", halfWidth);
	ai->dbg("leftValid", leftEdge + goalKickThresholdPixels < halfWidth);
	ai->dbg("rightValid", rightEdge - goalKickThresholdPixels > halfWidth);
	ai->dbg("goalKickThresholdPixels", goalKickThresholdPixels);
	ai->dbg("sinceLastKick", timeSinceLastKick);
	ai->dbg("forwardSpeed", forwardSpeed);
	ai->dbg("sideSpeed", sideSpeed);
	ai->dbg("whiteDistance", visionResults->front->whiteDistance.min);
	ai->dbg("robotOmega", robot->getOmega());
}

void TestController::ReturnFieldState::onEnter(Robot* robot, Parameters parameters) {
	queuedApproachGoal = false;
}

void TestController::ReturnFieldState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	robot->stop();

	if (robot->hasTasks()) {
		return;
	}

	FindBallState* findBallState = (FindBallState*)ai->states["find-ball"];

	if (ai->isRobotOutRear) {
		// turn half a turn in the same dir as normal find ball would
		robot->turnBy(Math::degToRad(180.0f) * findBallState->searchDir, Math::TWO_PI);

		return;
	} else if (ai->isRobotOutFront || queuedApproachGoal) {
		// robot is out from front camera, search for a goal and focus on it before starting moving
		float searchSpeed = Math::PI;

		if (stateDuration > Math::TWO_PI / searchSpeed) {
			searchSpeed /= 2.0f;
		}

		queuedApproachGoal = true;

		// search for any furthest goal
		Object* goal = visionResults->getLargestGoal(Side::UNKNOWN, Dir::FRONT);

		if (goal != NULL && goal->distance > Config::fieldWidth / 3.0f) {
			robot->lookAt(goal);

			if (Math::abs(goal->angle) < Math::degToRad(5.0f)) {
				// make a blind dash towards the goal
				robot->setTargetDirFor(1.0f, 0.0f, 0.0f, 1.0f);

				queuedApproachGoal = false;
			}
		} else {
			// no such goal is visible, spin and search for it
			robot->setTargetOmega(findBallState->searchDir * searchSpeed);

			// hasn't found goal, drive towards line instead
			if (stateDuration > 5.0f) {
				// keep turning until line is horizontal enough
				if (
					visionResults->front->whiteDistance.left != -1.0f
					&& visionResults->front->whiteDistance.right != -1.0f
					&& visionResults->front->blackDistance.left != -1.0f
					&& visionResults->front->blackDistance.right != -1.0f
					&& Math::abs(visionResults->front->whiteDistance.left - visionResults->front->whiteDistance.right) < 0.05f
				) {
					robot->setTargetDirFor(1.0f, 0.0f, 0.0f, 1.0f);

					queuedApproachGoal = false;
				}
			}
		}

		return;
	} else {
		ai->setState("find-ball");
	}

}

void TestController::EscapeCornerState::onEnter(Robot* robot, Parameters parameters) {
	startTravelledDistance = robot->getTravelledDistance();
}

void TestController::EscapeCornerState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	robot->stop();
	robot->dribbler->start();

	if (!robot->dribbler->gotBall()) {
		ai->setState("find-ball");

		return;
	}

	if (ai->isRobotOutFront || ai->isRobotOutRear) {
		// no point to mess with a ball that's already out
		robot->kick();

		ai->setState("return-field");

		return;
	}

	// configuration
	float reverseP = 1.0f;
	float sideP = 0.5f;
	float accelerationDuration = 1.5f;
	float outOfCornerLineDistance = 0.5f;
	float maxSideSpeedDiff = 0.2f;
	float reverseDistance = 0.5f;

	float stateTravelledDistance = robot->getTravelledDistance() - startTravelledDistance;

	ai->dbg("stateTravelledDistance", stateTravelledDistance);

	// check if enough distance has been travelled
	if (stateTravelledDistance > reverseDistance) {
		Parameters parameters;

		parameters["escape-corner-performed"] = "1";

		// notify aim state that escaping from corner has already been performed
		ai->setState("aim", parameters);

		return;
	}

	float sideSpeed = 0.0f;
	float reverseSpeed = reverseP * Math::map(stateDuration, 0.0f, accelerationDuration, 0.0f, 1.0f);

	// apply some side speed to steer towards the center of the corner
	if (ai->whiteDistance.left != -1.0f && ai->whiteDistance.right != -1.0f) {
		float diff = Math::abs(ai->whiteDistance.left - ai->whiteDistance.right);
		float side = ai->whiteDistance.left < ai->whiteDistance.right ? 1.0f : -1.0f;

		ai->dbg("diff", diff);
		ai->dbg("side", side);
		ai->dbg("whiteDistance.min", ai->whiteDistance.min);

		sideSpeed = sideP * side * Math::map(diff, 0.0f, maxSideSpeedDiff, 0.0, 1.0f);
	}

	robot->setTargetDir(-reverseSpeed, sideSpeed);

	ai->dbg("reverseSpeed", reverseSpeed);
	ai->dbg("sideSpeed", sideSpeed);
}

void TestController::DriveCircleState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	Object* goal = visionResults->getLargestGoal(ai->targetSide, Dir::FRONT);

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

void TestController::AccelerateState::onEnter(Robot* robot, Parameters parameters) {
	forwardSpeed = robot->getVelocity();
}

void TestController::AccelerateState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	robot->stop();
	
	if (robot->dribbler->gotBall()) {
		robot->dribbler->start();

		return;
	}

	Object* ball = visionResults->getClosestBall(Dir::FRONT);

	if (ball != NULL) {
		ai->setLastBall(ball);
	} else {
		ball = ai->getLastBall(Dir::FRONT);
	}

	if (ball == NULL) {
		return;
	}

	//float targetApproachSpeed = 2.0f;
	//float acceleration = 2.0f;
	float targetApproachSpeed = 3.5f;
	float minApproachSpeed = 0.3f;
	float accelerateAcceleration = 2.8f;
	float brakeAcceleration = 2.5f;
	float realSpeed = robot->getVelocity();
	float ballDistance = ball->getDribblerDistance();
	float brakeDistance = Math::getAccelerationDistance(forwardSpeed, 0.0f, brakeAcceleration);
	bool braking = ballDistance < brakeDistance;

	if (braking) {
		//float brakeAcceleration = Math::getAcceleration(forwardSpeed, 0.0f, brakeDistance);
		float brakeAcceleration = Math::getAcceleration(realSpeed, 0.0f, brakeDistance);

		targetApproachSpeed = forwardSpeed + brakeAcceleration * dt;

		ai->dbg("brakeAcceleration", brakeAcceleration);

		robot->dribbler->start();
	}

	forwardSpeed = Math::max(Math::getAcceleratedSpeed(forwardSpeed, targetApproachSpeed, dt, braking ? brakeAcceleration : accelerateAcceleration), minApproachSpeed);

	robot->setTargetDir(forwardSpeed, 0.0f);
	robot->lookAt(ball);

	ai->dbg("realSpeed", realSpeed);
	ai->dbg("ballDistance", ballDistance);
	ai->dbg("brakeDistance", brakeDistance);
	ai->dbg("targetApproachSpeed", targetApproachSpeed);
	ai->dbg("forwardSpeed", forwardSpeed);
	ai->dbg("dt", dt);
}

void TestController::DriveHomeState::onEnter(Robot* robot, Parameters parameters) {
	drivePerformed = false;
}

void TestController::DriveHomeState::step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration, float combinedDuration) {
	robot->stop();

	float homeX, homeY, homeOrientation;

	if (ai->targetSide == Side::YELLOW) {
		homeX = Config::fieldWidth - Config::robotRadius;
		homeY = Config::robotRadius;
		homeOrientation = Math::PI - Math::PI / 8.0f;
	} else {
		homeX = Config::robotRadius;
		homeY = Config::fieldHeight - Config::robotRadius;
		homeOrientation = -Math::PI / 8;
	}

	Math::Position homePos(homeX, homeY, homeOrientation);
	Math::Position robotPos = robot->getPosition();

	if (robot->hasTasks()) {
		ai->dbg("distanceDiff", homePos.distanceTo(robotPos));
		ai->dbg("minWhiteDistance", visionResults->rear->whiteDistance.min);

		// continue based on lines if close enough
		if (homePos.distanceTo(robotPos) < 1.0f && (visionResults->rear->whiteDistance.min != -1.0f && visionResults->rear->whiteDistance.min < 1.0f)) {
			robot->clearTasks();
		}

		return;
	}

	if (!drivePerformed) {
		robot->driveTo(homeX, homeY, homeOrientation);

		drivePerformed = true;

		return;
	}

	/*if (visionResults->rear->blackDistance.max < 0.25f) {
		ai->setState("manual-control");

		return;
	}*/

	Object* opponentGoal = visionResults->getLargestGoal(ai->targetSide, Dir::FRONT);

	float sideP = 0.5f;
	float reverseP = 0.5f;
	float maxSideSpeedDiff = 0.2f;

	float diff = Math::abs(visionResults->rear->whiteDistance.left - visionResults->rear->whiteDistance.right);
	float side = visionResults->rear->whiteDistance.left < visionResults->rear->whiteDistance.right ? -1.0f : 1.0f;

	float sideSpeed = sideP * side * Math::map(diff, 0.0f, maxSideSpeedDiff, 0.0, 1.0f);
	float reverseSpeed = Math::map(visionResults->rear->blackDistance.max, 0.25f, 1.0f, 0.0f, reverseP);

	robot->setTargetDir(-reverseSpeed, sideSpeed);

	if (opponentGoal != NULL) {
		robot->lookAt(opponentGoal);
	}

	ai->dbg("diff", diff);
	ai->dbg("side", side);
	ai->dbg("sideSpeed", sideSpeed);
	ai->dbg("reverseSpeed", reverseSpeed);
	ai->dbg("maxWhiteDistance", visionResults->rear->whiteDistance.max);
}