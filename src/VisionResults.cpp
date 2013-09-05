#include "VisionResults.h"
#include "Object.h"

VisionResults::VisionResults() {

}


VisionResults::~VisionResults() {

}

void VisionResults::merge(VisionResults* other) {

}

Object* VisionResults::getClosestBall(bool frontOnly) {
	return NULL;

	/*const ObjectList& frontBalls = getFrontBalls();
	float closestDistance = 100.0f;
	float distance;
	Object* ball;
	Object* closestBall = NULL;

	for (ObjectListItc it = frontBalls.begin(); it != frontBalls.end(); it++) {
		ball = *it;
		distance = ball->behind ? ball->distance * 1.25f : ball->distance;
		
		if (closestBall == NULL || distance < closestDistance) {
			closestBall = ball;
			closestDistance = distance;
		}
	}

	if (!frontOnly) {
		const ObjectList& rearBalls = getRearBalls();

		for (ObjectListItc it = rearBalls.begin(); it != rearBalls.end(); it++) {
			ball = *it;
			distance = ball->behind ? ball->distance * 1.25f : ball->distance;
		
			if (closestBall == NULL || distance < closestDistance) {
				closestBall = ball;
				closestDistance = distance;
			}
		}
	}

	return closestBall;*/
}

Object* VisionResults::getLargestGoal(Side side, bool frontOnly) {
	return NULL;

	/*const ObjectList& frontGoals = getFrontGoals();
	float area;
	float largestArea = 0.0f;
	Object* goal;
	Object* largestGoal = NULL;

	for (ObjectListItc it = frontGoals.begin(); it != frontGoals.end(); it++) {
		goal = *it;
		
		if (side != Side::UNKNOWN && goal->type != (int)side) {
			continue;
		}

		//area = goal->area;
		area = goal->width * goal->height;

		if (largestGoal == NULL || area > largestArea) {
			largestGoal = goal;
			largestArea = area;
		}
	}

	if (frontOnly != true) {
		const ObjectList& rearGoals = getRearGoals();

		for (ObjectListItc it = rearGoals.begin(); it != rearGoals.end(); it++) {
			goal = *it;

			if (side != Side::UNKNOWN && goal->type != (int)side) {
				continue;
			}

			//area = goal->area;
			area = goal->width * goal->height;
		
			if (largestGoal == NULL || area > largestArea) {
				largestGoal = goal;
				largestArea = area;
			}
		}
	}

	if (largestGoal != NULL) {
		/int minX, minY, maxX, maxY;
		const ObjectList& goals = largestGoal->behind ? getRearGoals() : getFrontGoals();

		for (ObjectListItc it = goals.begin(); it != goals.end(); it++) {
			goal = *it;
		
			if (goal == largestGoal || !goal->intersects(largestGoal)) {
				continue;
			}

			minX = Math::min(largestGoal->x - largestGoal->width / 2, goal->x - goal->width / 2);
			maxX = Math::min(largestGoal->x + largestGoal->width / 2, goal->x + goal->width / 2);
			minY = Math::min(largestGoal->y - largestGoal->height / 2, goal->y - goal->height / 2);
			maxY = Math::min(largestGoal->y + largestGoal->height / 2, goal->y + goal->height / 2);

			largestGoal->width = maxX - minX;
			largestGoal->height = maxY - minY;
			largestGoal->x = minX + largestGoal->width / 2;
			largestGoal->y = minY + largestGoal->height / 2;
		}/

		lastLargestGoal.copyFrom(largestGoal);

		return largestGoal;
	} else if (
		lastLargestGoal.width > 0
		&& lastLargestGoal.type == side
		&& Util::duration(lastLargestGoal.lastSeenTime) < Config::fakeObjectLifetime
		&& !frontOnly
	) {
		return &lastLargestGoal;
	} else {
		return NULL;
	}*/
}

/*Object* Vision::getFurthestGoal(bool frontOnly) {
	const ObjectList& frontGoals = getFrontGoals();
	float furthestDistance = 0.0f;
	Object* goal;
	Object* furthestGoal = NULL;

	for (ObjectListItc it = frontGoals.begin(); it != frontGoals.end(); it++) {
		goal = *it;

		if (furthestGoal == NULL || goal->distance > furthestDistance) {
			furthestGoal = goal;
			furthestDistance = goal->distance;
		}
	}

	if (!frontOnly) {
		const ObjectList& rearGoals = getRearGoals();

		for (ObjectListItc it = rearGoals.begin(); it != rearGoals.end(); it++) {
			goal = *it;

			if (furthestGoal == NULL || goal->distance > furthestDistance) {
				furthestGoal = goal;
				furthestDistance = goal->distance;
			}
		}
	}

	if (furthestGoal != NULL) {
		lastFurthestGoal.copyFrom(furthestGoal);

		return furthestGoal;
	} else if (
		lastFurthestGoal.width > 0
		&& Util::duration(lastFurthestGoal.lastSeenTime) < Config::fakeObjectLifetime
	) {
		return &lastFurthestGoal;
	} else {
		return NULL;
	}
}*/

Object* VisionResults::getFurthestGoal(bool frontOnly) {
	return NULL;

	/*Object* largestYellow = getLargestGoal(Side::YELLOW, frontOnly);
	Object* largestBlue = getLargestGoal(Side::BLUE, frontOnly);

	if (largestYellow != NULL) {
		if (largestBlue != NULL) {
			if (largestYellow->distance > largestBlue->distance) {
				return largestYellow;
			} else {
				return largestBlue;
			}
		} else {
			return largestYellow;
		}
	} else {
		if (largestBlue != NULL) {
			return largestBlue;
		} else {
			return NULL;
		}
	}*/
}
