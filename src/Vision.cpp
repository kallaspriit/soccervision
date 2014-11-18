#include "Vision.h"
#include "Config.h"
#include "Util.h"

#include <iostream>
#include <algorithm>

Vision::Vision(Blobber* blobber, CameraTranslator* cameraTranslator, Dir dir, int width, int height) : blobber(blobber), cameraTranslator(cameraTranslator), dir(dir), width(width), height(height), obstructionSide(Obstruction::NONE) {
    validBallBgColors.push_back("green");
    validBallBgColors.push_back("white");
    validBallBgColors.push_back("black");
    validBallBgColors.push_back("ball");
    validBallBgColors.push_back("yellow-goal");
    validBallBgColors.push_back("blue-goal");

    validBallPathColors.push_back("green");
    validBallPathColors.push_back("white");
    validBallPathColors.push_back("black");
    validBallPathColors.push_back("ball");
    validBallPathColors.push_back("yellow-goal");
    validBallPathColors.push_back("blue-goal");

	viewObstructedValidColors.push_back("green");
    viewObstructedValidColors.push_back("white");
    viewObstructedValidColors.push_back("black");
    viewObstructedValidColors.push_back("ball");
    viewObstructedValidColors.push_back("yellow-goal");
    viewObstructedValidColors.push_back("blue-goal");

    goalObstructedValidColors.push_back("green");
    goalObstructedValidColors.push_back("white");
    goalObstructedValidColors.push_back("ball");

    validGoalPathColors.push_back("green");
    validGoalPathColors.push_back("white");
    validGoalPathColors.push_back("black");
    validGoalPathColors.push_back("ball");
    //validGoalPathColors.push_back("yellow-goal");
    //validGoalPathColors.push_back("blue-goal");

	validColorsBelowBall.push_back("none");
	validColorsBelowBall.push_back("black");

	goalColors.push_back("yellow-goal");
    goalColors.push_back("blue-goal");

	obstructionSide = Obstruction::NONE;
}

Vision::~Vision() {
   
}

void Vision::setDebugImage(unsigned char* image, int width, int height) {
	canvas.data = image;
	canvas.width = width;
	canvas.height = height;
}

Vision::Result* Vision::process() {
	Result* result = new Result();

	result->vision = this;

	result->goals = processGoals(dir);
    result->balls = processBalls(dir);

	/*if (dir == Dir::FRONT) {
		updateObstructions();
	}*/

	updateColorDistances();
	updateColorOrder();

	result->obstructionSide = obstructionSide;
	result->colorOrder = colorOrder;
	result->whiteDistance = whiteDistance;
	result->blackDistance = blackDistance;
	result->goalPathObstruction = getGoalPathObstruction();

	return result;
}

ObjectList Vision::processBalls(Dir dir) {
	ObjectList allBalls;
	ObjectList filteredBalls;

    Distance distance;

    Blobber::Blob* blob = blobber->getBlobs("ball");

    while (blob != NULL) {
		if (blob->area < Config::ballBlobMinArea) {
			blob = blob->next;

			continue;
		}

		distance = getDistance((int)blob->centerX, (int)blob->y2);

		if (dir == Dir::REAR) {
			if (distance.angle > 0.0f) {
				distance.angle -= Math::PI;
			} else {
				distance.angle += Math::PI;
			}
		}

		if (blob->x1 < 0) blob->x1 = 0;
		if (blob->x2 > width - 1) blob->x2 = width - 1;
		if (blob->y1 < 0) blob->y1 = 0;
		if (blob->y2 > height - 1) blob->y2 = height - 1;

		int width = blob->x2 - blob->x1;
		int height = blob->y2 - blob->y1;

        Object* ball = new Object(
            blob->x1 + width / 2,
            blob->y1 + height / 2,
            width,
            height,
            blob->area,
			distance.straight,
			distance.x,
			distance.y,
            distance.angle,
			0,
			dir == Dir::FRONT ? false : true
        );
		
        allBalls.push_back(ball);

        blob = blob->next;
    }

	// TODO Make the overlap margin dependent on distance (larger for objects close-by)
	ObjectList mergedBalls = Object::mergeOverlapping(allBalls, Config::ballOverlapMargin);

	for (ObjectListItc it = mergedBalls.begin(); it != mergedBalls.end(); it++) {
		Object* ball = *it;

		if (isValidBall(ball, dir)) {
			int extendHeightBelow = getPixelsBelow(ball->x, ball->y + ball->height / 2, validColorsBelowBall);

			if (extendHeightBelow > 0) {
				ball->y += extendHeightBelow / 2;
				ball->height = (int)Math::min((float)(ball->height + extendHeightBelow), (float)ball->width);
			}

			distance = getDistance(ball->x, ball->y + ball->height / 2);

			if (dir == Dir::REAR) {
				if (distance.angle > 0.0f) {
					distance.angle -= Math::PI;
				} else {
					distance.angle += Math::PI;
				}
			}

			ball->distance = distance.straight;
			ball->distanceX = distance.x;
			ball->distanceY = distance.y;
			ball->angle = distance.angle;

			// TODO Review this
			if (ball->distance < 0.0f) {
				std::cout << "- Skipping ball with invalid distance: " << ball->distance << std::endl;

				continue;
			}

			filteredBalls.push_back(ball);
		}
	}

	return filteredBalls;
}

ObjectList Vision::processGoals(Dir dir) {
	ObjectList allGoals;
	ObjectList filteredGoals;

    Distance distance;
    
    for (int i = 0; i < 2; i++) {
        Blobber::Blob* blob = blobber->getBlobs(i == 0 ? "yellow-goal" : "blue-goal");

        while (blob != NULL) {
			if (blob->area < Config::goalBlobMinArea) {
				blob = blob->next;

				continue;
			}

			distance = getDistance((int)blob->centerX, (int)blob->y2);

			if (dir == Dir::REAR) {
				if (distance.angle > 0.0f) {
					distance.angle -= Math::PI;
				} else {
					distance.angle += Math::PI;
				}
			}

			if (blob->x1 < 0) blob->x1 = 0;
			if (blob->x2 > width - 1) blob->x2 = width - 1;
			if (blob->y1 < 0) blob->y1 = 0;
			if (blob->y2 > height - 1) blob->y2 = height - 1;

			int width = blob->x2 - blob->x1;
			int height = blob->y2 - blob->y1;

			Object* goal = new Object(
				blob->x1 + width / 2,
				blob->y1 + height / 2,
				width,
				height,
				blob->area,
				distance.straight,
				distance.x,
				distance.y,
				distance.angle,
				i == 0 ? Side::YELLOW : Side::BLUE,
				dir == Dir::FRONT ? false : true
			);

			goal->processed = false;
			allGoals.push_back(goal);

            blob = blob->next;
        }
    }

	ObjectList mergedGoals = Object::mergeOverlapping(allGoals, Config::goalOverlapMargin, true);

	for (ObjectListItc it = mergedGoals.begin(); it != mergedGoals.end(); it++) {
		Object* goal = *it;

		if (isValidGoal(goal, goal->type == 0 ? Side::YELLOW : Side::BLUE)) {
			// TODO Extend the goal downwards using extended color / limited ammount horizontal too

			distance = getDistance(goal->x, goal->y + goal->height / 2);

			if (dir == Dir::REAR) {
				if (distance.angle > 0.0f) {
					distance.angle -= Math::PI;
				} else {
					distance.angle += Math::PI;
				}
			}

			goal->distance = distance.straight;
			goal->distanceX = distance.x;
			goal->distanceY = distance.y;
			goal->angle = distance.angle;

			if (goal->distance < 0.0f) {
				std::cout << "- Skipping goal with invalid distance: " << goal->distance << std::endl;

				continue;
			}

			filteredGoals.push_back(goal);
		}
	}

	return filteredGoals;
}

bool Vision::isValidGoal(Object* goal, Side side) {
	/*int x1, y1, x2, y2;

	float undersideMetric = getUndersideMetric(
		goal->x - goal->width / 2,
		goal->y - goal->height / 2,
		goal->distance,
		goal->width,
		goal->height,
		side == Side::YELLOW ? "yellow-goal" : "blue-goal",
		side == Side::YELLOW ? "yellow-goal-wide" : "blue-goal-wide",
		validGoalPathColors,
		x1, y1, x2, y2
	);

	if (x1 != -1 && y1 != -1 && x2 != -1 && y2 != -1) {
		goal->width = x2 - x1;
		goal->height = y2 - y1;
		goal->x = x1 + goal->width / 2;
		goal->y = y1 + goal->height / 2;
	}*/

	std::string color1 = goal->type == 0 ? "blue-goal" : "yellow-goal";
	std::string color2 = goal->type == 0 ? "blue-goal-wide" : "yellow-goal-wide";
	int halfWidth = goal->width / 2;
	int halfHeight = goal->height / 2;

	EdgeDistanceMetric edgeDistanceMetric = getEdgeDistanceMetric(goal->x - halfWidth, goal->y - halfHeight, goal->width, goal->height, color1, color2);

	std::cout << "@ EDGE LEFT: " << edgeDistanceMetric.leftTopDistance.distance << "m, right: " << edgeDistanceMetric.rightTopDistance.distance << "m" << std::endl;

	if (goal->y + goal->height < Config::goalPathSenseStartY) {
		PathMetric pathMetric = getPathMetric(
			Config::cameraWidth / 2,
			Config::goalPathSenseStartY,
			goal->x,
			goal->y + goal->height / 2,
			validGoalPathColors
			//,"green"
		);

		if (
			pathMetric.percentage < Config::minValidGoalPathThreshold
			//|| pathMetric.out
			//|| !pathMetric.validColorFound
			//|| pathMetric.invalidSpree > getBallMaxInvalidSpree(ball->y + ball->height / 2)
		) {
			//std::cout << "@ GOAL PATH FAILS: " << pathMetric.percentage << " VS " << Config::minValidGoalPathThreshold << ", OUT: " << (pathMetric.out ? "YES" : "NO") << std::endl;

			return false;
		}
	}

	if (goal->area < Config::goalMinArea) {
		//std::cout << "@ GOAL INVALID MIN AREA: " << goal->area << " VS " << Config::goalMinArea << std::endl;

		return false;
	} else if (goal->area > Config::goalCertainArea) {
		return true;
	}

	if (goal->y - goal->height / 2 > Config::goalTopMaxY) {
		//std::cout << "@ GOAL NOT TOP ENOUGH: " << (goal->y - goal->height / 2) << " VS " << Config::goalTopMaxY << std::endl;

		// TODO restore in some form
		//return false;
	}

	/*if (undersideMetric < Config::goalMinUndersideMetric) {
		//std::cout << "@ GOAL INVALID UNDERSIDE: " << undersideMetric << " VS " << Config::goalMinUndersideMetric << std::endl;

		return false;
	}*/

    return true;
}

bool Vision::isValidBall(Object* ball, Dir dir) {
	int ballMinArea = (int)Math::map(ball->distance, 0.0f, 5.0f, 32.0f, 2.0f);

    //if (ball->area < Config::ballMinArea) {
    if (ball->area < ballMinArea) {
		//std::cout << "@ BALL AREA TOO SMALL: " << ball->area << " VS " << Config::ballMinArea << std::endl;

        return false;
    }

	/*float sizeRatio = (float)ball->width / (float)ball->height;

	if (sizeRatio > Config::maxBallSizeRatio || sizeRatio < 1.0f / Config::maxBallSizeRatio) {
		return false;
	}*/

	if (!ball->behind && ball->y - ball->height / 2 > Config::cameraHeight - 85) {
		//std::cout << "@ BALL ON ROBOT IN FRONT" << std::endl;

		return false;
	}

	if (ball->behind && ball->y - ball->height / 2 > Config::cameraHeight - 135) {
		//std::cout << "@ BALL ON ROBOT FROM BEHIND" << std::endl;

		return false;
	}

	int ballRadius = getBallRadius(ball->width, ball->height);
	int senseRadius = getBallSenseRadius(ballRadius, ball->distance);

	if (ball->y + ballRadius < Config::surroundSenseThresholdY) {
		float surroundMetric = getSurroundMetric(
			ball->x,
			(int)((float)ball->y + (float)ballRadius * 0.75f),
			senseRadius,
			validBallBgColors,
			"",
			1
		);

		//std::cout << "Surround: " << surroundMetric << std::endl;

		if (surroundMetric != -1.0f && surroundMetric < Config::minValidBallSurroundThreshold) {
			//std::cout << "@ BALL SURROUND FAIL: " << surroundMetric << " VS " << Config::minValidBallSurroundThreshold << std::endl;

			return false;
		}
	}

	if (ball->y + ballRadius < Config::ballPathSenseStartY) {
		PathMetric pathMetric = getPathMetric(
			Config::cameraWidth / 2,
			Config::ballPathSenseStartY,
			ball->x,
			//(int)((float)ball->y + (float)ballRadius * 0.75f + (float)senseRadius),
			ball->y + ballRadius + senseRadius / 2 + 10,
			validBallPathColors
			//,"green"
		);

		//std::cout << "Ball path: " << pathMetric << std::endl;

		if (
			pathMetric.percentage < Config::minValidBallPathThreshold
			|| pathMetric.out
			//|| !pathMetric.validColorFound
			//|| pathMetric.invalidSpree > getBallMaxInvalidSpree(ball->y + ball->height / 2)
		) {
			//std::cout << "@ BALL PATH FAIL: " << pathMetric.percentage << " VS " << Config::minValidBallPathThreshold << ", OUT: " << (pathMetric.out ? "YES" : "NO") << std::endl;

			return false;
		}
	}

	if (isBallInGoal(ball, dir)) {
		//std::cout << "@ BALL IN GOAL FAIL" << std::endl;

		return false;
	}

    return true;
}

bool Vision::isBallInGoal(Object* ball, Dir dir) {
	// think's the ball is in goal when it's near the side posts
	/*if (ball->distance < Config::ballInGoalConsiderMaxDistance) {
		int ballRadius = getBallRadius(ball->width, ball->height);
		int senseRadius = getBallSenseRadius(ballRadius, ball->distance);

		float surroundMetric = getSurroundMetric(
			ball->x,
			(int)((float)ball->y - (float)ballRadius * 0.5f),
			senseRadius,
			goalColors,
			"",
			-1,
			true
		);

		if (surroundMetric != -1.0f && surroundMetric > Config::ballInGoalSurroundThreshold) {
			//std::cout << "@ BALL IN GOAL SURROUND: " << surroundMetric << " VS " << Config::ballInGoalThreshold << std::endl;

			return true;
		}
	}*/

	/*if (dir == Dir::FRONT) {
		for (ObjectListItc it = frontGoals.begin(); it != frontGoals.end(); it++) {
			if (ball->contains(*it)) {
				//std::cout << "@ BALL IN GOAL INTERSECTS FRONT" << std::endl;

				return true;
			}
		}
	} else if (dir == Dir::REAR) {
		for (ObjectListItc it = rearGoals.begin(); it != rearGoals.end(); it++) {
			if (ball->contains(*it)) {
				//std::cout << "@ BALL IN GOAL INTERSECTS REAR" << std::endl;

				return true;
			}
		}
	}*/

	return false;
}

/*int Vision::getBallMaxInvalidSpree(int y) {
	return y / 20.0f; // @TODO Something reasonable..
}

int Vision::getGoalMaxInvalidSpree(int y) {
	return y / 20.0f;
}*/

int Vision::getBallRadius(int width, int height) {
	return (int)(Math::max((float)width, (float)height) / 2.0f);
}

int Vision::getBallSenseRadius(int ballRadius, float distance) {
	return (int)Math::min((float)ballRadius * 1.5f * Math::max(distance / 2.0f, 1.0f) + 10.0f, (float)Config::maxBallSenseRadius);
}

int Vision::getPixelsBelow(int startX, int startY, std::vector<std::string> validColors, int allowedWrongPixels) {
	int wrongPixelCount = 0;
	int validPixelCount = 0;
	int senseX = startX;
	bool unsegmentedAllowed = find(validColors.begin(), validColors.end(), std::string("none")) != validColors.end();
	bool debug = canvas.data != NULL;
	Blobber::Color* color;

	for (int senseY = startY + 1; senseY < height; senseY++) {
		color = getColorAt(senseX, senseY);

		if (color == NULL) {
			if (unsegmentedAllowed) {
				validPixelCount++;

				if (debug) canvas.drawMarker(senseX, senseY, 0, 128, 0);
			} else {
				wrongPixelCount++;

				if (debug) canvas.drawMarker(senseX, senseY, 128, 0, 0);
			}
		} else {
			if (find(validColors.begin(), validColors.end(), std::string(color->name)) != validColors.end()) {
				validPixelCount++;

				if (debug) canvas.drawMarker(senseX, senseY, 0, 128, 0);
			} else {
				wrongPixelCount++;

				if (debug) canvas.drawMarker(senseX, senseY, 128, 0, 0);
			}
		}

		if (wrongPixelCount > allowedWrongPixels) {
			if (debug) canvas.drawMarker(senseX, senseY, 255, 0, 0);

			break;
		}
	}

	return validPixelCount;
}

/*int Vision::getPixelRowAt(float distance) {
	CameraTranslator::CameraPosition pos = cameraTranslator->getCameraPosition(0, distance);

	return pos.y;
}*/

CameraTranslator::CameraPosition Vision::getPixelAt(float distanceX, float distanceY) {
	return cameraTranslator->getCameraPosition(distanceX, distanceY);
}

// TODO Implement..
Math::Point Vision::getScreenCoords(float distanceX, float distanceY) {
	return Math::Point(0, 0);
}

Vision::Distance Vision::getDistance(int x, int y) {
	/*int realX = x;
	int realY = y;

	Util::correctCameraPoint(realX, realY);

	float distance;

    if (dir == FRONT) {
		distance = frontDistanceLookup.getValue((float)realY);
    } else {
        distance = rearDistanceLookup.getValue((float)realY);
    }

	return Math::max(distance + Config::distanceCorrection, 0.01f);*/

	CameraTranslator::WorldPosition pos = cameraTranslator->getWorldPosition(x, y);

	//if (pos.dx < -20.0f || pos.dx > 20.0f) std::cout << "- Invalid distance dx: " << pos.dx << " for " << x << "x" << y << ", dir: " << (dir == Dir::FRONT ? "front" : "rear") << std::endl;
	//if (pos.dy < 0.0f || pos.dy > 20.0f) std::cout << "- Invalid distance dy: " << pos.dy << " for " << x << "x" << y << ", dir: " << (dir == Dir::FRONT ? "front" : "rear") << std::endl;
	//if (pos.distance < 0.0f || pos.distance > 20.0f) std::cout << "- Invalid distance: " << pos.distance << " for " << x << "x" << y << ", dir: " << (dir == Dir::FRONT ? "front" : "rear") << std::endl;

	return Distance(pos.dx, pos.dy, pos.distance, pos.angle);
}

float Vision::getAngle(int x, int y) {
	/*int realX = x;
	int realY = y;

	Util::correctCameraPoint(realX, realY);

	// last working
	float centerOffset = (float)(realX - (Config::cameraWidth / 2.0f)),
		angle = Math::degToRad(centerOffset / 11.5f);

	if (dir == Dir::REAR) {
		if (angle < 0.0f) {
			angle += Math::PI;
		} else {
			angle -= Math::PI;
		}
	}

	return angle;*/

	CameraTranslator::WorldPosition pos = cameraTranslator->getWorldPosition(x, y);

	float angle = pos.angle;

	if (dir == Dir::REAR) {
		if (angle < 0.0f) {
			angle += Math::PI;
		} else {
			angle -= Math::PI;
		}
	}

	return angle;
}

Blobber::Color* Vision::getColorAt(int x, int y) {
    return blobber->getColorAt(x, y);
}

// TODO When scanning the underside then some on the topside are also still created
float Vision::getSurroundMetric(int x, int y, int radius, std::vector<std::string> validColors, std::string requiredColor, int side, bool allowNone) {
	int matches = 0;
	int misses = 0;
    int points = radius * 2;
    bool requiredColorFound = false;
    bool debug = canvas.data != NULL;

	int start = 0;
	int sensePoints = points;
	
	if (side == 1) {
		start = points / 2;
		sensePoints = points / 2 + 1;
	} else if (side == -1) {
		sensePoints = points / 2 + 1;
	}

    for (int i = start; i <= start + sensePoints; i++) {
        double t = 2 * Math::PI * i / points + Math::PI;

        int senseX = (int)(x + radius * cos(t));
        int senseY = (int)(y + radius * sin(t));

		if (
			senseX < 0
			|| senseX > width - 1
			|| senseY < 0
			|| senseY > Config::surroundSenseThresholdY
		) {
			continue;
		}

        Blobber::Color* color = getColorAt(senseX, senseY);

        if (color != NULL) {
            if (find(validColors.begin(), validColors.end(), std::string(color->name)) != validColors.end()) {
                matches++;

                if (debug) {
                    canvas.drawMarker(senseX, senseY, 0, 200, 0);
                }
            } else {
				misses++;
			}

            if (requiredColor != "" && color->name == requiredColor) {
                requiredColorFound = true;
            }
        } else {
			if (!allowNone) {
				misses++;

				if (debug) {
					canvas.drawMarker(senseX, senseY, 200, 0, 0);
				}
			} else {
				if (debug) {
					canvas.drawMarker(senseX, senseY, 128, 128, 128);
				}
			}
        }
    }

	int sensedPoints = matches + misses;

	if (sensedPoints == 0) {
		return -1.0f;
	} else if (requiredColor != "" && !requiredColorFound) {
        return 0.0f;
    } else {
        return (float)matches / (float)sensedPoints;
    }
}

Vision::PathMetric Vision::getPathMetric(int x1, int y1, int x2, int y2, std::vector<std::string> validColors, std::string requiredColor) {
	x1 = (int)Math::limit((float)x1, 0.0f, (float)Config::cameraWidth);
	x2 = (int)Math::limit((float)x2, 0.0f, (float)Config::cameraWidth);
	y1 = (int)Math::limit((float)y1, 0.0f, (float)Config::cameraHeight);
	y2 = (int)Math::limit((float)y2, 0.0f, (float)Config::cameraHeight);

	int originalX1 = x1;
	int originalX2 = x2;
	int originalY1 = y1;
	int originalY2 = y2;
	
	if (y2 > y1) {
		return PathMetric(1.0f, 0, false, false);
	}
	
	int F, x, y;
    int pixelCounter = 0;
    int senseCounter = 0;
    int senseStep = 3;
    const int maxSensePoints = 512;
    int senseX[maxSensePoints];
    int senseY[maxSensePoints];
	int invalidSpree = 0;
	int longestInvalidSpree = 0;
	//int scaler = 10;

    if (x1 > x2) {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    if (x1 == x2) {
        if (y1 > y2) {
            std::swap(y1, y2);
        }

        x = x1;
        y = y1;

        while (y <= y2) {
			//senseStep = (y + scaler) / scaler;

            if (pixelCounter % senseStep == 0 && senseCounter < maxSensePoints) {
                senseX[senseCounter] = x;
                senseY[senseCounter] = y;
                senseCounter++;
            }

            pixelCounter++;

            y++;
        }
    } else if (y1 == y2) {
        x = x1;
        y = y1;

		//senseStep = (y + scaler) / scaler;

        while (x <= x2) {
            if (pixelCounter % senseStep == 0 && senseCounter < maxSensePoints) {
                senseX[senseCounter] = x;
                senseY[senseCounter] = y;
                senseCounter++;
            }

            pixelCounter++;

            x++;
        }
    } else {
        int dy = y2 - y1;
        int dx = x2 - x1;
        int dy2 = (dy << 1);
        int dx2 = (dx << 1);
        int sub = dy2 - dx2;
        int sum = dy2 + dx2;

        if (dy >= 0) {
            if (dy <= dx) {
                F = dy2 - dx;
                x = x1;
                y = y1;

                while (x <= x2) {
					//senseStep = (y + scaler) / scaler;

                    if (pixelCounter % senseStep == 0 && senseCounter < maxSensePoints) {
                        senseX[senseCounter] = x;
                        senseY[senseCounter] = y;
                        senseCounter++;
                    }

                    pixelCounter++;

                    if (F <= 0) {
                        F += dy2;
                    } else {
                        y++;
                        F += sub;
                    }

                    x++;
                }
            } else {
                F = dx2 - dy;
                y = y1;
                x = x1;

                while (y <= y2) {
					//senseStep = (y + scaler) / scaler;

                    if (pixelCounter % senseStep == 0 && senseCounter < maxSensePoints) {
                        senseX[senseCounter] = x;
                        senseY[senseCounter] = y;
                        senseCounter++;
                    }

                    pixelCounter++;

                    if (F <= 0) {
                        F += dx2;
                    } else {
                        x++;
                        F -= sub;
                    }

                    y++;
                }
            }
        } else {
            if (dx >= -dy) {
                F = -dy2 - dx;
                x = x1;
                y = y1;

                while (x <= x2) {
					//senseStep = (y + scaler) / scaler;

                    if (pixelCounter % senseStep == 0 && senseCounter < maxSensePoints) {
                        senseX[senseCounter] = x;
                        senseY[senseCounter] = y;
                        senseCounter++;
                    }

                    pixelCounter++;

                    if (F <= 0) {
                        F -= dy2;
                    } else {
                        y--;
                        F -= sum;
                    }

                    x++;
                }
            } else {
                F = dx2 + dy;
                y = y1;
                x = x1;

                while (y >= y2) {
					//senseStep = (y + scaler) / scaler;

                    if (pixelCounter % senseStep == 0 && senseCounter < maxSensePoints) {
                        senseX[senseCounter] = x;
                        senseY[senseCounter] = y;
                        senseCounter++;
                    }

                    pixelCounter++;

                    if (F <= 0) {
                        F += dx2;
                    } else {
                        x++;
                        F += sum;
                    }

                    y--;
                }
            }
        }
    }

    int matches = 0;
	int blacksInRow = 0;
	int maxBlacksInRow = 10;
    bool debug = canvas.data != NULL;
    bool requiredColorFound = false;
	bool sawGreen = false;
	bool sawWhite = false;
	bool sawBlack = false;
	bool sawWhiteBeforeBlack = false;
	int previousBlack = 0;
	bool crossingGreenWhiteBlackGreen = false;
	bool tooManyBlacksInRow = false;
	std::string firstColor = "";
	std::string lastColor = "";

	int start = originalX1 < originalX2 ? 0 : senseCounter - 1;
	int step = originalX1 < originalX2 ? 1 : -1;
	int sampleCount = 0;
	float distanceStep = 0.05f;
	//float distance1, distance2;

    for (int i = start; (originalX1 < originalX2 ? i < senseCounter : i >= 0); i += step) {
        x = senseX[i];
        y = senseY[i];

		/*if (y > Config::cameraHeight / 4) {
			// sample less points near by and more in the distance
			distance1 = getDistance(x, y).straight;
			distance2 = Math::round(distance1 / distanceStep, 0) * distanceStep;

			if (Math::abs(distance1 - distance2) > distanceStep / 5.0f) {
				continue;
			}
		}*/

		sampleCount++;

		Blobber::Color* color = getColorAt(x, y);

        if (color != NULL) {
			if (firstColor == "") {
				firstColor = std::string(color->name);
			}

			lastColor = std::string(color->name);

			if (strcmp(color->name, "black") == 0) {
				blacksInRow++;

				// TODO Review
				/*if (blacksInRow > maxBlacksInRow) {
					tooManyBlacksInRow = true;

					break;
				}*/
			} else {
				blacksInRow = 0;
			}

			if (strcmp(color->name, "green") == 0) {
				if (!sawGreen) {
					sawGreen = true;

					if (debug) {
						canvas.drawMarker(x, y, 0, 128, 0);
					}
				} else if ((sawWhite || firstColor == "white") && previousBlack >= 1) {
					crossingGreenWhiteBlackGreen = true;

					if (debug) {
						canvas.drawMarker(x, y, 128, 0, 0);
					}

					break;
				}
			} else if (!sawWhite /*&& sawGreen*/ && strcmp(color->name, "white") == 0) {
				sawWhite = true;

				if (debug) {
					canvas.drawMarker(x, y, 128, 128, 128);

					continue;
				}
			}
			
			if (strcmp(color->name, "black") == 0) {
				sawBlack = true;
				previousBlack++;

				if (sawWhite) {
					sawWhiteBeforeBlack = true;
				}
			} else {
				previousBlack = 0;
			}

            if (find(validColors.begin(), validColors.end(), std::string(color->name)) != validColors.end()) {
                matches++;

				if (invalidSpree > longestInvalidSpree) {
					longestInvalidSpree = invalidSpree;
				}

				invalidSpree = 0;

                if (debug) {
                    canvas.drawMarker(x, y, 0, 200, 0);
                }
            } else {
				invalidSpree++;

				if (debug) {
                    canvas.drawMarker(x, y, 200, 0, 0);
                }
			}

            if (requiredColor != "" && color->name == requiredColor) {
                requiredColorFound = true;
            }
        } else {
            if (debug) {
                canvas.drawMarker(x, y, 200, 0, 0);
            }

			invalidSpree++;
        }
    }

	// TODO improve and restore, can cause false positives when there are balls in front of other balls
	/*if (
		(lastColor == "black" && sawWhite)
		|| (firstColor == "white" && sawWhiteBeforeBlack && lastColor == "green")
	) {
		std::cout << "@ OUT LATE" << std::endl;

		crossingGreenWhiteBlackGreen = true;
	}*/

	if (senseCounter < 20) {
		return PathMetric(1.0f, 0, true, false);
	}

	float percentage = (float)matches / (float)sampleCount;
	bool validColorFound = requiredColor == "" || requiredColorFound;
	bool isOut = crossingGreenWhiteBlackGreen || tooManyBlacksInRow;

	/*std::cout << "@ blacksInRow: " << blacksInRow << std::endl;
	std::cout << "@ previousBlack: " << previousBlack << std::endl;
	std::cout << "@ crossingGreenWhiteBlackGreen: " << crossingGreenWhiteBlackGreen << std::endl;
	std::cout << "@ tooManyBlacksInRow: " << tooManyBlacksInRow << std::endl << std::endl;*/

	return PathMetric(percentage, longestInvalidSpree, validColorFound, isOut);
}

Vision::EdgeDistanceMetric Vision::getEdgeDistanceMetric(int x, int y, int width, int height, std::string color1, std::string color2) {
	Distance distance;
	EdgeDistance leftTopDistance;
	EdgeDistance rightTopDistance;
	int leftTopDistanceX = -1;
	int rightTopDistanceX = -1;
	int padding = (int)((float)width * 0.2f);
	Blobber::Color* color;
	bool colorFound;

	for (int senseX = x + padding; senseX <= x + width - padding * 2; senseX++) {
		for (int senseY = y; senseY <= y + height; senseY++) {
			color = getColorAt(senseX, senseY);

			if (color == NULL) {
				//canvas.setPixelAt(senseX, senseY, 255, 255, 255);

				continue;
			}

			colorFound = strcmp(color->name, color1.c_str()) == 0 || strcmp(color->name, color2.c_str());

			if (colorFound) {
				distance = getDistance(senseX, senseY);

				if (leftTopDistanceX == -1 || senseX < leftTopDistanceX) {
					leftTopDistance = EdgeDistance(senseX, senseY, distance.straight);
					leftTopDistanceX = senseX;
				}

				if (rightTopDistanceX == -1 || senseX > rightTopDistanceX) {
					rightTopDistance = EdgeDistance(senseX, senseY, distance.straight);
					rightTopDistanceX = senseX;
				}

				canvas.setPixelAt(senseX, senseY, 0, 255, 0);

				break;
			} else {
				canvas.setPixelAt(senseX, senseY, 255, 0, 0);
			}
		}
	}

	if (leftTopDistance.distance != -1) {
		canvas.fillBoxCentered(leftTopDistance.screenX, leftTopDistance.screenY, 10, 10, 255, 0, 0);
		canvas.drawText(leftTopDistance.screenX, leftTopDistance.screenY + 10, Util::toString(leftTopDistance.distance) + "m", 0, 0, 0);
	}

	if (rightTopDistance.distance != -1) {
		canvas.fillBoxCentered(rightTopDistance.screenX, rightTopDistance.screenY, 10, 10, 255, 0, 0);
		canvas.drawText(rightTopDistance.screenX, rightTopDistance.screenY + 10, Util::toString(rightTopDistance.distance) + "m", 0, 0, 0);
	}

	return EdgeDistanceMetric(leftTopDistance, rightTopDistance);
}

Obstruction Vision::getGoalPathObstruction() {
	Obstruction obstruction = Obstruction::NONE;
	//float corridorWidth = 0.1f;
	float yStep = 0.05f;
	float xStep = 0.1f;
	float xSteps = 4.0f;
	float startDistance = 0.2f;
	float endDistance = 6.0f; // TODO Something smarter?
	int stopGoalColorCount = 4; // stop searching any further if found this many goal colors
	float goalPathObstructedThreshold = 0.4f;

	float xDistance, yDistance;
	bool debug = canvas.data != NULL;
	Blobber::Color* color;
	CameraTranslator::CameraPosition pos;
	int sampleCount = 0;
	int validCount = 0;
	int validCountLeft = 0;
	int validCountRight = 0;
	int goalColorCount = 0;
	float goalDistance = -1.0f;
	bool running = true;

	// TODO make sure finds target side color in the end
	// sample points every step distances
	for (yDistance = startDistance; yDistance < endDistance; yDistance += yStep) {
		for (xDistance = -xStep * xSteps / 2 + xStep / 2.0f; xDistance < xStep * xSteps / 2; xDistance += xStep) {
			// find corridor positions
			pos = cameraTranslator->getCameraPosition(xDistance, yDistance);
			color = getColorAt(pos.x, pos.y);
			
			if (color != NULL) {
				if (strcmp(color->name, "blue-goal") == 0 || strcmp(color->name, "yellow-goal") == 0) {
					goalColorCount++;

					// stop if found enough goal colors
					if (goalColorCount >= stopGoalColorCount) {
						if (debug) {
							canvas.drawMarker(pos.x, pos.y, 255, 0, 0);
						}

						running = false;

						goalDistance = getDistance(pos.x, pos.y).y;

						break;
					} else {
						if (debug) {
							canvas.drawMarker(pos.x, pos.y, 128, 128, 128);
						}
					}

					continue;
				}

				if (find(goalObstructedValidColors.begin(), goalObstructedValidColors.end(), std::string(color->name)) != goalObstructedValidColors.end()) {
					if (debug) {
						canvas.drawMarker(pos.x, pos.y, 0, 255, 0);
					}
					
					validCount++;

					if (xDistance < 0) {
						validCountLeft++;
					} else {
						validCountRight++;
					}
				} else {
					if (debug) {
						canvas.drawMarker(pos.x, pos.y, 128, 0, 0);
					}
				}

				sampleCount++;
			} else {
				sampleCount++;

				if (debug) {
					canvas.drawMarker(pos.x, pos.y, 128, 0, 0);
				}
			}
		}

		if (!running) {
			break;
		}
	}

	float leftValidSamplesRatio = (float)validCountLeft / (float)(sampleCount / 2);
	float rightValidSamplesRatio = (float)validCountRight / (float)(sampleCount / 2);

	// not reliable near the goal with few samples
	if (sampleCount > 50 && goalDistance > 1.5f) {
		bool obstructed = leftValidSamplesRatio < (1.0f - goalPathObstructedThreshold) || rightValidSamplesRatio < (1.0f - goalPathObstructedThreshold);

		if (obstructed) {
			if (leftValidSamplesRatio < rightValidSamplesRatio) {
				obstruction = Obstruction::LEFT;
			} else {
				obstruction = Obstruction::RIGHT;
			}
		}
	}

	//std::cout << "@ Obstruction ratio samples: " << sampleCount << ", goal distance: " << goalDistance << ", left: " << leftValidSamplesRatio << ", right: " << rightValidSamplesRatio << ", side: " << (obstruction == Obstruction::LEFT ? "LEFT" : obstruction == Obstruction::RIGHT ? "RIGHT" : "NONE") << std::endl;

	return obstruction;
}

float Vision::getColorDistance(std::string colorName, int x1, int y1, int x2, int y2) {
	x1 = (int)Math::limit((float)x1, 0.0f, (float)Config::cameraWidth);
	x2 = (int)Math::limit((float)x2, 0.0f, (float)Config::cameraWidth);
	y1 = (int)Math::limit((float)y1, 0.0f, (float)Config::cameraHeight);
	y2 = (int)Math::limit((float)y2, 0.0f, (float)Config::cameraHeight);

	int originalX1 = x1;
	int originalX2 = x2;
	int originalY1 = y1;
	int originalY2 = y2;
	
	if (y2 > y1) {
		return -1.0f;
	}
	
	int F, x, y;
    int pixelCounter = 0;
    int senseCounter = 0;
    int senseStep = 3;
    const int maxSensePoints = 512;
    int senseX[maxSensePoints];
    int senseY[maxSensePoints];
	int invalidSpree = 0;
	int longestInvalidSpree = 0;
	//int scaler = 10;

    if (x1 > x2) {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    if (x1 == x2) {
        if (y1 > y2) {
            std::swap(y1, y2);
        }

        x = x1;
        y = y1;

        while (y <= y2) {
			//senseStep = (y + scaler) / scaler;

            if (pixelCounter % senseStep == 0 && senseCounter < maxSensePoints) {
                senseX[senseCounter] = x;
                senseY[senseCounter] = y;
                senseCounter++;
            }

            pixelCounter++;

            y++;
        }
    } else if (y1 == y2) {
        x = x1;
        y = y1;

		//senseStep = (y + scaler) / scaler;

        while (x <= x2) {
            if (pixelCounter % senseStep == 0 && senseCounter < maxSensePoints) {
                senseX[senseCounter] = x;
                senseY[senseCounter] = y;
                senseCounter++;
            }

            pixelCounter++;

            x++;
        }
    } else {
        int dy = y2 - y1;
        int dx = x2 - x1;
        int dy2 = (dy << 1);
        int dx2 = (dx << 1);
        int sub = dy2 - dx2;
        int sum = dy2 + dx2;

        if (dy >= 0) {
            if (dy <= dx) {
                F = dy2 - dx;
                x = x1;
                y = y1;

                while (x <= x2) {
					//senseStep = (y + scaler) / scaler;

                    if (pixelCounter % senseStep == 0 && senseCounter < maxSensePoints) {
                        senseX[senseCounter] = x;
                        senseY[senseCounter] = y;
                        senseCounter++;
                    }

                    pixelCounter++;

                    if (F <= 0) {
                        F += dy2;
                    } else {
                        y++;
                        F += sub;
                    }

                    x++;
                }
            } else {
                F = dx2 - dy;
                y = y1;
                x = x1;

                while (y <= y2) {
					//senseStep = (y + scaler) / scaler;

                    if (pixelCounter % senseStep == 0 && senseCounter < maxSensePoints) {
                        senseX[senseCounter] = x;
                        senseY[senseCounter] = y;
                        senseCounter++;
                    }

                    pixelCounter++;

                    if (F <= 0) {
                        F += dx2;
                    } else {
                        x++;
                        F -= sub;
                    }

                    y++;
                }
            }
        } else {
            if (dx >= -dy) {
                F = -dy2 - dx;
                x = x1;
                y = y1;

                while (x <= x2) {
					//senseStep = (y + scaler) / scaler;

                    if (pixelCounter % senseStep == 0 && senseCounter < maxSensePoints) {
                        senseX[senseCounter] = x;
                        senseY[senseCounter] = y;
                        senseCounter++;
                    }

                    pixelCounter++;

                    if (F <= 0) {
                        F -= dy2;
                    } else {
                        y--;
                        F -= sum;
                    }

                    x++;
                }
            } else {
                F = dx2 + dy;
                y = y1;
                x = x1;

                while (y >= y2) {
					//senseStep = (y + scaler) / scaler;

                    if (pixelCounter % senseStep == 0 && senseCounter < maxSensePoints) {
                        senseX[senseCounter] = x;
                        senseY[senseCounter] = y;
                        senseCounter++;
                    }

                    pixelCounter++;

                    if (F <= 0) {
                        F += dx2;
                    } else {
                        x++;
                        F += sum;
                    }

                    y--;
                }
            }
        }
    }

	int matches = 0;
    bool debug = canvas.data != NULL;
	int start = originalX1 < originalX2 ? 0 : senseCounter - 1;
	int step = originalX1 < originalX2 ? 1 : -1;

    for (int i = start; (originalX1 < originalX2 ? i < senseCounter : i >= 0); i += step) {
        x = senseX[i];
        y = senseY[i];

        Blobber::Color* color = getColorAt(x, y);

		if (color != NULL) {
			
			if (strcmp(color->name, colorName.c_str()) == 0) {
				if (debug) {
					//canvas.drawMarker(x, y, 0, 200, 0);
					canvas.fillBox(x - 5, y - 5, 10, 10, 255, 0, 0);
				}

				return getDistance(x, y).straight;
			} else {
				if (debug) {
					//canvas.drawMarker(x, y, 200, 0, 0);
				}
			}
		} else {
			if (debug) {
                //canvas.drawMarker(x, y, 64, 64, 64);
            }
		}
	}

	return -1.0f;
}

Vision::ColorDistance Vision::getColorDistance(std::string colorName) {
	float left = getColorDistance(
		colorName,
		Config::cameraWidth / 2, Config::colorDistanceStartY,
		0, 0
	);
	float leftMiddle = getColorDistance(
		colorName,
		Config::cameraWidth / 2, Config::colorDistanceStartY,
		Config::cameraWidth / 4, 0
	);
	float center = getColorDistance(
		colorName,
		Config::cameraWidth / 2, Config::colorDistanceStartY,
		Config::cameraWidth / 2, 0
	);
	float rightMiddle = getColorDistance(
		colorName,
		Config::cameraWidth / 2, Config::colorDistanceStartY,
		Config::cameraWidth / 2 + Config::cameraWidth / 4, 0
	);
	float right = getColorDistance(
		colorName,
		Config::cameraWidth / 2, Config::colorDistanceStartY,
		Config::cameraWidth, 0
	);

	/*getColorDistance(
		colorName,
		Config::cameraWidth - 10, 0,
		Config::cameraWidth / 2 - 10, Config::colorDistanceStartY
	);*/

	return ColorDistance(left, leftMiddle, center, rightMiddle, right);
}

Vision::ColorList Vision::getViewColorOrder() {
	ColorList colors;
	int x = Config::cameraWidth / 2;
	int y;
	Blobber::Color* color;
	bool debug = canvas.data != NULL;
	std::string lastColor = "";

	for (y = Config::ballPathSenseStartY; y >= 0; y -= 3) {
		color = getColorAt(x, y);

		if (color == NULL) {
			continue;
		}

		if (color->name != lastColor) {
			colors.push_back(color->name);

			lastColor = color->name;

			if (debug) {
				canvas.drawMarker(x, y, 255, 0, 0);
			}
		}
	}

	return colors;
}

float Vision::getBlockMetric(int x1, int y1, int blockWidth, int blockHeight, std::vector<std::string> validColors, int step) {
	bool debug = canvas.data != NULL;
	int matches = 0;
	int misses = 0;

	for (int x = (int)Math::max((float)x1, 0.0f); x < (int)Math::min((float)(x1 + blockWidth), (float)width); x += step) {
		for (int y = (int)Math::max((float)y1, 0.0f); y < (int)Math::min((float)(y1 + blockHeight), (float)height); y += step) {
			Blobber::Color* color = getColorAt(x, y);

			if (color != NULL) {
				if (find(validColors.begin(), validColors.end(), std::string(color->name)) != validColors.end()) {
					matches++;

					if (debug) {
						canvas.drawMarker(x, y, 0, 200, 0);
					}
				} else {
					misses++;

					if (debug) {
						canvas.drawMarker(x, y, 200, 0, 0);
					}
				}
			} else {
				misses++;

				if (debug) {
					canvas.drawMarker(x, y, 200, 0, 0);
				}
			}
		}
	}

	int points = matches + misses;

	return (float)matches / (float)points;
}

float Vision::getUndersideMetric(int x1, int y1, float distance, int blockWidth, int blockHeight, std::string targetColor, std::string targetColor2, std::vector<std::string> validColors, bool expand) {
	int minValidX = -1;
	int maxValidX = -1;
	int minValidY = -1;
	int maxValidY = -1;

	return getUndersideMetric(x1, y1, distance, blockWidth, blockHeight, targetColor, targetColor2, validColors, minValidX, minValidY, maxValidX, maxValidY, expand);
}

float Vision::getUndersideMetric(int x1, int y1, float distance, int blockWidth, int blockHeight, std::string targetColor, std::string targetColor2, std::vector<std::string> validColors, int& minValidX, int& minValidY, int& maxValidX, int& maxValidY, bool expand) {
	bool debug = canvas.data != NULL;
	int xStep = 6;
	int yStep = 6;
	int gapStep = 3;
	int senseSteps = (int)Math::max((float)Config::undersideMetricBaseSteps / distance, 3.0f);
	int matches = 0;
	int misses = 0;
	int stepsBelow;
	bool sawGreenOrBlack;
	bool sawWhite = false;
	bool sawBlack = false;
	const char* targetColorName = targetColor.c_str();
	const char* targetColorName2 = targetColor2.c_str();
	std::string lastColorName;
	Blobber::Color* color;

	minValidX = -1;
	maxValidX = -1;
	minValidY = -1;
	maxValidY = -1;

	for (int x = (int)Math::max((float)x1, 0.0f); x < (int)Math::min((float)(x1 + blockWidth), (float)width); x += xStep) {
		if (x > width - 1) {
			break;
		}

		stepsBelow = 0;

		for (int y = y1; y < (int)Math::min((float)(y1 + blockHeight * 4), (float)height); y += yStep) {
			if (y > height - 1) {
				break;
			}

			color = getColorAt(x, y);

			if (color == NULL || (strcmp(color->name, targetColorName) != 0 && strcmp(color->name, targetColorName2) != 0)) {
				if (debug) {
					canvas.drawMarker(x, y, 64, 64, 64);
				}

				continue;
			}

			if (minValidX == -1) {
				minValidX = x;
			}

			if (minValidY == -1 || y < minValidY) {
				minValidY = y;
			}

			if (x > maxValidX) {
				maxValidX = x;
			}

			if (y > maxValidY) {
				maxValidY = y;
			}

			if (debug) {
				canvas.drawMarker(x, y, 255, 255, 255);
			}

			//std::cout << "! DOWN FROM " << (y + yStep) << " to " << y + blockHeight << std::endl;

			for (int senseY = y + yStep; senseY < (int)Math::min((float)(y + blockHeight * 4), (float)height); senseY += yStep) {
				if (senseY > height - 1) {
					break;
				}
		
				color = getColorAt(x, senseY);

				//std::cout << "! SENSE " << x << " " << senseY << " | " << blockHeight << std::endl;

				if (color != NULL && (strcmp(color->name, targetColorName) == 0 || strcmp(color->name, targetColorName2) == 0)) {
					if (senseY > maxValidY) {
						maxValidY = senseY;
					}

					if (debug) {
						canvas.drawMarker(x, senseY, 0, 0, 0);
					}
				} else {
					if (debug) {
						canvas.drawMarker(x, senseY, 255, 255, 255);
					}

					sawGreenOrBlack = false;
					bool retryTarget = false;
					int runMatches = 0;
					int runMisses = 0;

					for (int gapY = senseY + gapStep; gapY < (int)Math::min((float)(senseY + senseSteps * yStep), (float)height); gapY += gapStep) {
						if (gapY > height - 1) {
							break;
						}

						color = getColorAt(x, gapY);

						if (color != NULL) {
							if (strcmp(color->name, targetColorName) == 0 || strcmp(color->name, targetColorName2) == 0) {
								retryTarget = true;

								if (gapY > maxValidY) {
									maxValidY = gapY;
								}

								break;
							}

							if (
								!sawGreenOrBlack
								&& (
									strcmp(color->name, "green") == 0
									|| strcmp(color->name, "black") == 0
								)
							) {
								sawGreenOrBlack = true;
							}

							if (sawGreenOrBlack) {
								if (!sawWhite && strcmp(color->name, "white") == 0) {
									sawWhite = true;
								} else if (!sawBlack && strcmp(color->name, "black") == 0) {
									sawBlack = true;
								}
							}

							if (
								sawGreenOrBlack
								&& find(validColors.begin(), validColors.end(), std::string(color->name)) != validColors.end()
							) {
								runMatches++;

								lastColorName = std::string(color->name);

								if (debug) {
									canvas.drawMarker(x, gapY, 0, 200, 0, true);
								}
							} else {
								runMisses++;

								if (debug) {
									canvas.drawMarker(x, gapY, 200, 0, 0, true);
								}
							}
						} else {
							// allow one invalid color after white/black
							if (lastColorName != "black" && lastColorName != "white") {
								canvas.drawMarker(x, gapY, 100, 0, 0, true);

								runMisses++;
							} else {
								lastColorName = "";

								if (debug) {
									canvas.drawMarker(x, gapY, 255, 255, 0, true);
								}
							}
						}
					}

					if (!retryTarget) {
						matches += runMatches;
						misses += runMisses;

						break;
					} else {

					}
				}
			}

			break;
		}
	}
	
	/*if (expand) {
		int maxGap = 2;
		int expandY = 60;
		int expandX = 60;

		if (debug) {
			canvas.drawBox(minValidX, minValidY, maxValidX - minValidX, maxValidY - minValidY, 255, 0, 0);
			//canvas.drawBox(maxValidX, 0, Config::cameraWidth - maxValidX, Math::min(maxValidY + 60, Config::cameraHeight - 1), 255, 255, 255);
		}

		for (int y = 0; y < Math::min(maxValidY + expandY, Config::cameraHeight - 1); y += yStep) {
			int gap = 0;

			for (int x = maxValidX; x < maxValidX + expandX; x += xStep) {
				color = getColorAt(x, y);

				if (color != NULL && (strcmp(color->name, targetColorName) == 0 || strcmp(color->name, targetColorName2) == 0)) {
					gap = 0;

					if (x > maxValidX) maxValidX = x;
					if (y > maxValidY) maxValidY = y;

					if (debug) {
						canvas.drawMarker(x, y, 255, 255, 255);
					}
				} else {
					gap++;

					if (debug) {
						canvas.drawMarker(x, y, 200, 0, 0);
					}
				}

				if (gap > maxGap) {
					if (debug) {
						canvas.drawMarker(x, y, 100, 0, 0);
					}

					break;
				}
			}
		}
	}*/

	// sure?
	if (y1 > Config::undersideMetricBlackWhiteMinY && !sawWhite && !sawBlack) {
		return 0.0f;
	}

	// sure?
	if (matches < Config::undersideMetricValidGoalMinMatches) {
		return 0.0f;
	}

	int points = matches + misses;

	return (float)matches / (float)points;
}

/*void Vision::updateViewObstructed() {
	float validColorsPercentage = getBlockMetric(
		Config::viewObstructedX,
		Config::viewObstructedY,
		Config::viewObstructedWidth,
		Config::viewObstructedHeight,
		viewObstructedValidColors,
		20
	);

	if (validColorsPercentage < Config::viewObstructedThreshold) {
		viewObstructed = true;
	} else {
		viewObstructed = false;
	}
}*/

void Vision::updateObstructions() {
	float leftMetric = getBlockMetric(
		Config::cameraWidth / 2 - Config::obstructionsSenseWidth,
		Config::obstructionsStartY,
		Config::obstructionsSenseWidth,
		Config::obstructionsSenseHeight,
		viewObstructedValidColors,
		20
	);

	float rightMetric = getBlockMetric(
		Config::cameraWidth / 2,
		Config::obstructionsStartY,
		Config::obstructionsSenseWidth,
		Config::obstructionsSenseHeight,
		viewObstructedValidColors,
		20
	);

	obstructionSide = Obstruction::NONE;

	if (leftMetric < Config::obstructedThreshold && rightMetric < Config::obstructedThreshold) {
		obstructionSide = Obstruction::BOTH;
	} else if (leftMetric < Config::obstructedThreshold) {
		obstructionSide = Obstruction::LEFT;
	} else if (rightMetric < Config::obstructedThreshold) {
		obstructionSide = Obstruction::RIGHT;
	}
}

void Vision::updateColorDistances() {
	whiteDistance = getColorDistance("white");
	blackDistance = getColorDistance("black");
}

void Vision::updateColorOrder() {
	colorOrder = getViewColorOrder();

	/*std::cout << "Color order " << dir << std::endl;

	for (int i = 0; i < colorOrder.size(); i++) {
		std::cout << "  " << i << ": " << colorOrder[i] << std::endl;
	}

	std::cout << std::endl;*/
}

Object* Vision::Results::getClosestBall(Dir dir, bool nextClosest) {
	Object* blueGoal = getLargestGoal(Side::BLUE);
	Object* yellowGoal = getLargestGoal(Side::YELLOW);

	float closestDistance = 100.0f;
	Object* ball;
	Object* closestBall = NULL;
	Object* nextClosestBall = NULL;

	if (front != NULL && dir != Dir::REAR) {
		for (ObjectListItc it = front->balls.begin(); it != front->balls.end(); it++) {
			ball = *it;

			if (isBallInGoal(ball, blueGoal, yellowGoal)) {
				continue;
			}

			if (closestBall == NULL || ball->distance < closestDistance) {
				if (closestBall != NULL) {
					nextClosestBall = closestBall;
				}

				closestBall = ball;
				closestDistance = ball->distance;
			}
		}
	}

	if (rear != NULL && dir != Dir::FRONT) {
		for (ObjectListItc it = rear->balls.begin(); it != rear->balls.end(); it++) {
			ball = *it;

			if (isBallInGoal(ball, blueGoal, yellowGoal)) {
				continue;
			}

			if (closestBall == NULL || ball->distance < closestDistance) {
				if (closestBall != NULL) {
					nextClosestBall = closestBall;
				}

				closestBall = ball;
				closestDistance = ball->distance;
			}
		}
	}

	return nextClosest ? nextClosestBall : closestBall;
}

Object* Vision::Results::getFurthestBall(Dir dir) {
	Object* blueGoal = getLargestGoal(Side::BLUE);
	Object* yellowGoal = getLargestGoal(Side::YELLOW);

	float furthestDistance = -1.0f;
	Object* ball;
	Object* furthestBall = NULL;

	if (front != NULL && dir != Dir::REAR) {
		for (ObjectListItc it = front->balls.begin(); it != front->balls.end(); it++) {
			ball = *it;

			if (isBallInGoal(ball, blueGoal, yellowGoal)) {
				continue;
			}

			if (furthestBall == NULL || ball->distance > furthestDistance) {
				furthestBall = ball;
				furthestDistance = ball->distance;
			}
		}
	}

	if (rear != NULL && dir != Dir::FRONT) {
		for (ObjectListItc it = rear->balls.begin(); it != rear->balls.end(); it++) {
			ball = *it;

			if (isBallInGoal(ball, blueGoal, yellowGoal)) {
				continue;
			}

			if (furthestBall == NULL || ball->distance > furthestDistance) {
				furthestBall = ball;
				furthestDistance = ball->distance;
			}
		}
	}

	return furthestBall;
}

Object* Vision::Results::getNextClosestBall(Dir dir) {
	return getClosestBall(dir, true);
}

Object* Vision::Results::getLargestGoal(Side side, Dir dir) {
	int area;
	int largestArea = 0;
	Object* goal;
	Object* largestGoal = NULL;

	if (front != NULL && dir != Dir::REAR) {
		for (ObjectListItc it = front->goals.begin(); it != front->goals.end(); it++) {
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

	if (rear != NULL && dir != Dir::FRONT) {
		for (ObjectListItc it = rear->goals.begin(); it != rear->goals.end(); it++) {
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
		if (dir == Dir::ANY) {
			Side otherSide = side == Side::BLUE ? Side::YELLOW : Side::BLUE;
			Dir sameDir = largestGoal->behind ? Dir::REAR : Dir::FRONT;
			Object* otherGaol = getLargestGoal(otherSide, sameDir);

			// don't use the found goal if other goal is also visible on the same side and is larger
			if (otherGaol != NULL && otherGaol->area > largestGoal->area) {
				return NULL;
			}
		}

		//lastLargestGoal.copyFrom(largestGoal);

		return largestGoal;
	}/* else if (
		lastLargestGoal.width > 0
		&& lastLargestGoal.type == side
		&& Util::duration(lastLargestGoal.lastSeenTime) < Config::fakeObjectLifetime
		&& !frontOnly
	) {
		return &lastLargestGoal;
	}*/ else {
		return NULL;
	}
}

Object* Vision::Results::getFurthestGoal(Dir dir) {
	return NULL;

	Object* largestYellow = getLargestGoal(Side::YELLOW, dir);
	Object* largestBlue = getLargestGoal(Side::BLUE, dir);

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
	}
}

bool Vision::Results::isBallInGoal(Object* ball) {
	return isBallInGoal(ball, getLargestGoal(Side::BLUE), getLargestGoal(Side::YELLOW));
}

bool Vision::Results::isBallInGoal(Object* ball, Object* blueGoal, Object* yellowGoal) {
	// causes problems with balls near the goal when looking at the goal at a large angle
	/*if (
		(blueGoal != NULL && blueGoal->behind == ball->behind && blueGoal->contains(ball))
		|| (yellowGoal != NULL && yellowGoal->behind == ball->behind && yellowGoal->contains(ball))
	) {
		//std::cout << "@ Ball in goal, ball distance: " << ball->distance << ", blue goal: " << (blueGoal != NULL ? Util::toString(blueGoal->distance) : "n/a") << ", yellow goal: " << (yellowGoal != NULL ? Util::toString(yellowGoal->distance) : "n/a") << std::endl;
		return true;
	}*/

	return false;
}

Vision::BallInWayMetric Vision::Results::getBallInWayMetric(ObjectList balls, int goalY) {
	Object* blueGoal = getLargestGoal(Side::BLUE);
	Object* yellowGoal = getLargestGoal(Side::YELLOW);

	int startY = Config::goalPathSenseStartY;
	int halfWidth = Config::cameraWidth / 2;
	float ballDiameter = 0.043f;
	//float ballInWayAngleThreshold = Math::degToRad(4.0f);
	//float ballInWayDistanceThreshold = ballDiameter * 4.0f;
	Object* ball;
	//float checkWidth;
	bool isBallInWay = false;
	float closestBallInWayDistance = -1.0f;
	float furthestBallInWayDistance = -1.0f;
	int ballInWayCount = 0;

	for (ObjectListItc it = balls.begin(); it != balls.end(); it++) {
		ball = *it;
		
		if (isBallInGoal(ball, blueGoal, yellowGoal) || ball->getDribblerDistance() < 0.02f) {
			continue;
		}

		// angle based
		/*float ballLeftX = ball->distanceX - ballDiameter * 1.5f;
		float ballRightX = ball->distanceX + ballDiameter * 1.5f;

		float ballLeftAngle = atan2(ballLeftX, ball->distanceY);
		float ballRightAngle = atan2(ballRightX, ball->distanceY);

		//std::cout << "@ ball angle: " << ball->angle << ", left: " << ballLeftAngle << ", right: " << ballRightAngle << std::endl;

		if (
			Math::abs(ballLeftAngle) < ballInWayAngleThreshold
			|| Math::abs(ball->angle) < ballInWayAngleThreshold
			|| Math::abs(ballRightAngle) < ballInWayAngleThreshold
			|| (ballLeftAngle < -ballInWayAngleThreshold && ballRightAngle > ballInWayAngleThreshold)
		) {
			return true;
		}*/

		float ballInWayDistanceThreshold = Math::map(ball->getDribblerDistance(), 0.0f, 2.0f, ballDiameter * 3.0f, ballDiameter * 6.0f);

		// distance based
		if (Math::abs(ball->distanceX) < ballInWayDistanceThreshold / 2.0f) {
			isBallInWay = true;
			ballInWayCount++;

			if (closestBallInWayDistance == -1.0f || ball->distanceY < closestBallInWayDistance) {
				closestBallInWayDistance = ball->distanceY;
			}

			if (furthestBallInWayDistance == -1.0f || ball->distanceY > furthestBallInWayDistance) {
				furthestBallInWayDistance = ball->distanceY;
			}
		}
		
		// pixels based
		//checkWidth = ball->width * 2.5f;

		/*if (
			ball->x - checkWidth < halfWidth && ball->x + checkWidth > halfWidth
			&& ball->y - ball->height < startY && ball->y  + ball->height > goalY
		) {
			return true;
		}*/
	}

	return BallInWayMetric(isBallInWay, ballInWayCount, closestBallInWayDistance, furthestBallInWayDistance);
}

bool Vision::Results::isRobotOut(Dir dir) {
	if (dir == Dir::ANY) {
		return isRobotOut(Dir::FRONT) || isRobotOut(Dir::REAR);
	}

	ColorList colorOrder;

	if (dir == Dir::FRONT && front != NULL) {
		colorOrder = front->colorOrder;
	} else if (dir == Dir::REAR && rear != NULL) {
		colorOrder = rear->colorOrder;
	} else {
		return false;
	}

	for (int i = 0; i < (int)colorOrder.size(); i++) {
		if (colorOrder[i] != "black") {
			continue;
		}

		// found black, search for black > white > green

		if ((int)colorOrder.size() < i + 3) {
			return false;
		}

		if (colorOrder[i + 1] == "white" && colorOrder[i + 2] == "green") {
			return true;
		}
	}

	return false;
}
