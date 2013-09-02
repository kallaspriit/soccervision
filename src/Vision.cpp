#include "Vision.h"
#include "Config.h"
#include "Maths.h"
#include "Util.h"

#include <iostream>
#include <algorithm>

Vision::Vision(int width, int height) : blobber(NULL), width(width), height(height), lastFrameFront(NULL), lastFrameRear(NULL), classificationFront(NULL), classificationRear(NULL), viewObstructed(false), robotInWay(0), blackDistance(-1.0f) {
    blobber = new Blobber();

    blobber->initialize(width, height);
    blobber->loadOptions(Config::blobberConfigFilename);
    blobber->enable(BLOBBER_DENSITY_MERGE);

    frontDistanceLookup.load("config/distance-front.cfg"/*, 0.13f*/);
    rearDistanceLookup.load("config/distance-rear.cfg");
    frontAngleLookup.load("config/angle-front.cfg");
    rearAngleLookup.load("config/angle-rear.cfg");

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

    validGoalPathColors.push_back("green");
    validGoalPathColors.push_back("white");
    validGoalPathColors.push_back("black");
    validGoalPathColors.push_back("ball");
    //validGoalPathColors.push_back("yellow-goal");
    //validGoalPathColors.push_back("blue-goal");

	goalColors.push_back("yellow-goal");
    goalColors.push_back("blue-goal");

    //blobber->enable(BLOBBER_DUAL_THRESHOLD);
    //blobber->setMapFilter(this);

    /*
    [Colors]
    (255,128,  0) 0.5000 5 ball
    (255,255,  0) 0.5000 5 yellow-goal
    (  0,  0,255) 0.5000 5 blue-goal
    (255,255,255) 0.5000 5 white
    (  0,255,  0) 0.5000 5 green
    ( 32, 32, 32) 0.5000 5 black

    [Thresholds]
    ( 50:133, 88:128,154:196)
    (  0:  0,  0:255,  0:255)
    (  0:  0,  0:255,  0:255)
    (  0:  0,  0:255,  0:255)
    (  0:  0,  0:255,  0:255)
    (  0:  0,  0:255,  0:255)
    */

    /*
    frontDistanceLookup.load("config/distance-front.cfg");

    for (int i = 0; i < 800; i += 100) {
        float v = frontDistanceLookup.getValue(i);

        std::cout << "Test " << i << ": " << v << std::endl;
    }
    */

	/*LookupTable testLookup;
    testLookup.addValue(0, 4.0f);
    testLookup.addValue(100, 3.0f);
    testLookup.addValue(150, 2.0f);

    double d1 = testLookup.getValue(0);
    double d2 = testLookup.getValue(100);
    double d3 = testLookup.getValue(50);
    double d4 = testLookup.getValue(25);
    double d5 = testLookup.getValue(75);
    double d6 = testLookup.getValue(125);
    double d7 = testLookup.getValue(200);

    std::cout << "Distance   0: " << d1 << " / 4.00" << std::endl;
    std::cout << "Distance 100: " << d2 << " / 3.00" << std::endl;
    std::cout << "Distance  50: " << d3 << " / 3.50" << std::endl;
    std::cout << "Distance  25: " << d4 << " / 3.75" << std::endl;
    std::cout << "Distance  75: " << d5 << " / 3.25" << std::endl;
    std::cout << "Distance 125: " << d6 << " / 2.50" << std::endl;
    std::cout << "Distance 200: " << d7 << " / 2.00" << std::endl;*/
}

Vision::~Vision() {
    if (blobber != NULL) {
        blobber->saveOptions(Config::blobberConfigFilename);

        delete blobber;

        blobber = NULL;
    }

    if (classificationFront != NULL) {
        delete classificationFront;
    }

	if (classificationRear != NULL) {
        delete classificationRear;
    }
}

void Vision::setFrame(unsigned char* frame, Dir dir) {
    blobber->processFrame((Blobber::Pixel*)frame);

	if (dir == Dir::DIR_FRONT) {
		lastFrameFront = frame;
	} else {
		lastFrameRear = frame;
	}
}

void Vision::process(Dir dir) {
    processBalls(dir);
	processGoals(dir);

	if (dir == Dir::DIR_FRONT) {
		updateObstructions();
		updateColorDistances();
	}
}

void Vision::processBalls(Dir dir) {
	ObjectList ballset;
	ObjectList* balls = dir == Dir::DIR_FRONT ? &frontBalls : &rearBalls;

    for (ObjectListIt it = balls->begin(); it != balls->end(); it++) {
        delete *it;
    }

    balls->clear();

    float distance;
    float angle;

    Blobber::Blob* blob = blobber->getBlobs("ball");

    while (blob != NULL) {
		if (blob->area < Config::ballBlobMinArea) {
			blob = blob->next;

			continue;
		}

		distance = getDistance(dir, blob->centerX, blob->y2);
        angle = getAngle(dir, blob->centerX, blob->y2);

		if (dir == Dir::DIR_REAR) {
			if (angle > 0.0f) {
				angle -= Math::PI;
			} else {
				angle += Math::PI;
			}
		}

		if (blob->x1 < 0) blob->x1 = 0;
		if (blob->x2 > width - 1) blob->x2 = width - 1;
		if (blob->y1 < 0) blob->y1 = 0;
		if (blob->y2 > height - 1) blob->y2 = height - 1;

		float width = blob->x2 - blob->x1;
		float height = blob->y2 - blob->y1;

        Object* ball = new Object(
            blob->x1 + width / 2,
            blob->y1 + height / 2,
            width,
            height,
            blob->area,
            distance,
            angle,
			0,
			dir == Dir::DIR_FRONT ? false : true
        );
		
        ballset.push_back(ball);

        blob = blob->next;
    }

	ObjectList individualBalls = Object::mergeOverlapping(ballset, Config::ballOverlapMargin);

	for (ObjectListItc it = individualBalls.begin(); it != individualBalls.end(); it++) {
		Object* ball = *it;

		if (isValidBall(ball, dir)) {
			ball->distance = getDistance(dir, ball->x, ball->y + ball->height / 2);
			ball->angle = getAngle(dir, ball->x, ball->y + ball->height / 2);
			balls->push_back(ball);
		}
	}
}

void Vision::processGoals(Dir dir) {
	ObjectList goalset;
	ObjectList* goals = dir == Dir::DIR_FRONT ? &frontGoals : &rearGoals;

    for (ObjectListIt it = goals->begin(); it != goals->end(); it++) {
        delete *it;
    }

    goals->clear();

    float distance;
    float angle;

    for (int i = 0; i < 2; i++) {
        Blobber::Blob* blob = blobber->getBlobs(i == 0 ? "yellow-goal" : "blue-goal");

        while (blob != NULL) {
			if (blob->area < Config::goalBlobMinArea) {
				blob = blob->next;

				continue;
			}

			// TODO Calculate from center lowest y pixel and recalculate after merge
			distance = getDistance(dir, blob->centerX, blob->y2);
            angle = getAngle(dir, blob->centerX, blob->y2);

			if (dir == Dir::DIR_REAR) {
				if (angle > 0.0f) {
					angle -= Math::PI;
				} else {
					angle += Math::PI;
				}
			}

			if (blob->x1 < 0) blob->x1 = 0;
			if (blob->x2 > width - 1) blob->x2 = width - 1;
			if (blob->y1 < 0) blob->y1 = 0;
			if (blob->y2 > height - 1) blob->y2 = height - 1;

			float width = blob->x2 - blob->x1;
			float height = blob->y2 - blob->y1;

			Object* goal = new Object(
				blob->x1 + width / 2,
				blob->y1 + height / 2,
				width,
				height,
				blob->area,
				distance,
				angle,
				i == 0 ? Side::YELLOW : Side::BLUE,
				dir == Dir::DIR_FRONT ? false : true
			);

			goal->processed = false;
			goalset.push_back(goal);

            blob = blob->next;
        }
    }

	ObjectList individualGoals = Object::mergeOverlapping(goalset, Config::goalOverlapMargin, true);

	for (ObjectListItc it = individualGoals.begin(); it != individualGoals.end(); it++) {
		Object* goal = *it;

		if (isValidGoal(goal, goal->type == 0 ? Side::YELLOW : Side::BLUE)) {
			// TODO Extend the goal downwards using extended color / limited ammount horizontal too

			goal->distance = getDistance(dir, goal->x, goal->y + goal->height / 2);
			goal->angle = getAngle(dir, goal->x, goal->y + goal->height / 2);
			goals->push_back(goal);
		}
	}
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

	if (goal->area < Config::goalMinArea) {
		//std::cout << "@ GOAL INVALID MIN AREA: " << goal->area << " VS " << Config::goalMinArea << std::endl;

		return false;
	} else if (goal->area > 10000) {
		return true;
	}

	if (goal->y - goal->height / 2 > Config::goalTopMaxY) {
		//std::cout << "@ GOAL NOT TOP ENOUGH: " << (goal->y - goal->height / 2) << " VS " << Config::goalTopMaxY << std::endl;

		return false;
	}

	/*if (undersideMetric < Config::goalMinUndersideMetric) {
		//std::cout << "@ GOAL INVALID UNDERSIDE: " << undersideMetric << " VS " << Config::goalMinUndersideMetric << std::endl;

		return false;
	}*/

    return true;
}

bool Vision::isValidBall(Object* ball, Dir dir) {
    if (ball->area < Config::ballMinArea) {
		//std::cout << "@ BALL AREA TOO SMALL: " << ball->area << " VS " << Config::ballMinArea << std::endl;

        return false;
    }

	float sizeRatio = (float)ball->width / (float)ball->height;

	if (sizeRatio > Config::maxBallSizeRatio || sizeRatio < 1.0f/Config::maxBallSizeRatio) {
		return false;
	}

	int ballRadius = Math::min(ball->width, ball->height) / 2;
	int senseRadius = Math::min(ballRadius * 1.35f * Math::max(ball->distance / 2.0f, 1.0f) + 10.0f, Config::maxBallSenseRadius);

	if (ball->y + ballRadius < Config::maxSurroundSenseY) {
		float surroundMetric = getSurroundMetric(
			ball->x,
			ball->y + ballRadius,
			senseRadius,
			validBallBgColors,
			"",
			1
		);

		//std::cout << "Surround: " << surroundMetric << std::endl;

		if (surroundMetric != -1.0f && surroundMetric < Config::validBallSurroundThreshold) {
			//std::cout << "@ BALL SURROUND FAIL: " << surroundMetric << " VS " << Config::validBallSurroundThreshold << std::endl;

			return false;
		}
	}

	if (ball->y < Config::cameraPathStartY) {
		PathMetric pathMetric = getPathMetric(
			Config::cameraPathStartX,
			Config::cameraPathStartY,
			ball->x,
			ball->y + ballRadius + senseRadius / 2 + 6,
			validBallPathColors
			//,"green"
		);

		//std::cout << "Ball path: " << pathMetric << std::endl;

		// @TODO Path must not contain green-white-black

		if (
			pathMetric.percentage < Config::validBallPathThreshold
			|| pathMetric.out
			//|| !pathMetric.validColorFound
			//|| pathMetric.invalidSpree > getBallMaxInvalidSpree(ball->y + ball->height / 2)
		) {
			//std::cout << "@ BALL PATH FAILS: " << pathMetric.percentage << " VS " << Config::validBallPathThreshold << ", OUT: " << (pathMetric.out ? "YES" : "NO") << std::endl;

			return false;
		}
	}

	if (isBallInGoal(ball, dir)) {
		//std::cout << "@ BALL IN GOAL FAILS" << std::endl;

		return false;
	}

    return true;
}

bool Vision::isBallInGoal(Object* ball, Dir dir) {
	if (ball->distance < 1.0f) {
		int ballRadius = Math::max(ball->width, ball->height) / 2;
		int senseRadius = Math::min(ballRadius * 1.35f * Math::max(ball->distance / 2.0f, 1.0f) + 10.0f, Config::maxBallSenseRadius);

		float surroundMetric = getSurroundMetric(
			ball->x,
			ball->y + ballRadius,
			senseRadius,
			goalColors,
			"",
			-1,
			true
		);

		if (surroundMetric != -1.0f && surroundMetric > Config::ballInGoalThreshold) {
			//std::cout << "@ BALL IN GOAL SURROUND: " << surroundMetric << " VS " << Config::ballInGoalThreshold << std::endl;

			return true;
		}
	}

	/*if (dir == Dir::DIR_FRONT) {
		for (ObjectListItc it = frontGoals.begin(); it != frontGoals.end(); it++) {
			if (ball->contains(*it)) {
				//std::cout << "@ BALL IN GOAL INTERSECTS FRONT" << std::endl;

				return true;
			}
		}
	} else if (dir == Dir::DIR_REAR) {
		for (ObjectListItc it = rearGoals.begin(); it != rearGoals.end(); it++) {
			if (ball->contains(*it)) {
				//std::cout << "@ BALL IN GOAL INTERSECTS REAR" << std::endl;

				return true;
			}
		}
	}*/

	return false;
}

int Vision::getBallMaxInvalidSpree(int y) {
	return y / 20.0f; // @TODO Something reasonable..
}

int Vision::getGoalMaxInvalidSpree(int y) {
	return y / 20.0f;
}

/*void Vision::filterMap(unsigned int* map) {

}*/

float Vision::getDistance(Dir dir, int x, int y) {
	//float yCorrection = 0.000095 * Math::pow(x, 2) - 0.0536 * x + 7;
	//std::cout << "! Y-correction: " << yCorrection << " at x: " << x << std::endl;

	int realX = x;
	int realY = y;

	Util::correctCameraPoint(realX, realY);

	float yCorrection = 0;
	float distance;

    if (dir == DIR_FRONT) {
		distance = frontDistanceLookup.getValue(realY - yCorrection);
    } else {
        distance = rearDistanceLookup.getValue(realY - yCorrection);
    }

	return Math::max(distance + Config::distanceCorrection, 0.01f);
}

float Vision::getHorizontalDistance(Dir dir, int x, int y) {
	/*float measurementPixels = 150;
	float distance = getDistance(dir, x, y);
	float distanceLookup = dir == DIR_FRONT ? frontAngleLookup.getValue(distance) : frontAngleLookup.getValue(distance);
	float metersPerPixel = distanceLookup / measurementPixels;
	float centerOffset = (float)(x - (Config::cameraWidth / 2));
	float horizontalDistance = centerOffset * metersPerPixel;*/

	/*std::cout << "! Solve horizontal distance for " << x << "x" << y << std::endl;
	std::cout << " > Dir " << dir << std::endl;
	std::cout << " > Distance " << distance << std::endl;
	std::cout << " > Horizontal " << measurementPixels << " pixels meters: " << distanceLookup << std::endl;
	std::cout << " > Test 2m lookup: " << frontAngleLookup.getValue(2.0f) << std::endl;
	std::cout << " > Meters for pixel: " << metersPerPixel << std::endl;
	std::cout << " > Center offset: " << centerOffset << std::endl;
	std::cout << " > Horizontal distance: " << horizontalDistance << std::endl;*/

	//return horizontalDistance;

	int realX = x;
	int realY = y;

	Util::correctCameraPoint(realX, realY);

	float distance = getDistance(dir, realX, realY);
	float centerOffset = (float)(realX - (Config::cameraWidth / 2));
	float localAngle = distance / 686.0f;

	return Math::tan(localAngle) * (realY + 0.062);
}

float Vision::getAngle(Dir dir, int x, int y) {
	int realX = x;
	int realY = y;

	Util::correctCameraPoint(realX, realY);

    /*float centerOffset = (float)(x - (Config::cameraWidth / 2));
    float distance = getDistance(dir, x, y);
    float pixelsPerCm = dir == DIR_FRONT ? frontAngleLookup.getValue(distance) : rearAngleLookup.getValue(distance);
    float horizontalDistance = (double)centerOffset / pixelsPerCm;
	//return Math::tan(horizontalDistance / distance) * 180.0 / Math::PI;
	*/

	/*float distance = getDistance(dir, x, y);
	float horizontalDistance = getHorizontalDistance(dir, x, y);
	return Math::tan(horizontalDistance / distance);*/

	//float distance = getDistance(dir, x, y);

	// last working
	float centerOffset = (float)(realX - (Config::cameraWidth / 2.0f)),
		angle = Math::degToRad(centerOffset / 11.5f);

	if (dir == Dir::DIR_REAR) {
		if (angle < 0.0f) {
			angle += Math::PI;
		} else {
			angle -= Math::PI;
		}
	}

	return angle;

	/*float distance = getDistance(dir, x, y);
	float horizontalDistance = getHorizontalDistance(dir, x, y);

	return Math::atan(horizontalDistance / distance);*/
}

Blobber::Color* Vision::getColorAt(int x, int y) {
    return blobber->getColorAt(x, y);
}

float Vision::getSurroundMetric(int x, int y, float radius, std::vector<std::string> validColors, std::string requiredColor, int side, bool allowNone) {
	int matches = 0;
	int misses = 0;
    int points = radius * 2;
    bool requiredColorFound = false;
    bool debug = img.data != NULL;

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

        int senseX = x + radius * cos(t);
        int senseY = y + radius * sin(t);

		if (
			senseX < 0
			|| senseX > width - 1
			|| senseY < 0
			|| senseY > Config::maxSurroundSenseY
		) {
			continue;
		}

        Blobber::Color* color = getColorAt(senseX, senseY);

        if (color != NULL) {
            if (find(validColors.begin(), validColors.end(), std::string(color->name)) != validColors.end()) {
                matches++;

                if (debug) {
                    img.drawMarker(senseX, senseY, 0, 200, 0);
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
					img.drawMarker(senseX, senseY, 200, 0, 0);
				}
			} else {
				if (debug) {
					img.drawMarker(senseX, senseY, 128, 128, 128);
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
	x1 = Math::limit(x1, 0, Config::cameraWidth);
	x2 = Math::limit(x2, 0, Config::cameraWidth);
	y1 = Math::limit(y1, 0, Config::cameraHeight);
	y2 = Math::limit(y2, 0, Config::cameraHeight);

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
    const int maxSensePoints = 255;
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
    bool debug = img.data != NULL;
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

    for (int i = start; (originalX1 < originalX2 ? i < senseCounter : i >= 0); i += step) {
        x = senseX[i];
        y = senseY[i];

        Blobber::Color* color = getColorAt(x, y);

        if (color != NULL) {
			if (firstColor == "") {
				firstColor = std::string(color->name);
			}

			lastColor = std::string(color->name);

			if (strcmp(color->name, "black") == 0) {
				blacksInRow++;

				if (blacksInRow > maxBlacksInRow) {
					tooManyBlacksInRow = true;

					break;
				}
			} else {
				blacksInRow = 0;
			}

			if (strcmp(color->name, "green") == 0) {
				if (!sawGreen) {
					sawGreen = true;

					if (debug) {
						img.drawMarker(x, y, 0, 128, 0);
					}
				} else if ((sawWhite || firstColor == "white") && previousBlack >= 2) {
					crossingGreenWhiteBlackGreen = true;

					if (debug) {
						img.drawMarker(x, y, 128, 0, 0);
					}

					break;
				}
			} else if (!sawWhite /*&& sawGreen*/ && strcmp(color->name, "white") == 0) {
				sawWhite = true;

				if (debug) {
					img.drawMarker(x, y, 128, 128, 128);

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
                    img.drawMarker(x, y, 0, 200, 0);
                }
            } else {
				invalidSpree++;

				if (debug) {
                    img.drawMarker(x, y, 200, 0, 0);
                }
			}

            if (requiredColor != "" && color->name == requiredColor) {
                requiredColorFound = true;
            }
        } else {
            if (debug) {
                img.drawMarker(x, y, 200, 0, 0);
            }

			invalidSpree++;
        }
    }

	if (
		(lastColor == "black" && sawWhite)
		|| (sawWhiteBeforeBlack && lastColor == "green")
	) {
		//std::cout << "@ OUT LATE" << std::endl;

		crossingGreenWhiteBlackGreen = true;
	}

	//std::cout << "@ MOST BLACKS IN ROW: " << mostBlacksInRow << std::endl;

	if (senseCounter < 20) {
		return PathMetric(1.0f, 0, true, false);
	}

	float percentage = (float)matches / (float)senseCounter;
	bool validColorFound = requiredColor == "" || requiredColorFound;

	return PathMetric(percentage, longestInvalidSpree, validColorFound, crossingGreenWhiteBlackGreen || tooManyBlacksInRow);
}

float Vision::getColorDistance(std::string colorName, int x1, int y1, int x2, int y2) {
	x1 = Math::limit(x1, 0, Config::cameraWidth);
	x2 = Math::limit(x2, 0, Config::cameraWidth);
	y1 = Math::limit(y1, 0, Config::cameraHeight);
	y2 = Math::limit(y2, 0, Config::cameraHeight);

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
    const int maxSensePoints = 255;
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
    bool debug = img.data != NULL;
	int start = originalX1 < originalX2 ? 0 : senseCounter - 1;
	int step = originalX1 < originalX2 ? 1 : -1;

    for (int i = start; (originalX1 < originalX2 ? i < senseCounter : i >= 0); i += step) {
        x = senseX[i];
        y = senseY[i];

        Blobber::Color* color = getColorAt(x, y);

		if (color != NULL) {
			
			if (strcmp(color->name, colorName.c_str()) == 0) {
				if (debug) {
					img.drawMarker(x, y, 0, 200, 0);
				}

				return getDistance(Dir::DIR_FRONT, x, y);
			} else {
				if (debug) {
					img.drawMarker(x, y, 200, 0, 0);
				}
			}
		} else {
			if (debug) {
                img.drawMarker(x, y, 64, 64, 64);
            }
		}
	}

	return -1.0f;
}

float Vision::getColorDistance(std::string colorName) {
	float distanceA = getColorDistance(
		colorName,
		Config::cameraPathStartX, Config::cameraPathStartY,
		0, 0
	);
	float distanceB = getColorDistance(
		colorName,
		Config::cameraPathStartX, Config::cameraPathStartY,
		Config::cameraWidth / 2, 0
	);
	float distanceC = getColorDistance(
		colorName,
		Config::cameraPathStartX, Config::cameraPathStartY,
		Config::cameraWidth, 0
	);

	return Math::min(Math::min(distanceA, distanceB), distanceC);
}

bool Vision::isBallInWay(int goalY) {
	int startY = Config::cameraHeight - 100;
	int halfWidth = Config::cameraWidth / 2;
	const ObjectList& frontBalls = getFrontBalls();
	Object* ball;
	float checkWidth;

	for (ObjectListItc it = frontBalls.begin(); it != frontBalls.end(); it++) {
		ball = *it;
		checkWidth = ball->width * 2.0f;
		
		if (
			ball->x - checkWidth < halfWidth && ball->x + checkWidth > halfWidth
			&& ball->y - ball->height < startY && ball->y  + ball->height > goalY
		) {
			return true;
		}
	}

	return false;

	/*int step = 6;

	for (int y = Config::cameraHeight - 50; y >= goalY; y -= step) {

	}*/
}

float Vision::getBlockMetric(int x1, int y1, int blockWidth, int blockHeight, std::vector<std::string> validColors, int step) {
	bool debug = img.data != NULL;
	int matches = 0;
	int misses = 0;

	for (int x = Math::max(x1, 0); x < Math::min(x1 + blockWidth, width); x += step) {
		for (int y = Math::max(y1, 0); y < Math::min(y1 + blockHeight, height); y += step) {
			Blobber::Color* color = getColorAt(x, y);

			if (color != NULL) {
				if (find(validColors.begin(), validColors.end(), std::string(color->name)) != validColors.end()) {
					matches++;

					if (debug) {
						img.drawMarker(x, y, 0, 200, 0);
					}
				} else {
					misses++;

					if (debug) {
						img.drawMarker(x, y, 200, 0, 0);
					}
				}
			} else {
				misses++;

				if (debug) {
					img.drawMarker(x, y, 200, 0, 0);
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
	bool debug = img.data != NULL;
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

	for (int x = Math::max(x1, 0); x < Math::min(x1 + blockWidth, width); x += xStep) {
		if (x > width - 1) {
			break;
		}

		stepsBelow = 0;

		for (int y = y1; y < Math::min(y1 + blockHeight * 4, height); y += yStep) {
			if (y > height - 1) {
				break;
			}

			color = getColorAt(x, y);

			if (color == NULL || (strcmp(color->name, targetColorName) != 0 && strcmp(color->name, targetColorName2) != 0)) {
				if (debug) {
					img.drawMarker(x, y, 64, 64, 64);
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
				img.drawMarker(x, y, 255, 255, 255);
			}

			//std::cout << "! DOWN FROM " << (y + yStep) << " to " << y + blockHeight << std::endl;

			for (int senseY = y + yStep; senseY < Math::min(y + blockHeight * 4, height); senseY += yStep) {
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
						img.drawMarker(x, senseY, 0, 0, 0);
					}
				} else {
					if (debug) {
						img.drawMarker(x, senseY, 255, 255, 255);
					}

					sawGreenOrBlack = false;
					bool retryTarget = false;
					int runMatches = 0;
					int runMisses = 0;

					for (int gapY = senseY + gapStep; gapY < Math::min(senseY + senseSteps * yStep, height); gapY += gapStep) {
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
									img.drawMarker(x, gapY, 0, 200, 0, true);
								}
							} else {
								runMisses++;

								if (debug) {
									img.drawMarker(x, gapY, 200, 0, 0, true);
								}
							}
						} else {
							// allow one invalid color after white/black
							if (lastColorName != "black" && lastColorName != "white") {
								img.drawMarker(x, gapY, 100, 0, 0, true);

								runMisses++;
							} else {
								lastColorName = "";

								if (debug) {
									img.drawMarker(x, gapY, 255, 255, 0, true);
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
			img.drawBox(minValidX, minValidY, maxValidX - minValidX, maxValidY - minValidY, 255, 0, 0);
			//img.drawBox(maxValidX, 0, Config::cameraWidth - maxValidX, Math::min(maxValidY + 60, Config::cameraHeight - 1), 255, 255, 255);
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
						img.drawMarker(x, y, 255, 255, 255);
					}
				} else {
					gap++;

					if (debug) {
						img.drawMarker(x, y, 200, 0, 0);
					}
				}

				if (gap > maxGap) {
					if (debug) {
						img.drawMarker(x, y, 100, 0, 0);
					}

					break;
				}
			}
		}
	}*/

	if (y1 > Config::whiteBlackMinY && !sawWhite && !sawBlack) {
		return false;
	}

	if (matches < Config::validGoalMinMatches) {
		return false;
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
		Config::cameraWidth / 2 - Config::robotInWayWidth,
		Config::robotInWayY,
		Config::robotInWayWidth,
		Config::robotInWayHeight,
		viewObstructedValidColors,
		20
	);

	float rightMetric = getBlockMetric(
		Config::cameraWidth / 2,
		Config::robotInWayY,
		Config::robotInWayWidth,
		Config::robotInWayHeight,
		viewObstructedValidColors,
		20
	);

	viewObstructed = false;
	robotInWay = 0;

	if (leftMetric < Config::viewObstructedThreshold && rightMetric < Config::viewObstructedThreshold) {
		viewObstructed = true;
	} else if (leftMetric < Config::viewObstructedThreshold) {
		robotInWay = -1;
	} else if (rightMetric < Config::viewObstructedThreshold) {
		robotInWay = 1;
	}
}

void Vision::updateColorDistances() {
	blackDistance = getColorDistance("black");
}

unsigned char* Vision::getClassification(Dir dir) {
	if (dir == Dir::DIR_FRONT) {
		if (classificationFront == NULL) {
			std::cout << "! Creating front classification buffer.. ";
			classificationFront = new unsigned char[width * height * 3];
			std::cout << "done!" << std::endl;
		}

		return classificationFront;
	} else {
		if (classificationRear == NULL) {
			std::cout << "! Creating rear classification buffer.. ";
			classificationRear = new unsigned char[width * height * 3];
			std::cout << "done!" << std::endl;
		}

		return classificationRear;
	}
}

ImageBuffer* Vision::classify(Dir dir) {
	unsigned char* lastFrame = getLastFrame(dir);
	unsigned char* classification = getClassification(dir);

    if (lastFrame == NULL) {
        return NULL;
    }

    blobber->classify((Blobber::Rgb*)classification, (Blobber::Pixel*)lastFrame);

    img.width = width;
    img.height = height;
    img.data = classification;

    return &img;
}

void Vision::setImage(unsigned char* image, int width, int height) {
	img.width = width;
    img.height = height;
    img.data = image;
}

Object* Vision::getClosestBall(bool frontOnly) {
	const ObjectList& frontBalls = getFrontBalls();
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

	return closestBall;

	/*if (closestBall != NULL) {
		lastClosestBall.copyFrom(closestBall);

		return closestBall;
	} else if (
		lastClosestBall.width > 0
		&& Util::duration(lastClosestBall.lastSeenTime) < Config::fakeBallLifetime
	) {
		//std::cout << "@ RETURNING FAKE CLOSEST BALL " << Util::duration(lastClosestBall.lastSeenTime) << std::endl;

		return &lastClosestBall;
	} else {
		return NULL;
	}*/
}

Object* Vision::getLargestGoal(Side side, bool frontOnly) {
	const ObjectList& frontGoals = getFrontGoals();
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
		/*int minX, minY, maxX, maxY;
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
		}*/

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
	}
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

Object* Vision::getFurthestGoal(bool frontOnly) {
	Object* largestYellow = getLargestGoal(Side::YELLOW, frontOnly);
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
	}
}
