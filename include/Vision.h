#ifndef VISION_H
#define VISION_H

#include "Blobber.h"
#include "Canvas.h"
#include "Object.h"
#include "LookupTable.h"
#include "Config.h"
#include "Maths.h"
#include "CameraTranslator.h"

#include <string>
#include <vector>

class Vision {

public:
	struct PathMetric {
		PathMetric(float percentage, int invalidSpree, bool validColorFound, bool out) : percentage(percentage), invalidSpree(invalidSpree), validColorFound(validColorFound), out(out) {}

		float percentage;
		int invalidSpree;
		bool validColorFound;
		bool out;
	};

	struct Result {
		ObjectList balls;
		ObjectList goals;
		Obstruction obstructionSide;
		float whiteDistance;
		float blackDistance;
	};

	class Results {
		public:
			Results() : front(NULL), rear(NULL) {}
			Object* getClosestBall(Dir dir = Dir::ANY, bool nextClosest = false);
			Object* getNextClosestBall(Dir dir = Dir::ANY);
			Object* getLargestGoal(Side side, Dir dir = Dir::ANY);
			Object* getFurthestGoal(Dir dir = Dir::ANY);
			bool isBallInWay(ObjectList balls, int goalY);

			Vision::Result* front;
			Vision::Result* rear;
	};

	struct Distance {
		Distance(float x = 0.0f, float y = 0.0f, float straight = 0.0f, float angle = 0.0f) : x(x), y(y), straight(straight), angle(angle) {}

		float x;
		float y;
		float straight;
		float angle;
	};

    Vision(Blobber* blobber, CameraTranslator* cameraTranslator, Dir dir, int width, int height);
    ~Vision();

	void setDebugImage(unsigned char* image, int width, int height);
    Result* process();
    Blobber::Color* getColorAt(int x, int y);
	CameraTranslator* getCameraTranslator() { return cameraTranslator; }
	bool isViewObstructed() { return obstructionSide != Obstruction::NONE; }
	Obstruction getObstruction() { return obstructionSide; }
	Dir getDir() { return dir; }
    Distance getDistance(int x, int y);
	//float getHorizontalDistance(Dir dir, int x, int y);
    float getAngle(int x, int y);
	//int getPixelRowAt(float distance);
	CameraTranslator::CameraPosition getPixelAt(float distanceX, float distanceY);
	Math::Point getScreenCoords(float distanceX, float distanceY);

private:
	ObjectList processBalls(Dir dir);
    ObjectList processGoals(Dir dir);
	float getSurroundMetric(int x, int y, int radius, std::vector<std::string> validColors, std::string requiredColor = "", int side = 0, bool allowNone = false);
    PathMetric getPathMetric(int x1, int y1, int x2, int y2, std::vector<std::string> validColors, std::string requiredColor = "");
	float getBlockMetric(int x, int y, int width, int height, std::vector<std::string> validColors, int step = 6);
	float getUndersideMetric(int x, int y, float distance, int width, int height, std::string targetColor, std::string targetColor2, std::vector<std::string> validColors, bool expand = true);
	float getUndersideMetric(int x, int y, float distance, int width, int height, std::string targetColor, std::string targetColor2, std::vector<std::string> validColors, int& minValidX, int& minValidY, int& maxValidX, int& maxValidY, bool expand = true);
	float getColorDistance(std::string colorName, int x1, int y1, int x2, int y2);
	float getColorDistance(std::string colorName);
	Object* Vision::mergeGoals(Object* goal1, Object* goal2);
    bool isValidBall(Object* ball, Dir dir);
    bool isValidGoal(Object* goal, Side side);
	bool isBallInGoal(Object* ball, Dir dir);
	int getBallRadius(int width, int height);
	int getBallSenseRadius(int ballRadius, float distance);
	int getPixelsBelow(int x, int y, std::vector<std::string> validColors, int allowedWrongPixels = 3);
	/*int getBallMaxInvalidSpree(int y);
	int getGoalMaxInvalidSpree(int y);*/
	void updateObstructions();
	void updateColorDistances();

	Dir dir;
	Canvas canvas;
    Blobber* blobber;
	CameraTranslator* cameraTranslator;
    std::vector<std::string> validBallBgColors;
    std::vector<std::string> validBallPathColors;
    std::vector<std::string> validGoalPathColors;
    std::vector<std::string> validColorsBelowBall;
    std::vector<std::string> viewObstructedValidColors;
    std::vector<std::string> goalColors;
    int width;
    int height;
	Obstruction obstructionSide;
	float whiteDistance;
	float blackDistance;

};

#endif // VISION_H
