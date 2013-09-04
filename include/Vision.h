#ifndef VISION_H
#define VISION_H

#include "Blobber.h"
#include "ImageBuffer.h"
#include "Object.h"
#include "LookupTable.h"
#include "Config.h"
#include "VisionResults.h"

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

		enum Obstruction {
			NONE = 0,
			LEFT = 1,
			RIGHT = 2,
			BOTH = 3,
		};

        Vision(Blobber* blobber, int width, int height);
        ~Vision();

		// make sure to destroy it!
		void setDebugImage(unsigned char* image, int width, int height);
        VisionResults* process(unsigned char* frame, Dir dir);
        Blobber::Color* getColorAt(int x, int y);
		bool isViewObstructed() { return obstructionSide != Obstruction::NONE; }
		Obstruction getObstruction() { return obstructionSide; }
		bool isBallInWay(ObjectList balls, int goalY);
		float getBlackDistance() { return blackDistance; }
        float getDistance(Dir dir, int x, int y);
		//float getHorizontalDistance(Dir dir, int x, int y);
        float getAngle(Dir dir, int x, int y);

    private:
		ObjectList processBalls(Dir dir);
        ObjectList processGoals(Dir dir);

		float getSurroundMetric(int x, int y, float radius, std::vector<std::string> validColors, std::string requiredColor = "", int side = 0, bool allowNone = false);
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
		int getBallSenseRadius(int ballRadius, int distance);
		/*int getBallMaxInvalidSpree(int y);
		int getGoalMaxInvalidSpree(int y);*/

		void updateObstructions();
		void updateColorDistances();

		unsigned char* frame;
		ImageBuffer img;
        Blobber* blobber;
        LookupTable frontDistanceLookup;
        LookupTable rearDistanceLookup;
        LookupTable frontAngleLookup;
        LookupTable rearAngleLookup;
        std::vector<std::string> validBallBgColors;
        std::vector<std::string> validBallPathColors;
        std::vector<std::string> validGoalPathColors;
        std::vector<std::string> viewObstructedValidColors;
        std::vector<std::string> goalColors;
        int width;
        int height;
		Obstruction obstructionSide;
		float blackDistance;
};

#endif // VISION_H
