#ifndef VISION_H
#define VISION_H

#include "Blobber.h"
#include "ImageBuffer.h"
#include "Object.h"
#include "LookupTable.h"
#include "Config.h"

#include <string>
#include <vector>

class Vision/* : public Blobber::MapFilter*/ {
    public:
        enum Dir { DIR_FRONT, DIR_REAR };
		struct PathMetric {
			PathMetric(float percentage, int invalidSpree, bool validColorFound, bool out) : percentage(percentage), invalidSpree(invalidSpree), validColorFound(validColorFound), out(out) {}

			float percentage;
			int invalidSpree;
			bool validColorFound;
			bool out;
		};

        Vision(int width, int height);
        ~Vision();

        void setFrame(unsigned char* frame, Dir dir);
        void process(Dir dir);
		void setImage(unsigned char* image, int width = Config::cameraWidth, int height = Config::cameraHeight);
        //void filterMap(unsigned int* map);
        unsigned int* getColorMap() { return blobber->getMap(); }
        ImageBuffer* classify(Dir dir);
		unsigned char* getLastFrame(Dir dir) { return dir == Dir::DIR_FRONT ? lastFrameFront : lastFrameRear; }
		unsigned char* getClassification(Dir dir);
        Blobber* getBlobber() { return blobber; }

        Blobber::Color* getColorAt(int x, int y);
        float getSurroundMetric(int x, int y, float radius, std::vector<std::string> validColors, std::string requiredColor = "", int side = 0, bool allowNone = false);
        PathMetric getPathMetric(int x1, int y1, int x2, int y2, std::vector<std::string> validColors, std::string requiredColor = "");
		float getBlockMetric(int x, int y, int width, int height, std::vector<std::string> validColors, int step = 6);
		float getUndersideMetric(int x, int y, float distance, int width, int height, std::string targetColor, std::string targetColor2, std::vector<std::string> validColors, bool expand = true);
		float getUndersideMetric(int x, int y, float distance, int width, int height, std::string targetColor, std::string targetColor2, std::vector<std::string> validColors, int& minValidX, int& minValidY, int& maxValidX, int& maxValidY, bool expand = true);
		float getColorDistance(std::string colorName, int x1, int y1, int x2, int y2);
		float getColorDistance(std::string colorName);
		bool isViewObstructed() { return viewObstructed; }
		int getRobotInWay() { return robotInWay; }
		bool isBallInWay(int goalY);
		float getBlackDistance() { return blackDistance; }

        const ObjectList& getFrontBalls() const { return frontBalls; }
        const ObjectList& getFrontGoals() const { return frontGoals; }
		const ObjectList& getRearBalls() const { return rearBalls; }
        const ObjectList& getRearGoals() const { return rearGoals; }

		Object* getClosestBall(bool frontOnly = false);
		Object* getLargestGoal(Side side, bool frontOnly = false);
		Object* getFurthestGoal(bool frontOnly = false);

        float getDistance(Dir dir, int x, int y);
		float getHorizontalDistance(Dir dir, int x, int y);
        float getAngle(Dir dir, int x, int y);
		static int getBallMaxInvalidSpree(int y);
		static int getGoalMaxInvalidSpree(int y);

    private:
		void processBalls(Dir dir);
        void processGoals(Dir dir);

		Object* Vision::mergeGoals(Object* goal1, Object* goal2);
        bool isValidBall(Object* ball, Dir dir);
        bool isValidGoal(Object* goal, Side side);
		bool isBallInGoal(Object* ball, Dir dir);

		void updateObstructions();
		void updateColorDistances();

        Blobber* blobber;
        ImageBuffer img;
        ObjectList frontBalls;
        ObjectList frontGoals;
		ObjectList rearBalls;
        ObjectList rearGoals;
        LookupTable frontDistanceLookup;
        LookupTable rearDistanceLookup;
        LookupTable frontAngleLookup;
        LookupTable rearAngleLookup;
        std::vector<std::string> validBallBgColors;
        std::vector<std::string> validBallPathColors;
        std::vector<std::string> validGoalPathColors;
        std::vector<std::string> viewObstructedValidColors;
        std::vector<std::string> goalColors;
		Object lastClosestBall;
		Object lastLargestGoal;
		Object lastFurthestGoal;
        int width;
        int height;
        unsigned char* lastFrameFront;
        unsigned char* lastFrameRear;
        unsigned char* classificationFront;
        unsigned char* classificationRear;
		bool viewObstructed;
		int robotInWay;
		float blackDistance;
};

#endif // VISION_H
