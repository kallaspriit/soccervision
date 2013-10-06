#ifndef BALLLOCALIZER_H
#define BALLLOCALIZER_H

#include "Maths.h"

class BallLocalizer {

public:
	class Ball {

    public:
        Ball(float x, float y, float distance, float angle, float dt);
        void updateVisible(float x, float y, float distance, float angle, float dt);
        void updateInvisible(float dt);
        void markForRemoval(double afterSeconds);
        bool shouldBeRemoved();

        int id;
        double createdTime;
        double updatedTime;
        double removeTime;
        float x;
        float y;
        float velocityX;
        float velocityY;
        float distance;
        float angle;
        float elasticity;
        float radius;
        bool visible;
    private:
        static int instances;

        void applyDrag(float dt);

	};

	typedef std::vector<Ball*> BallList;
	typedef std::vector<Ball*>::iterator BallListIt;

    BallLocalizer();
    ~BallLocalizer();

    void update(Math::Position robotPosition, const BallList& visibleBalls, const Math::Polygon& cameraFOV, float dt);
    Ball* getBallAround(float x, float y);
    void purge(const BallList& visibleBalls, const Math::Polygon& cameraFOV);
    bool isValid(Ball* ball, const BallList& visibleBalls, const Math::Polygon& cameraFOV);

private:
    BallList balls;

};

#endif // BALLLOCALIZER_H
