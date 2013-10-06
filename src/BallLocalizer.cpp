#include "BallLocalizer.h"
#include "Config.h"
#include "Util.h"

#include <vector>
#include <algorithm>

BallLocalizer::BallLocalizer() {

}

BallLocalizer::~BallLocalizer() {

}

int BallLocalizer::Ball::instances = 0;

BallLocalizer::Ball::Ball(float x, float y, float distance, float angle, float dt) {
    id = instances++;
    createdTime = Util::millitime();
    updatedTime = createdTime;
    removeTime = -1,
    visible = true;

    updateVisible(x, y, distance, angle, dt);
}

void BallLocalizer::Ball::updateVisible(float newX, float newY, float newDistance, float newAngle, float dt) {
    double currentTime = Util::millitime();
    double timeSinceLastUpdate = currentTime - updatedTime;

    if (timeSinceLastUpdate <= Config::velocityUpdateMaxTime) {
        velocityX = (newX - x) / dt;
        velocityY = (newY - y) / dt;
    } else {
        applyDrag(dt);
    }

    x = newX;
    y = newY;
    distance = newDistance;
    angle = newAngle;
    updatedTime = currentTime;
    removeTime = -1;
    visible = true;
}

void BallLocalizer::Ball::updateInvisible(float dt) {
    x += velocityX * dt;
    y += velocityY * dt;

    applyDrag(dt);

    visible = false;
}

void BallLocalizer::Ball::markForRemoval(double afterSeconds) {
    if (removeTime == -1) {
        return;
    }

    removeTime = Util::millitime() + afterSeconds;
}

bool BallLocalizer::Ball::shouldBeRemoved() {
    return removeTime != -1 && removeTime < Util::millitime();
}

void BallLocalizer::Ball::applyDrag(float dt) {
    float xSign = velocityX > 0 ? 1.0f : -1.0f;
    float ySign = velocityY > 0 ? 1.0f : -1.0f;
    float stepDrag = Config::rollingDrag * dt;

    if (Math::abs(velocityX) > stepDrag) {
        velocityX -= stepDrag * xSign;
    } else {
        velocityX = 0.0f;
    }

    if (Math::abs(velocityY) > stepDrag) {
        velocityY -= stepDrag * ySign;
    } else {
        velocityY = 0.0f;
    }
}

void BallLocalizer::update(Math::Position robotPosition, const BallList& visibleBalls, const Math::Polygon& cameraFOV, float dt) {
    Ball* closestBall;
    std::vector<int> handledBalls;
    float globalAngle;

    for (unsigned int i = 0; i < visibleBalls.size(); i++) {
        globalAngle = Math::floatModulus(robotPosition.orientation + visibleBalls[i]->angle, Math::TWO_PI);

        visibleBalls[i]->x = robotPosition.x + Math::cos(globalAngle) * visibleBalls[i]->distance;
        visibleBalls[i]->y = robotPosition.y + Math::sin(globalAngle) * visibleBalls[i]->distance;

        closestBall = getBallAround(visibleBalls[i]->x, visibleBalls[i]->y);

        if (closestBall != NULL) {
            closestBall->updateVisible(visibleBalls[i]->x, visibleBalls[i]->y, visibleBalls[i]->distance, visibleBalls[i]->angle, dt);

            handledBalls.push_back(closestBall->id);
        } else {
            Ball* newBall = new Ball(visibleBalls[i]->x, visibleBalls[i]->y, visibleBalls[i]->distance, visibleBalls[i]->angle, dt);

            balls.push_back(newBall);

            handledBalls.push_back(newBall->id);
        }
    }

    for (unsigned int i = 0; i < balls.size(); i++) {
        if (std::find(handledBalls.begin(), handledBalls.end(), balls[i]->id) != handledBalls.end()) {
            continue;
        }

        balls[i]->updateInvisible(dt);
    }

    purge(visibleBalls, cameraFOV);
}

BallLocalizer::Ball* BallLocalizer::getBallAround(float x, float y) {
    float distance;
    float minDistance = -1;
    Ball* ball;
    Ball* closestBall = NULL;

    for (unsigned int i = 0; i < balls.size(); i++) {
        ball = balls[i];

        distance = Math::distanceBetween(ball->x, ball->y, x, y);

        if (
            distance <= Config::objectIdentityDistanceThreshold
            && (
                minDistance == -1
                || distance < minDistance
            )
        ) {
            minDistance = distance;
            closestBall = ball;
        }
    }

    return closestBall;
}

void BallLocalizer::purge(const BallList& visibleBalls, const Math::Polygon& cameraFOV) {
    BallList remainingBalls;
    Ball* ball;

    for (unsigned int i = 0; i < balls.size(); i++) {
        ball = balls[i];

        if (!ball->shouldBeRemoved()) {
            remainingBalls.push_back(ball);

            if (!isValid(ball, visibleBalls, cameraFOV)) {
                ball->markForRemoval(Config::objectMarkForRemovalThreshold);
            }
        } else {
            delete ball;
            ball = NULL;
        }
    }

    balls = remainingBalls;
}

bool BallLocalizer::isValid(Ball* ball, const BallList& visibleBalls, const Math::Polygon& cameraFOV) {
    double currentTime = Util::millitime();

    if (currentTime - ball->updatedTime > Config::objectPurgeLifetime) {
        return false;
    }

    Math::Vector velocity(ball->velocityX, ball->velocityY);

    if (velocity.getLength() > Config::objectMaxVelocity) {
        return false;
    }

    // @TODO Remove if in either goal or out of bounds..

    if (cameraFOV.containsPoint(ball->x, ball->y)) {
        bool ballNear = false;
        float distance;

        for (unsigned int i = 0; i < visibleBalls.size(); i++) {
            distance = Math::distanceBetween(ball->x, ball->y, visibleBalls[i]->x, visibleBalls[i]->y);

            if (distance <= Config::objectFovCloseEnough) {
                ballNear = true;

                break;
            }
        }

        if (!ballNear)  {
            return false;
        }
    }

    return true;
}
