#ifndef TESTCONTROLLER_H
#define TESTCONTROLLER_H

#include "BaseAI.h"
#include "Vision.h"
#include "DebouncedButton.h"
#include "Config.h"

class TestController : public BaseAI {

public:
	class State : public BaseAI::State {

	public:
		State(TestController* ai) : BaseAI::State(ai), ai(ai) {}

	protected:
		TestController* ai;

	};

	class IdleState : public State {

	public:
		IdleState(TestController* ai) : State(ai) {}
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration);

	};

	class WatchBallState : public State {

	public:
		WatchBallState(TestController* ai) : State(ai) {}
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration);

	};

	class WatchGoalState : public State {

	public:
		WatchGoalState(TestController* ai) : State(ai) {}
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration);

	};

	class SpinAroundDribblerState : public State {

	public:
		SpinAroundDribblerState(TestController* ai) : State(ai) {}
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration);

	};

	class DriveToState : public State {

	public:
		DriveToState(TestController* ai) : State(ai), x(Config::fieldWidth / 2.0f), y(Config::fieldHeight / 2.0f), orientation(0.0f) {}
		void onEnter(Robot* robot);
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration);

		float x;
		float y;
		float orientation;

	};

	
	TestController(Robot* robot, Communication* com);

    bool handleCommand(const Command& cmd);
	void handleTargetVectorCommand(const Command& cmd);
	void handleToggleGoCommand();
	void handleResetCommand();
	void handleDriveToCommand(const Command& cmd);
    void step(float dt, Vision::Results* visionResults);
	std::string getJSON();

private:
	void setupStates();
	void updateGoalDistances(Vision::Results* visionResults);

	DebouncedButton toggleGoBtn;
	DebouncedButton resetBtn;
	bool running;
	float manualSpeedX;
	float manualSpeedY;
	float manualOmega;
	float blueGoalDistance;
	float yellowGoalDistance;

};

#endif // TESTCONTROLLER_H
