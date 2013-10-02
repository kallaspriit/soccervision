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
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration) {}

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

	
	TestController(Robot* robot, Communication* com);

    bool handleCommand(const Command& cmd);
	void handleTargetVectorCommand(const Command& cmd);
	void handleToggleGoCommand();
	void handleResetCommand();
    void step(float dt, Vision::Results* visionResults);
	std::string getJSON();

private:
	void setupStates();

	DebouncedButton toggleGoBtn;
	DebouncedButton resetBtn;
	bool running;
	float manualSpeedX;
	float manualSpeedY;
	float manualOmega;

};

#endif // TESTCONTROLLER_H
