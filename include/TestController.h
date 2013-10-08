#ifndef TESTCONTROLLER_H
#define TESTCONTROLLER_H

#include "BaseAI.h"
#include "Vision.h"
#include "DebouncedButton.h"
#include "Config.h"

#include <map>
#include <string>

class TestController : public BaseAI {

public:
	class State : public BaseAI::State {

	public:
		State(TestController* ai) : BaseAI::State(ai), ai(ai) {}

	protected:
		TestController* ai;

	};

	class ManualControlState : public State {

	public:
		ManualControlState(TestController* ai) : State(ai) {}
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

	class FetchBallInfrontState : public State {

	public:
		FetchBallInfrontState(TestController* ai) : State(ai) {}
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration);

	};

	
	TestController(Robot* robot, Communication* com);
	~TestController();

    bool handleCommand(const Command& cmd);
	void handleTargetVectorCommand(const Command& cmd);
	void handleResetCommand();
	void handleDriveToCommand(const Command& cmd);
	void handleParameterCommand(const Command& cmd);

    void step(float dt, Vision::Results* visionResults);
	void dbg(std::string key, std::string value) { messages[key] = value; }
	std::string getJSON();

private:
	void setupStates();
	void updateGoalDistances(Vision::Results* visionResults);

	DebouncedButton toggleGoBtn;
	DebouncedButton resetBtn;

	float manualSpeedX;
	float manualSpeedY;
	float manualOmega;
	float blueGoalDistance;
	float yellowGoalDistance;

	double lastCommandTime;

	std::map<int, std::string> parameters;
	std::map<std::string, std::string> messages;

};

#endif // TESTCONTROLLER_H
