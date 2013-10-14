#ifndef TESTCONTROLLER_H
#define TESTCONTROLLER_H

#include "BaseAI.h"
#include "Vision.h"
#include "DebouncedButton.h"
#include "Util.h"
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

	class FetchBallBehindState : public State {

	public:
		FetchBallBehindState(TestController* ai) : State(ai) {}
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration);

	};

	class FetchBallStraightState : public State {

	public:
		FetchBallStraightState(TestController* ai) : State(ai) {}
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration);

	private:
		float getTargetAngle(float goalX, float goalY, float ballX, float ballY, float D);

	};

	class FetchBallNearState : public State {

	public:
		FetchBallNearState(TestController* ai) : State(ai) {}
		void onEnter(Robot* robot);
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration);

	private:
		float enterVelocity;

	};

	class AimState : public State {

	public:
		AimState(TestController* ai) : State(ai), lastKickTime(0.0) {}
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration);

	private:
		double lastKickTime;

	};

	typedef std::map<std::string, std::string> Messages;
	typedef Messages::iterator MessagesIt;
	typedef std::map<int, std::string> Parameters;
	
	TestController(Robot* robot, Communication* com);
	~TestController();

    bool handleCommand(const Command& cmd);
	void handleTargetVectorCommand(const Command& cmd);
	void handleDribblerCommand(const Command& cmd);
	void handleKickCommand(const Command& cmd);
	void handleResetCommand();
	void handleDriveToCommand(const Command& cmd);
	void handleParameterCommand(const Command& cmd);

    void step(float dt, Vision::Results* visionResults);
	void dbg(std::string key, std::string value) { messages[key] = value; }
	void dbg(std::string key, int value) { messages[key] = Util::toString(value); }
	void dbg(std::string key, float value) { messages[key] = Util::toString(value); }
	void dbg(std::string key, double value) { messages[key] = Util::toString(value); }
	void dbg(std::string key, bool value) { messages[key] = value ? "true" : "false"; }
	std::string getJSON();

private:
	void setupStates();
	void updateGoalDistances(Vision::Results* visionResults);

	DebouncedButton toggleGoBtn;
	DebouncedButton resetBtn;

	float manualSpeedX;
	float manualSpeedY;
	float manualOmega;
	int manualDribblerSpeed;
	int manualKickStrength;
	float blueGoalDistance;
	float yellowGoalDistance;

	double lastCommandTime;

	Parameters parameters;
	Messages messages;

};

#endif // TESTCONTROLLER_H
