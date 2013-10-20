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
	enum TargetMode { LEFT = -1, INLINE = 0, RIGHT = 1, UNDECIDED = 2 };

	typedef std::map<std::string, std::string> Messages;
	typedef Messages::iterator MessagesIt;
	typedef std::map<int, std::string> Parameters;

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

	class FindBallState : public State {

	public:
		FindBallState(TestController* ai) : State(ai) {}
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration);

	};

	class FetchBallFrontState : public State {

	public:
		FetchBallFrontState(TestController* ai) : State(ai), startBrakingDistance(-1.0f), startBrakingVelocity(-1.0f), lastBallDistance(-1.0f) {}
		void onEnter(Robot* robot);
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration);

	private:
		void reset();

		float startBrakingDistance;
		float startBrakingVelocity;
		float lastBallDistance;

	};

	class FetchBallDirectState : public State {

	public:
		FetchBallDirectState(TestController* ai) : State(ai), enterVelocity(-1.0f) {}
		void onEnter(Robot* robot);
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration);

	private:
		float enterVelocity;

	};

	class FetchBallBehindState : public State {

	public:
		FetchBallBehindState(TestController* ai) : State(ai), hadBall(false), lastTargetAngle(0.0f), lostBallTime(-1.0), timeSinceLostBall(0.0), lostBallVelocity(0.0f), startBallDistance(-1.0f), targetMode(TargetMode::UNDECIDED) {}
		void onEnter(Robot* robot);
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration);

	private:
		bool hadBall;
		float lastTargetAngle;
		double lostBallTime;
		double timeSinceLostBall;
		float lostBallVelocity;
		float startBallDistance;
		TargetMode targetMode;

	};

	class FetchBallNearState : public State {

	public:
		FetchBallNearState(TestController* ai) : State(ai), enterDistance(-1.0f), enterVelocity(0.0f), enterBallDistance(-1.0f), smallestForwardSpeed(-1.0f) {}
		void onEnter(Robot* robot);
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration);

	private:
		float enterDistance;
		float enterVelocity;
		float enterBallDistance;
		float smallestForwardSpeed;

	};

	class AimState : public State {

	public:
		void onEnter(Robot* robot);
		AimState(TestController* ai) : State(ai), lastKickTime(-1.0), avoidBallSide(TargetMode::UNDECIDED), searchGoalDir(0.0f) {}
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration);

	private:
		double lastKickTime;
		TargetMode avoidBallSide;
		float searchGoalDir;

	};

	class DriveCircleState : public State {

	public:
		DriveCircleState(TestController* ai) : State(ai) {}
		void step(float dt, Vision::Results* visionResults, Robot* robot, float totalDuration, float stateDuration);

	private:
		static float getCircleTargetAngle(float start, float time, float period);

	};
	
	TestController(Robot* robot, Communication* com);
	~TestController();

    bool handleCommand(const Command& cmd);
	void handleTargetVectorCommand(const Command& cmd);
	void handleDribblerCommand(const Command& cmd);
	void handleKickCommand(const Command& cmd);
	void handleResetCommand();
	void handleToggleSideCommand();
	void handleDriveToCommand(const Command& cmd);
	void handleParameterCommand(const Command& cmd);

	float getTargetAngle(float goalX, float goalY, float ballX, float ballY, float D, TargetMode targetMode = TargetMode::INLINE);

    void step(float dt, Vision::Results* visionResults);
	Side getTargetSide() { return targetSide; }
	void dbgs(std::string key, std::string value) { messages[key] = value; }
	void dbg(std::string key, int value) { messages[key] = Util::toString(value); }
	void dbg(std::string key, float value) { messages[key] = Util::toString(value); }
	void dbg(std::string key, double value) { messages[key] = Util::toString(value); }
	void dbg(std::string key, bool value) { messages[key] = value ? "true" : "false"; }
	std::string getJSON();

private:
	void setupStates();
	void updateGoalDistances(Vision::Results* visionResults);

	DebouncedButton toggleGoBtn;
	DebouncedButton toggleSideBtn;
	DebouncedButton resetBtn;

	Side targetSide;
	float manualSpeedX;
	float manualSpeedY;
	float manualOmega;
	int manualDribblerSpeed;
	int manualKickStrength;
	float blueGoalDistance;
	float yellowGoalDistance;
	float lastTargetGoalAngle;

	double lastCommandTime;

	Parameters parameters;
	Messages messages;

};

#endif // TESTCONTROLLER_H
