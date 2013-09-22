#include "SoccerBot.h"
#include "XimeaCamera.h"
#include "Vision.h"
#include "Communication.h"
#include "ProcessThread.h"
#include "Gui.h"
#include "FpsCounter.h"
#include "SignalHandler.h"
#include "Config.h"
#include "Util.h"
#include "Robot.h"
#include "Dribbler.h"
#include "Wheel.h"
#include "ManualController.h"

#include <iostream>

SoccerBot::SoccerBot() :
	frontCamera(NULL), rearCamera(NULL),
	frontBlobber(NULL), rearBlobber(NULL),
	frontVision(NULL), rearVision(NULL),
	frontProcessor(NULL), rearProcessor(NULL),
	gui(NULL), fpsCounter(NULL), visionResults(NULL), robot(NULL), activeController(NULL), server(NULL), com(NULL),
	running(false), playing(false), debugVision(false), showGui(false), controllerRequested(false),
	dt(0.01666f), lastStepTime(0.0f), totalTime(0.0f),
	targetSide(Side::UNKNOWN)
{

}

SoccerBot::~SoccerBot() {
	std::cout << "! Releasing all resources" << std::endl;

	for (std::map<std::string, Controller*>::iterator it = controllers.begin(); it != controllers.end(); it++) {
        delete it->second;
    }

    controllers.clear();
    activeController = NULL;

	if (gui != NULL) delete gui; gui = NULL;
	if (server != NULL) delete server; server = NULL;
	if (robot != NULL) delete robot; robot = NULL;
	if (frontCamera != NULL) delete frontCamera; frontCamera = NULL;
	if (rearCamera != NULL) delete rearCamera; rearCamera = NULL;
	if (fpsCounter != NULL) delete fpsCounter; fpsCounter = NULL;
	if (frontProcessor != NULL) frontBlobber->saveOptions(Config::blobberConfigFilename); delete frontProcessor; frontProcessor = NULL;
	if (rearProcessor != NULL) delete rearProcessor; rearProcessor = NULL;
	if (visionResults != NULL) delete visionResults; visionResults = NULL;
	if (frontVision != NULL) delete frontVision; frontVision = NULL;
	if (rearVision != NULL) delete rearVision; rearVision = NULL;
	if (frontBlobber != NULL) delete frontBlobber; frontBlobber = NULL;
	if (rearBlobber != NULL) delete rearBlobber; rearBlobber = NULL;
	if (com != NULL) delete com; com = NULL;

	std::cout << "! Resources freed" << std::endl;
}

void SoccerBot::setup() {
	setupVision();
	setupProcessors();
	setupFpsCounter();
	setupCameras();
	setupRobot();
	setupControllers();
	setupSignalHandler();
	setupServer();

	if (showGui) {
		setupGui();
	}

	setupCommunication();
}

void SoccerBot::run() {
	std::cout << "! Starting main loop" << std::endl;

	running = true;

	if (frontCamera->isOpened()) {
		frontCamera->startAcquisition();
	}

	if (rearCamera->isOpened()) {
		rearCamera->startAcquisition();
	}

	if (!frontCamera->isOpened() && !rearCamera->isOpened()) {
		std::cout << "! Neither of the cameras was opened, running in test mode" << std::endl;

		while (running) {
			Sleep(100);

			if (SignalHandler::exitRequested) {
				running = false;
			}
		}

		return;
	}

	bool gotFrontFrame, gotRearFrame;
	float time;

	while (running) {
		//__int64 startTime = Util::timerStart();

		time = Util::millitime();

		if (lastStepTime != 0.0f) {
			dt = time - lastStepTime;
		} else {
			dt = 1.0f / 60.0f;
		}

		totalTime += dt;

		handleServerMessages();
		handleCommunicationMessages();

		gotFrontFrame = gotRearFrame = false;
		frontProcessor->debug = rearProcessor->debug = debugVision || showGui;

		gotFrontFrame = fetchFrame(frontCamera, frontProcessor);
		gotRearFrame = fetchFrame(rearCamera, rearProcessor);

		if (!gotFrontFrame && !gotRearFrame && fpsCounter->frameNumber > 0) {
			std::cout << "- Didn't get any frames from either of the cameras" << std::endl;

			continue;
		}

		fpsCounter->step();

		if (gotFrontFrame) {
			frontProcessor->start();
		}

		if (gotRearFrame) {
			rearProcessor->start();
		}

		if (gotFrontFrame) {
			frontProcessor->join();
			visionResults->front = frontProcessor->visionResult;
		}

		if (gotRearFrame) {
			rearProcessor->join();
			visionResults->rear = rearProcessor->visionResult;
		}

		if (showGui) {
			if (gui == NULL) {
				setupGui();
			}

			gui->setFps(fpsCounter->getFps());

			if (gotFrontFrame) {
				gui->setFrontImages(
					frontProcessor->rgb,
					frontProcessor->dataYUYV,
					frontProcessor->dataY, frontProcessor->dataU, frontProcessor->dataV,
					frontProcessor->classification
				);
			}

			if (gotRearFrame) {
				gui->setRearImages(
					rearProcessor->rgb,
					rearProcessor->dataYUYV,
					rearProcessor->dataY, rearProcessor->dataU, rearProcessor->dataV,
					rearProcessor->classification
				);
			}

			gui->update();

			if (gui->isQuitRequested()) {
				running = false;
			}
		}

		if (fpsCounter->frameNumber % 60 == 0) {
			std::cout << "! FPS: " << fpsCounter->getFps() << std::endl;
		}

		if (SignalHandler::exitRequested) {
			running = false;
		}

		if (server != NULL) {
			server->broadcast(Util::json("state", getStateJSON()));
		}

		if (activeController != NULL) {
			activeController->step(dt, visionResults);
		}

		robot->step(dt, visionResults);

		lastStepTime = time;

		//std::cout << "! Total time: " << Util::timerEnd(startTime) << std::endl;
	}

	std::cout << "! Main loop ended" << std::endl;
}

bool SoccerBot::fetchFrame(XimeaCamera* camera, ProcessThread* processor) {
	if (camera->isAcquisitioning()) {
		const BaseCamera::Frame* frame = camera->getFrame();

		if (frame != NULL) {
			if (frame->fresh) {
				processor->setFrame(frame->data);

				return true;
			}
		}
	}

	return false;
}

void SoccerBot::setupVision() {
	std::cout << "! Setting up vision.. ";

	frontBlobber = new Blobber();
	rearBlobber = new Blobber();

	frontBlobber->initialize(Config::cameraWidth, Config::cameraHeight);
	rearBlobber->initialize(Config::cameraWidth, Config::cameraHeight);

	frontBlobber->loadOptions(Config::blobberConfigFilename);
	rearBlobber->loadOptions(Config::blobberConfigFilename);

	frontVision = new Vision(frontBlobber, Dir::FRONT, Config::cameraWidth, Config::cameraHeight);
	rearVision = new Vision(rearBlobber, Dir::REAR, Config::cameraWidth, Config::cameraHeight);

	visionResults = new Vision::Results();

	std::cout << "done!" << std::endl;
}

void SoccerBot::setupProcessors() {
	std::cout << "! Setting up processor threads.. ";

	frontProcessor = new ProcessThread(frontBlobber, frontVision);
	rearProcessor = new ProcessThread(rearBlobber, rearVision);

	std::cout << "done!" << std::endl;
}

void SoccerBot::setupFpsCounter() {
	std::cout << "! Setting up fps counter.. ";

	fpsCounter = new FpsCounter();

	std::cout << "done!" << std::endl;
}

void SoccerBot::setupGui() {
	std::cout << "! Setting up GUI" << std::endl;

	gui = new Gui(
		GetModuleHandle(0),
		frontBlobber, rearBlobber,
		Config::cameraWidth, Config::cameraHeight
	);
}

void SoccerBot::setupCameras() {
	std::cout << "! Setting up cameras" << std::endl;

	frontCamera = new XimeaCamera();
	rearCamera = new XimeaCamera();

	frontCamera->open(Config::frontCameraSerial);
	rearCamera->open(Config::rearCameraSerial);

	if (frontCamera->isOpened()) {
		setupCamera("Front", frontCamera);
	} else {
		std::cout << "- Opening front camera failed" << std::endl;
	}

	if (rearCamera->isOpened()) {
		setupCamera("Rear", rearCamera);
	} else {
		std::cout << "- Opening rear camera failed" << std::endl;
	}

	if (!frontCamera->isOpened() && !rearCamera->isOpened()) {
		std::cout << "! Neither of the cameras could be opened" << std::endl;
	}
}

void SoccerBot::setupRobot() {
	std::cout << "! Setting up the robot.. ";

	robot = new Robot();

	robot->setup();

	std::cout << "done!" << std::endl;
}

void SoccerBot::setupControllers() {
	std::cout << "! Setting up controllers.. ";

	addController("manual", new ManualController(robot, com));

	setController("manual");

	std::cout << "done!" << std::endl;
}

void SoccerBot::setupCamera(std::string name, XimeaCamera* camera) {
	camera->setGain(6);
	camera->setExposure(10000);
	camera->setFormat(XI_RAW8);
	camera->setAutoWhiteBalance(false);
	camera->setAutoExposureGain(false);
	camera->setQueueSize(12); // TODO Affects anything?

	std::cout << "! " << name << " camera info:" << std::endl;
	std::cout << "  > Name: " << camera->getName() << std::endl;
	std::cout << "  > Type: " << camera->getDeviceType() << std::endl;
	std::cout << "  > API version: " << camera->getApiVersion() << std::endl;
	std::cout << "  > Driver version: " << camera->getDriverVersion() << std::endl;
	std::cout << "  > Serial number: " << camera->getSerialNumber() << std::endl;
	std::cout << "  > Color: " << (camera->supportsColor() ? "yes" : "no") << std::endl;
	std::cout << "  > Framerate: " << camera->getFramerate() << std::endl;
	std::cout << "  > Available bandwidth: " << camera->getAvailableBandwidth() << std::endl;
}

void SoccerBot::setupSignalHandler() {
	SignalHandler::setup();
}

void SoccerBot::setupServer() {
	server = new Server();
	server->start();
}

void SoccerBot::setupCommunication() {
	com = new Communication(Config::communicationHost, Config::communicationPort);
	com->start();
}

void SoccerBot::addController(std::string name, Controller* controller) {
    controllers[name] = controller;
}

Controller* SoccerBot::getController(std::string name) {
    std::map<std::string, Controller*>::iterator result = controllers.find(name);

    if (result == controllers.end()) {
        return NULL;
    }

    return result->second;
}

bool SoccerBot::setController(std::string name) {
    if (name == "") {
		if (activeController != NULL) {
			activeController->onExit();
		}

		activeController = NULL;
		activeControllerName = "";
		controllerRequested = true;

		return true;
	} else {
		std::map<std::string, Controller*>::iterator result = controllers.find(name);
		
		if (result != controllers.end()) {
			if (activeController != NULL) {
				activeController->onExit();
			}

			activeController = result->second;
			activeControllerName = name;
			activeController->onEnter();

			controllerRequested = true;

			return true;
		} else {
			return false;
		}
    }
}

std::string SoccerBot::getActiveControllerName() {
	return activeControllerName;
}

void SoccerBot::handleServerMessages() {
	Server::Message* message;

	while ((message = server->popLastMessage()) != NULL) {
		handleServerMessage(message);

		delete message;
	}
}

void SoccerBot::handleServerMessage(Server::Message* message) {
	std::cout << "! Request from " << message->client->id << ": " << message->content << std::endl;

	if (Command::isValid(message->content)) {
        Command command = Command::parse(message->content);

        if (
			activeController == NULL
			|| (!activeController->handleCommand(command) && !activeController->handleRequest(message->content))
		) {
			if (command.name == "get-controller") {
				handleGetController(message);
			} else if (command.name == "set-controller") {
				handleSetController(command.parameters);
			} else {
				std::cout << "- Unsupported command: " << command.name << std::endl;
			}
		}
	} else {
		std::cout << "- Message '" << message->content << "' is not a valid command" << std::endl;
	}
}

void SoccerBot::handleGetController(Server::Message* message) {
	message->respond(Util::json("controller", activeControllerName));
}

void SoccerBot::handleSetController(Command::Parameters parameters) {
	std::string name = parameters[0];

	if (setController(name)) {
		std::cout << "+ Changed controller to: '" << name << "'" << std::endl;
	} else {
		std::cout << "- Failed setting controller to '" << name << "'" << std::endl;
	}
}

void SoccerBot::handleCommunicationMessages() {
	std::string message;

	while ((message = com->popLastMessage()) != "") {
		handleCommunicationMessage(message);
	}
}

void SoccerBot::handleCommunicationMessage(std::string message) {
	robot->handleCommunicationMessage(message);

	if (activeController != NULL) {
		activeController->handleCommunicationMessage(message);
	}

	/*if (Command::isValid(message)) {
        Command command = Command::parse(message);

		// do something?
	}*/
}

std::string SoccerBot::getStateJSON() {
	std::stringstream stream;

    Math::Position pos = robot->getPosition();

    stream << "{";

    // general robot info
    stream << "\"x\":" << pos.x << ",";
    stream << "\"y\":" << pos.y << ",";
    stream << "\"orientation\":" << pos.orientation << ",";
    stream << "\"dt\":" << dt << ",";
    //stream << "\"load\":" << lastStepLoad << ",";
    //stream << "\"duration\":" << lastStepDuration << ",";
	//stream << "\"isError\":" << (infoBoard->isError() ? "true" : "false") << ",";
	//stream << "\"isViewObstructed\":" << (vision->isViewObstructed() ? "true" : "false") << ",";
	//stream << "\"robotInWay\":" << vision->getRobotInWay() << ",";
    stream << "\"totalTime\":" << totalTime << ",";
	stream << "\"gotBall\":" << robot->getDribbler()->gotBall() << ",";

    // wheels
    stream << "\"wheelFL\": {";
	stream << "\"stalled\":" << (robot->getWheelFL()->isStalled() ? "true" : "false") << ",";
    stream << "\"targetOmega\":" << robot->getWheelFL()->getTargetOmega() << ",";
    stream << "\"realOmega\":" << robot->getWheelFL()->getRealOmega();
    stream << "},";

    // front right wheel
    stream << "\"wheelFR\": {";
	stream << "\"stalled\":" << (robot->getWheelFR()->isStalled() ? "true" : "false") << ",";
    stream << "\"targetOmega\":" << robot->getWheelFR()->getTargetOmega() << ",";
    stream << "\"realOmega\":" << robot->getWheelFR()->getRealOmega();
    stream << "},";

    // rear left wheel
    stream << "\"wheelRL\": {";
	stream << "\"stalled\":" << (robot->getWheelRL()->isStalled() ? "true" : "false") << ",";
    stream << "\"targetOmega\":" << robot->getWheelRL()->getTargetOmega() << ",";
    stream << "\"realOmega\":" << robot->getWheelRL()->getRealOmega();
    stream << "},";

    // rear right wheel
    stream << "\"wheelRR\": {";
	stream << "\"stalled\":" << (robot->getWheelRR()->isStalled() ? "true" : "false") << ",";
    stream << "\"targetOmega\":" << robot->getWheelRR()->getTargetOmega() << ",";
    stream << "\"realOmega\":" << robot->getWheelRR()->getRealOmega();
    stream << "},";

    // tasks
    stream << "\"tasks\": [";

    TaskQueue tasks = robot->getTasks();
    bool first = true;

    for (TaskQueueIt it = tasks.begin(); it != tasks.end(); it++) {
        Task* task = *it;

        if (!first) {
            stream << ",";
        } else {
            first = false;
        }

        stream << "{";
        stream << "\"started\": " << (task->isStarted() ? "true" : "false") << ",";
        stream << "\"percentage\": " << task->getPercentage() << ",";
        stream << "\"status\": \"" << task->toString() << "\"";
        stream << "}";
    }

    stream << "],";

	/*
	// balls
	ObjectList* balls = NULL;
	ObjectList frontBalls = vision->getFrontBalls();
	ObjectList rearBalls = vision->getRearBalls();
	Object* ball;

	first = true;

	stream << "\"balls\": [";

	for (int i = 0; i < 2; i++) {
		balls = i == 0 ? &frontBalls : &rearBalls;

		for (ObjectListItc it = balls->begin(); it != balls->end(); it++) {
			ball = *it;

			if (!first) {
				stream << ",";
			} else {
				first = false;
			}

			stream << "{";
			stream << "\"x\": " << ball->x << ",";
			stream << "\"y\": " << ball->y << ",";
			stream << "\"width\": " << ball->width << ",";
			stream << "\"height\": " << ball->height << ",";
			stream << "\"distance\": " << ball->distance << ",";
			stream << "\"angle\": " << ball->angle << ",";
			stream << "\"camera\": \"" << (i == 0 ? "front" : "rear") << "\"";
			stream << "}";
		}
	}

	stream << "],";

	// goals
	ObjectList* goals = NULL;
	ObjectList frontGoals = vision->getFrontGoals();
	ObjectList rearGoals = vision->getRearGoals();
	Object* goal;

	first = true;

	stream << "\"goals\": [";

	for (int i = 0; i < 2; i++) {
		goals = i == 0 ? &frontGoals : &rearGoals;
		
		for (ObjectListItc it = goals->begin(); it != goals->end(); it++) {
			goal = *it;

			if (!first) {
				stream << ",";
			} else {
				first = false;
			}

			stream << "{";
			stream << "\"x\": " << goal->x << ",";
			stream << "\"y\": " << goal->y << ",";
			stream << "\"width\": " << goal->width << ",";
			stream << "\"height\": " << goal->height << ",";
			stream << "\"distance\": " << goal->distance << ",";
			stream << "\"angle\": " << goal->angle << ",";
			stream << "\"camera\": \"" << (i == 0 ? "front" : "rear") << "\"";
			stream << "}";
		}
	}

	stream << "],";
	*/

	/*
	stream << "\"measurements\": {";

	const Measurements measurements = robot->getMeasurements();

	first = true;

	for (Measurements::const_iterator it = measurements.begin(); it != measurements.end(); it++) {
		if (!first) {
			stream << ",";
		} else {
			first = false;
		}

		stream << "\"" + it->first + "\": \"" + Util::toString(it->second) + "\"";
	}

	stream << "},";
	*/

	if (activeController != NULL) {
		stream << "\"controllerName\": \"" + activeControllerName + "\",";
		std::string controllerInfo = activeController->getJSON();

		if (controllerInfo.length() > 0) {
			stream << "\"controllerState\": " << controllerInfo << ",";
		} else {
			stream << "\"controllerState\": null,";
		}
	} else {
		stream << "\"controllerName\": null,";
		stream << "\"controllerState\": null,";
	}

	stream << "\"fps\":" << fpsCounter->getFps()  << ",";
	stream << "\"targetGoal\":" << targetSide << ",";
	stream << "\"playing\":" << (playing ? "1" : "0");

    stream << "}";

    return stream.str();
}
