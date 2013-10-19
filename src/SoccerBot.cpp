#include "SoccerBot.h"
#include "XimeaCamera.h"
#include "VirtualCamera.h"
#include "CameraTranslator.h"
#include "Vision.h"
#include "DebugRenderer.h"
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
#include "TestController.h"
#include "OffensiveAI.h"
#include "ImageProcessor.h"

#include <iostream>
#include <algorithm>

SoccerBot::SoccerBot() :
	frontCamera(NULL), rearCamera(NULL),
	ximeaFrontCamera(NULL), ximeaRearCamera(NULL),
	virtualFrontCamera(NULL), virtualRearCamera(NULL),
	frontBlobber(NULL), rearBlobber(NULL),
	frontVision(NULL), rearVision(NULL),
	frontProcessor(NULL), rearProcessor(NULL),
	frontCameraTranslator(NULL), rearCameraTranslator(NULL),
	gui(NULL), fpsCounter(NULL), visionResults(NULL), robot(NULL), activeController(NULL), server(NULL), com(NULL),
	jpegBuffer(NULL), screenshotBufferFront(NULL), screenshotBufferRear(NULL),
	running(false), debugVision(false), showGui(false), controllerRequested(false), stateRequested(false), frameRequested(false), useScreenshot(false),
	dt(0.01666f), lastStepTime(0.0), totalTime(0.0f),
	debugCameraDir(Dir::FRONT)
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
	if (ximeaFrontCamera != NULL) delete ximeaFrontCamera; ximeaFrontCamera = NULL;
	if (ximeaRearCamera != NULL) delete ximeaRearCamera; ximeaRearCamera = NULL;
	if (virtualFrontCamera != NULL) delete virtualFrontCamera; virtualFrontCamera = NULL;
	if (virtualRearCamera != NULL) delete virtualRearCamera; virtualRearCamera = NULL;
	if (frontCameraTranslator != NULL) delete frontCameraTranslator; frontCameraTranslator = NULL;
	if (rearCameraTranslator != NULL) delete rearCameraTranslator; rearCameraTranslator = NULL;
	if (fpsCounter != NULL) delete fpsCounter; fpsCounter = NULL;
	if (frontProcessor != NULL) frontBlobber->saveOptions(Config::blobberConfigFilename); delete frontProcessor; frontProcessor = NULL;
	if (rearProcessor != NULL) delete rearProcessor; rearProcessor = NULL;
	if (visionResults != NULL) delete visionResults; visionResults = NULL;
	if (frontVision != NULL) delete frontVision; frontVision = NULL;
	if (rearVision != NULL) delete rearVision; rearVision = NULL;
	if (frontBlobber != NULL) delete frontBlobber; frontBlobber = NULL;
	if (rearBlobber != NULL) delete rearBlobber; rearBlobber = NULL;
	if (com != NULL) delete com; com = NULL;
	if (jpegBuffer != NULL) delete jpegBuffer; jpegBuffer = NULL;

	frontCamera = NULL;
	rearCamera = NULL;

	std::cout << "! Resources freed" << std::endl;
}

void SoccerBot::setup() {
	setupCommunication();
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
}

void SoccerBot::run() {
	std::cout << "! Starting main loop" << std::endl;

	running = true;

	com->start();
	server->start();

	com->send("reset");

	setController(Config::defaultController);

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
	double time;
	double debugging;

	while (running) {
		//__int64 startTime = Util::timerStart();

		time = Util::millitime();

		if (lastStepTime != 0.0) {
			dt = (float)(time - lastStepTime);
		} else {
			dt = 1.0f / 60.0f;
		}

		totalTime += dt;

		gotFrontFrame = gotRearFrame = false;
		debugging = frontProcessor->debug = rearProcessor->debug = debugVision || showGui || frameRequested;

		gotFrontFrame = fetchFrame(frontCamera, frontProcessor);
		gotRearFrame = fetchFrame(rearCamera, rearProcessor);

		if (!gotFrontFrame && !gotRearFrame && fpsCounter->frameNumber > 0) {
			//std::cout << "- Didn't get any frames from either of the cameras" << std::endl;

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

		if (debugging) {
			Object* closestBall = visionResults->getClosestBall();

			if (closestBall != NULL) {
				if (!closestBall->behind) {
					DebugRenderer::renderObjectHighlight(frontProcessor->rgb, closestBall, 255, 0, 0);
				} else {
					DebugRenderer::renderObjectHighlight(rearProcessor->rgb, closestBall, 255, 0, 0);
				}
			}

			Object* largestBlueGoal = visionResults->getLargestGoal(Side::BLUE);
			Object* largestYellowGoal = visionResults->getLargestGoal(Side::YELLOW);

			if (largestBlueGoal != NULL) {
				if (!largestBlueGoal->behind) {
					DebugRenderer::renderObjectHighlight(frontProcessor->rgb, largestBlueGoal, 0, 0, 255);
				} else {
					DebugRenderer::renderObjectHighlight(rearProcessor->rgb, largestBlueGoal, 0, 0, 255);
				}
			}

			if (largestYellowGoal != NULL) {
				if (!largestYellowGoal->behind) {
					DebugRenderer::renderObjectHighlight(frontProcessor->rgb, largestYellowGoal, 255, 255, 0);
				} else {
					DebugRenderer::renderObjectHighlight(rearProcessor->rgb, largestYellowGoal, 255, 255, 0);
				}
			}
			//DebugRenderer::highlightObject(
		}

		if (frameRequested) {
			// TODO Add camera choice
			if (debugCameraDir == Dir::FRONT) {
				broadcastFrame(frontProcessor->rgb, frontProcessor->classification);
			} else {
				broadcastFrame(rearProcessor->rgb, rearProcessor->classification);
			}

			frameRequested = false;
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

		handleServerMessages();
		handleCommunicationMessages();

		if (activeController != NULL) {
			activeController->step(dt, visionResults);
		}

		robot->step(dt, visionResults);

		if (server != NULL && stateRequested) {
			server->broadcast(Util::json("state", getStateJSON()));

			stateRequested = false;
		}

		lastStepTime = time;

		if (SignalHandler::exitRequested) {
			running = false;
		}

		//std::cout << "! Total time: " << Util::timerEnd(startTime) << std::endl;
	}

	com->send("reset");

	std::cout << "! Main loop ended" << std::endl;
}

bool SoccerBot::fetchFrame(BaseCamera* camera, ProcessThread* processor) {
	if (camera->isAcquisitioning()) {
		const BaseCamera::Frame* frame = camera->getFrame();

		//ImageProcessor::loadBitmap("screenshot.bin", frame->data, frame->width * frame->height * 4);

		if (frame != NULL) {
			if (frame->fresh) {
				processor->setFrame(frame->data);

				return true;
			}
		}
	}

	return false;
}

void SoccerBot::broadcastFrame(unsigned char* rgb, unsigned char* classification) {
	int jpegBufferSize = Config::jpegBufferSize;

	if (jpegBuffer == NULL) {
		std::cout << "! Creating frame JPEG buffer of " << jpegBufferSize << " bytes.. ";
        jpegBuffer = new unsigned char[Config::jpegBufferSize];
		std::cout << "done!" << std::endl;
    }

	if (!ImageProcessor::rgbToJpeg(rgb, jpegBuffer, jpegBufferSize, Config::cameraWidth, Config::cameraHeight)) {
		std::cout << "- Converting RGB image to JPEG failed, probably need to increase buffer size" << std::endl;

		return;
	}

	std::string base64Rgb = Util::base64Encode(jpegBuffer, jpegBufferSize);

	jpegBufferSize = Config::jpegBufferSize;

	if (!ImageProcessor::rgbToJpeg(classification, jpegBuffer, jpegBufferSize, Config::cameraWidth, Config::cameraHeight)) {
		std::cout << "- Converting classification image to JPEG failed, probably need to increase buffer size" << std::endl;

		return;
	}

	std::string base64Classification = Util::base64Encode(jpegBuffer, jpegBufferSize);
	std::string frameResponse = Util::json("frame", "{\"rgb\": \"" + base64Rgb + "\",\"classification\": \"" + base64Classification + "\",\"activeStream\":\"" + activeStreamName + "\",\"cameraK\":" + Util::toString(Util::cameraCorrectionK) + ",\"cameraZoom\":" + Util::toString(Util::cameraCorrectionZoom) + "}");

	server->broadcast(frameResponse);
}

void SoccerBot::broadcastScreenshots() {
	std::vector<std::string> screenshotFiles = Util::getFilesInDir(Config::screenshotsDirectory);
	std::vector<std::string> screenshotNames;
	std::string filename;
	std::string screenshotName;
	int dashPos;

	std::cout << "! Screenshots:" << std::endl;

	for (std::vector<std::string>::const_iterator it = screenshotFiles.begin(); it != screenshotFiles.end(); it++) {
		filename = *it;

		//std::cout << "  > " << filename << std::endl;

		dashPos = Util::strpos(filename, "-");

		if (dashPos != -1) {
			screenshotName = filename.substr(0, dashPos);

			if (std::find(screenshotNames.begin(), screenshotNames.end(), screenshotName) == screenshotNames.end()) {
				screenshotNames.push_back(screenshotName);

				std::cout << "  > " << screenshotName << std::endl;
			}
		}
	}

	std::string response = Util::json("screenshots", Util::toString(screenshotNames));

	server->broadcast(response);
}

void SoccerBot::setupVision() {
	std::cout << "! Setting up vision.. ";

	frontBlobber = new Blobber();
	rearBlobber = new Blobber();

	frontBlobber->initialize(Config::cameraWidth, Config::cameraHeight);
	rearBlobber->initialize(Config::cameraWidth, Config::cameraHeight);

	frontBlobber->loadOptions(Config::blobberConfigFilename);
	rearBlobber->loadOptions(Config::blobberConfigFilename);

	frontCameraTranslator = new CameraTranslator();
	rearCameraTranslator = new CameraTranslator();

	// TODO Add to config or load from file
	frontCameraTranslator->setConstants(
		-73.0f, 249000.0f, 186.6f,
		0.0f, 0.0f, 0.0f,
		-19.0f, Config::cameraWidth, Config::cameraHeight
	);

	rearCameraTranslator->setConstants(
		21.5f, 187000.0f, 142.5f,
		0.0f, 0.0f, 0.0f,
		19.0f, Config::cameraWidth, Config::cameraHeight
	);

	// TODO Remove this test
	/*for (int x = 0; x <= Config::cameraWidth; x += Config::cameraWidth / 2) {
		for (int y = 0; y <= Config::cameraHeight; y+= Config::cameraHeight / 2) {
			CameraTranslator::CameraPosition distorted = rearCameraTranslator->distort(x, y);
			CameraTranslator::CameraPosition undistorted = rearCameraTranslator->undistort(x, y);

			//std::cout << "@ " << x << "x" << y << " DISTORTED: " << distorted.x << "x" << distorted.y << " UNDISTORTED: " << undistorted.x << "x" << undistorted.y << std::endl;
		}
	}*/

	frontVision = new Vision(frontBlobber, frontCameraTranslator, Dir::FRONT, Config::cameraWidth, Config::cameraHeight);
	rearVision = new Vision(rearBlobber, rearCameraTranslator, Dir::REAR, Config::cameraWidth, Config::cameraHeight);

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

	ximeaFrontCamera = new XimeaCamera();
	ximeaRearCamera = new XimeaCamera();

	ximeaFrontCamera->open(Config::frontCameraSerial);
	ximeaRearCamera->open(Config::rearCameraSerial);

	if (ximeaFrontCamera->isOpened()) {
		setupXimeaCamera("Front", ximeaFrontCamera);
	} else {
		std::cout << "- Opening front camera failed" << std::endl;
	}

	if (ximeaRearCamera->isOpened()) {
		setupXimeaCamera("Rear", ximeaRearCamera);
	} else {
		std::cout << "- Opening rear camera failed" << std::endl;
	}

	if (!ximeaFrontCamera->isOpened() && !ximeaRearCamera->isOpened()) {
		std::cout << "! Neither of the cameras could be opened" << std::endl;
	}

	virtualFrontCamera = new VirtualCamera();
	virtualRearCamera = new VirtualCamera();

	frontCamera = ximeaFrontCamera;
	rearCamera = ximeaRearCamera;
}

void SoccerBot::setupRobot() {
	std::cout << "! Setting up the robot.. ";

	robot = new Robot(com);

	robot->setup();

	std::cout << "done!" << std::endl;
}

void SoccerBot::setupControllers() {
	std::cout << "! Setting up controllers.. ";

	addController("manual", new ManualController(robot, com));
	addController("test", new TestController(robot, com));
	addController("offensive-ai", new OffensiveAI(robot, com));

	std::cout << "done!" << std::endl;
}

void SoccerBot::setupXimeaCamera(std::string name, XimeaCamera* camera) {
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
}

void SoccerBot::setupCommunication() {
	com = new Communication(Config::communicationHost, Config::communicationPort);
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

	while ((message = server->dequeueMessage()) != NULL) {
		handleServerMessage(message);

		delete message;
	}
}

void SoccerBot::handleServerMessage(Server::Message* message) {
	//std::cout << "! Request from " << message->client->id << ": " << message->content << std::endl;

	if (Command::isValid(message->content)) {
        Command command = Command::parse(message->content);

        if (
			activeController == NULL
			|| (!activeController->handleCommand(command) && !activeController->handleRequest(message->content))
		) {
			if (command.name == "get-controller") {
				handleGetControllerCommand(message);
			} else if (command.name == "set-controller" && command.parameters.size() == 1) {
				handleSetControllerCommand(command.parameters, message);
			} else if (command.name == "get-state") {
				handleGetStateCommand();
			} else if (command.name == "get-frame") {
				handleGetFrameCommand();
			} else if (command.name == "camera-choice" && command.parameters.size() == 1) {
                handleCameraChoiceCommand(command.parameters);
            } else if (command.name == "camera-adjust" && command.parameters.size() == 2) {
                handleCameraAdjustCommand(command.parameters);
            } else if (command.name == "stream-choice" && command.parameters.size() == 1) {
                handleStreamChoiceCommand(command.parameters);
            } else if (command.name == "blobber-threshold" && command.parameters.size() == 6) {
                handleBlobberThresholdCommand(command.parameters);
            } else if (command.name == "blobber-clear" && (command.parameters.size() == 0 || command.parameters.size() == 1)) {
                handleBlobberClearCommand(command.parameters);
            } else if (command.name == "screenshot" && command.parameters.size() == 1) {
                handleScreenshotCommand(command.parameters);
            } else if (command.name == "list-screenshots") {
                handleListScreenshotsCommand(message);
            } else {
				std::cout << "- Unsupported command: " << command.name << " " << Util::toString(command.parameters) << std::endl;
			}
		}
	} else {
		std::cout << "- Message '" << message->content << "' is not a valid command" << std::endl;
	}
}

void SoccerBot::handleGetControllerCommand(Server::Message* message) {
	std::cout << "! Client #" << message->client->id << " requested controller, sending: " << activeControllerName << std::endl;

	message->respond(Util::json("controller", activeControllerName));
}

void SoccerBot::handleSetControllerCommand(Command::Parameters parameters, Server::Message* message) {
	std::string name = parameters[0];

	if (setController(name)) {
		std::cout << "+ Changed controller to: '" << name << "'" << std::endl;
	} else {
		std::cout << "- Failed setting controller to '" << name << "'" << std::endl;
	}

	message->respond(Util::json("controller", activeControllerName));
}

void SoccerBot::handleGetStateCommand() {
	stateRequested = true;
}

void SoccerBot::handleGetFrameCommand() {
	frameRequested = true;
}

void SoccerBot::handleCameraChoiceCommand(Command::Parameters parameters) {
	debugCameraDir = Util::toInt(parameters[0]) == 2 ? Dir::REAR : Dir::FRONT;

	std::cout << "! Debugging now from " << (debugCameraDir == Dir::FRONT ? "front" : "rear") << " camera" << std::endl;
}

void SoccerBot::handleCameraAdjustCommand(Command::Parameters parameters) {
	Util::cameraCorrectionK = Util::toFloat(parameters[0]);
	Util::cameraCorrectionZoom = Util::toFloat(parameters[1]);

	std::cout << "! Adjust camera correction k: " << Util::cameraCorrectionK << ", zoom: " << Util::cameraCorrectionZoom << std::endl;
}

void SoccerBot::handleStreamChoiceCommand(Command::Parameters parameters) {
	std::string requestedStream = parameters[0];

	if (requestedStream == "") {
		std::cout << "! Switching to live stream" << std::endl;

		frontCamera = ximeaFrontCamera;
		rearCamera = ximeaRearCamera;

		activeStreamName = requestedStream;
	} else {
		try {
			bool frontSuccess = virtualFrontCamera->loadImage(Config::screenshotsDirectory + "/" + requestedStream + "-front.scr", Config::cameraWidth * Config::cameraHeight * 4);
			bool rearSuccess = virtualRearCamera->loadImage(Config::screenshotsDirectory + "/" + requestedStream + "-rear.scr", Config::cameraWidth * Config::cameraHeight * 4);

			if (!frontSuccess || !rearSuccess) {
				std::cout << "- Loading screenshot '" << requestedStream << "' failed" << std::endl;

				return;
			}

			std::cout << "! Switching to screenshot stream: " << requestedStream << std::endl;

			frontCamera = virtualFrontCamera;
			rearCamera = virtualRearCamera;

			activeStreamName = requestedStream;
		} catch (std::exception& e) {
			std::cout << "- Failed to load screenshot: " << requestedStream << " (" << e.what() << ")" << std::endl;
		} catch (...) {
			std::cout << "- Failed to load screenshot: " << requestedStream << std::endl;
		}
	}
}

void SoccerBot::handleBlobberThresholdCommand(Command::Parameters parameters) {
	std::string selectedColorName = parameters[0];
    int centerX = Util::toInt(parameters[1]);
    int centerY = Util::toInt(parameters[2]);
    int mode = Util::toInt(parameters[3]);
    int brushRadius = Util::toInt(parameters[4]);
    float stdDev = Util::toFloat(parameters[5]);

	unsigned char* dataY = debugCameraDir == Dir::FRONT ? frontProcessor->dataY : rearProcessor->dataY;
	unsigned char* dataU = debugCameraDir == Dir::FRONT ? frontProcessor->dataU : rearProcessor->dataU;
	unsigned char* dataV = debugCameraDir == Dir::FRONT ? frontProcessor->dataV : rearProcessor->dataV;

	ImageProcessor::YUYVRange yuyvRange = ImageProcessor::extractColorRange(dataY, dataU, dataV, Config::cameraWidth, Config::cameraHeight, centerX, centerY, brushRadius, stdDev);

	frontBlobber->getColor(selectedColorName)->addThreshold(
		yuyvRange.minY, yuyvRange.maxY,
		yuyvRange.minU, yuyvRange.maxU,
		yuyvRange.minV, yuyvRange.maxV
	);
	rearBlobber->getColor(selectedColorName)->addThreshold(
		yuyvRange.minY, yuyvRange.maxY,
		yuyvRange.minU, yuyvRange.maxU,
		yuyvRange.minV, yuyvRange.maxV
	);
}

void SoccerBot::handleBlobberClearCommand(Command::Parameters parameters) {
	if (parameters.size() == 1) {
		std::string color = parameters[0];

		frontBlobber->clearColor(color);
		rearBlobber->clearColor(color);
	} else {
		frontBlobber->clearColors();
		rearBlobber->clearColors();
	}
}

void SoccerBot::handleScreenshotCommand(Command::Parameters parameters) {
	std::string name = parameters[0];

	std::cout << "! Storing screenshot: " << name << std::endl;

	ImageProcessor::saveBitmap(frontProcessor->frame, Config::screenshotsDirectory + "/" + name + "-front.scr", Config::cameraWidth * Config::cameraHeight * 4);
	ImageProcessor::saveBitmap(rearProcessor->frame, Config::screenshotsDirectory + "/" + name + "-rear.scr", Config::cameraWidth * Config::cameraHeight * 4);
	
	ImageProcessor::saveJPEG(frontProcessor->rgb, Config::screenshotsDirectory + "/" + name + "-rgb-front.jpeg", Config::cameraWidth, Config::cameraHeight, 3);
	ImageProcessor::saveJPEG(frontProcessor->classification, Config::screenshotsDirectory + "/" + name + "-classification-front.jpeg", Config::cameraWidth, Config::cameraHeight, 3);

	ImageProcessor::saveJPEG(rearProcessor->rgb, Config::screenshotsDirectory + "/" + name + "-rgb-rear.jpeg", Config::cameraWidth, Config::cameraHeight, 3);
	ImageProcessor::saveJPEG(rearProcessor->classification, Config::screenshotsDirectory + "/" + name + "-classification-rear.jpeg", Config::cameraWidth, Config::cameraHeight, 3);

	broadcastScreenshots();
}

void SoccerBot::handleListScreenshotsCommand(Server::Message* message) {
	broadcastScreenshots();
}

void SoccerBot::handleCommunicationMessages() {
	std::string message;

	while ((message = com->dequeueMessage()) != "") {
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

    
	stream << "\"robot\":{" << robot->getJSON() << "},";
    stream << "\"dt\":" << dt << ",";
    stream << "\"totalTime\":" << totalTime << ",";
	stream << "\"gotBall\":" << (robot->dribbler->gotBall() ? "true" : "false") << ",";

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

		stream << "\"targetSide\":" << activeController->getTargetSide() << ",";
		stream << "\"playing\":" << (activeController->isPlaying() ? "true" : "false") << ",";
	} else {
		stream << "\"controllerName\": null,";
		stream << "\"controllerState\": null,";
		stream << "\"targetSide\":" << Side::UNKNOWN << ",";
		stream << "\"playing\":false,";
	}

	stream << "\"fps\":" << fpsCounter->getFps();

    stream << "}";

    return stream.str();
}
