#include "Gui.h"
#include "XimeaCamera.h"
#include "ProcessThread.h"
#include "Blobber.h"
#include "Vision.h"
#include "FpsCounter.h"
#include "Util.h"
#include "Config.h"

#include <iostream>

void setupCamera(XimeaCamera* camera) {
	camera->setGain(6);
	camera->setExposure(10000);
	camera->setFormat(XI_RAW8);
	camera->setAutoWhiteBalance(false);
	camera->setAutoExposureGain(false);
	camera->setQueueSize(12); // affects anything?

	std::cout << "Camera info:" << std::endl;
	std::cout << "  > Name: " << camera->getName() << std::endl;
	std::cout << "  > Type: " << camera->getDeviceType() << std::endl;
	std::cout << "  > API version: " << camera->getApiVersion() << std::endl;
	std::cout << "  > Driver version: " << camera->getDriverVersion() << std::endl;
	std::cout << "  > Serial number: " << camera->getSerialNumber() << std::endl;
	std::cout << "  > Color: " << (camera->supportsColor() ? "yes" : "no") << std::endl;
	std::cout << "  > Framerate: " << camera->getFramerate() << std::endl;
	std::cout << "  > Available bandwidth: " << camera->getAvailableBandwidth() << std::endl;

	camera->startAcquisition();
}

/*void processCamera(XimeaCamera& camera, Blobber* blobber) {

}*/

int main(int argc, char* argv[]) {
	HWND consoleWindow = GetConsoleWindow();
	HINSTANCE instance = GetModuleHandle(0);

    std::cout << "-- Starting Up --" << std::endl;

	// config
	int width = Config::cameraWidth;
	int height = Config::cameraHeight;
	bool debug = true;

	Gui* gui = new Gui(instance);
	FpsCounter* fpsCounter = new FpsCounter();
	DisplayWindow* winRGB1 = gui->createWindow(1280, 1024, "Camera 1 RGB");
	DisplayWindow* winRGB2 = gui->createWindow(1280, 1024, "Camera 2 RGB");
	DisplayWindow* winClassification1 = gui->createWindow(1280, 1024, "Camera 1 Classification");
	DisplayWindow* winClassification2 = gui->createWindow(1280, 1024, "Camera 2 Classification");

	XimeaCamera* camera1 = new XimeaCamera();
	XimeaCamera* camera2 = new XimeaCamera();

	Blobber* blobber1 = new Blobber();
	blobber1->initialize(width, height);
	blobber1->loadOptions(Config::blobberConfigFilename);
	Vision* vision1 = new Vision(blobber1, Dir::FRONT, width, height);

	Blobber* blobber2 = new Blobber();
	blobber2->initialize(width, height);
	blobber2->loadOptions(Config::blobberConfigFilename);
	Vision* vision2 = new Vision(blobber2, Dir::REAR, width, height);

	ProcessThread* processor1 = new ProcessThread(blobber1, vision1);
	ProcessThread* processor2 = new ProcessThread(blobber2, vision2);

	Vision::Results* visionResults = new Vision::Results();

	int cameraSerial1 = 857735761;
	int cameraSerial2 = 857769553;

	if (!camera1->open(cameraSerial2) && !camera1->open(cameraSerial1)) {
		std::cout << "- Opening camera 1 failed" << std::endl;
	}

	if (!camera2->open(cameraSerial1) && !camera2->open(cameraSerial2)) {
		std::cout << "- Opening camera 2 failed" << std::endl;
	}

	if (!camera1->isOpened() && !camera2->isOpened()) {
		std::cout << "- Opening both cameras failed, giving up" << std::endl;

		return 1;
	}

	setupCamera(camera1);
	//setupCamera(camera2);

	std::cout << "! Capturing frames" << std::endl;

	const BaseCamera::Frame* frame = NULL;

	bool running = true;
	bool gotFrame1, gotFrame2;

	//for (int i = 0; i < 60 * 10; i++) {
	while (running) {
		gotFrame1 = false;
		gotFrame2 = false;

		processor1->debug = processor2->debug = debug;

		//__int64 startTime = Util::timerStart();

		// camera1
		if (camera1->isAcquisitioning()) {

			// get the frame
			//Util::timerStart();
			frame = camera1->getFrame();
			//double getFrameTime = Util::timerEnd();

			// check if there's a new frame
			if (frame != NULL && frame->fresh) {
				gotFrame1 = true;
				processor1->setFrame(frame->data);
			} else {
				std::cout << "- Got empty/old frame from camera 1" << std::endl;
			}
		}

		if (camera2->isAcquisitioning()) {
			frame = camera2->getFrame();

			if (frame != NULL && frame->fresh) {
				gotFrame2 = true;

				processor2->setFrame(frame->data);
			} else {
				std::cout << "- Got empty/old frame from camera 2" << std::endl;
			}
		}

		processor1->start();
		processor2->start();

		processor1->join();
		processor2->join();

		visionResults->front = processor1->visionResult;
		visionResults->rear = processor2->visionResult;

		// TODO Visualize the vision results

		if (debug) {
			if (gotFrame1) {
				winRGB1->setImage(processor1->rgb, false);
				winClassification1->setImage(processor1->classification, true);
			}

			if (gotFrame2) {
				winRGB2->setImage(processor2->rgb, false);
				winClassification2->setImage(processor2->classification, true);
			}

			gui->update();
		}

		//std::cout << "! Total time: " << Util::timerEnd(startTime) << ", " << fpsCounter->getFps() << "FPS " << (gotFrame1 ? '+' : '-') << (gotFrame2 ? '+' : '-') << std::endl;
		std::cout << "! " << fpsCounter->getFps() << "FPS " << (gotFrame1 ? '+' : '-') << (gotFrame2 ? '+' : '-') << std::endl;

		fpsCounter->step();
	}

	blobber1->saveOptions(Config::blobberConfigFilename);
	//blobber2->saveOptions(Config::blobberConfigFilename);

	delete blobber1;
	delete blobber2;
	delete vision1;
	delete vision2;
	delete camera1;
	delete camera2;
	delete winRGB1;
	delete winRGB2;
	delete fpsCounter;
	delete gui;

    std::cout << "-- Properly Terminated --" << std::endl;

    return 0;
}
