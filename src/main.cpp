#include "Gui.h"
#include "XimeaCamera.h"
#include "FpsCounter.h"

#include <iostream>

int main(int argc, char* argv[]) {
	HWND consoleWindow = GetConsoleWindow();
	HINSTANCE instance = GetModuleHandle(0);

    std::cout << "-- Starting Up --" << std::endl;

	Gui gui = Gui(instance);
	FpsCounter fpsCounter = FpsCounter();
	DisplayWindow* cameraWindow = gui.createWindow(1280, 1024, "Camera RGB");

	XimeaCamera camera = XimeaCamera();

	int cameraSerial1 = 857735761;
	int cameraSerial2 = 857769553;

	if (!camera.open(cameraSerial1) && !camera.open(cameraSerial2)) {
		std::cout << "- Opening camera failed" << std::endl;

		return 1;
	}

	camera.setExposure(16000);
	camera.setFormat(XI_RAW8);
	camera.setAutoWhiteBalance(false);
	camera.setAutoExposureGain(false);

	camera.startAcquisition();

	std::cout << "! Capturing frames" << std::endl;

	for (int i = 0; i < 60 * 10; i++) {
		const BaseCamera::Frame* frame = camera.getFrame();

		if (frame == NULL) {
			std::cout << "  > failed getting frame" << std::endl;

			continue;
		}

		if (!frame->fresh) {
			std::cout << "  > got old frame" << std::endl;

			continue;
		}

		//cameraWindow->setImage(frame->data);
		//gui.update();

		fpsCounter.step();

		std::cout << "  > frame #" << frame->number << " @ " << frame->width << "x" << frame->height << ", " << fpsCounter.getFps() << "FPS" << (!frame->fresh ? " (not fresh)" : "") << std::endl;
	}

	delete cameraWindow;

	//camera.open(857735761);

    std::cout << "-- Properly Terminated --" << std::endl;

    return 0;
}
