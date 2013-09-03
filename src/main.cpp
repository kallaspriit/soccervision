#include "Gui.h"
#include "XimeaCamera.h"
#include "ImageProcessor.h"
#include "FpsCounter.h"
#include "Util.h"
#include <libyuv.h>

#include <iostream>

void setupCamera(XimeaCamera& camera) {
	camera.setExposure(16000);
	camera.setFormat(XI_RAW8);
	camera.setAutoWhiteBalance(false);
	camera.setAutoExposureGain(false);
	camera.setQueueSize(12);

	std::cout << "Camera info:" << std::endl;
	std::cout << "  > Name: " << camera.getName() << std::endl;
	std::cout << "  > Type: " << camera.getDeviceType() << std::endl;
	std::cout << "  > API version: " << camera.getApiVersion() << std::endl;
	std::cout << "  > Driver version: " << camera.getDriverVersion() << std::endl;
	std::cout << "  > Serial number: " << camera.getSerialNumber() << std::endl;
	std::cout << "  > Color: " << (camera.supportsColor() ? "yes" : "no") << std::endl;
	std::cout << "  > Framerate: " << camera.getFramerate() << std::endl;
	std::cout << "  > Available bandwidth: " << camera.getAvailableBandwidth() << std::endl;

	camera.startAcquisition();
}

int main(int argc, char* argv[]) {
	HWND consoleWindow = GetConsoleWindow();
	HINSTANCE instance = GetModuleHandle(0);

    std::cout << "-- Starting Up --" << std::endl;

	Gui gui = Gui(instance);
	FpsCounter fpsCounter = FpsCounter();
	DisplayWindow* cameraWindow1 = gui.createWindow(1280, 1024, "Camera 1 RGB");
	DisplayWindow* cameraWindow2 = gui.createWindow(1280, 1024, "Camera 2 RGB");

	XimeaCamera camera1 = XimeaCamera();
	XimeaCamera camera2 = XimeaCamera();

	int cameraSerial1 = 857735761;
	int cameraSerial2 = 857769553;

	if (!camera1.open(cameraSerial2) && !camera1.open(cameraSerial1)) {
		std::cout << "- Opening camera 1 failed" << std::endl;
	}

	if (!camera2.open(cameraSerial1) && !camera2.open(cameraSerial2)) {
		std::cout << "- Opening camera 2 failed" << std::endl;
	}

	if (!camera1.isOpened() && !camera2.isOpened()) {
		std::cout << "- Opening both cameras failed, giving up" << std::endl;

		return 1;
	}

	setupCamera(camera1);
	setupCamera(camera2);

	std::cout << "! Capturing frames" << std::endl;

	int width = 1280;
	int height = 1024;
	int strideY = width;
	int strideU = (width + 1) / 2;
	int strideV = (width + 1) / 2;

	unsigned char* argbBuffer = new unsigned char[width * height * 4];
	unsigned char* rgbBuffer = new unsigned char[width * height * 3];

	unsigned char* dataY = new uint8[width * height];
    unsigned char* dataU = new uint8[(width / 2) * (height / 2)];
    unsigned char* dataV = new uint8[(width / 2) * (height / 2)];
	unsigned char* dataYUYV = new uint8[width * height * 3];

	const BaseCamera::Frame* frame = NULL;

	//for (int i = 0; i < 60 * 10; i++) {
	bool running = true;

	while (running) {
		// camera1
		if (camera1.isAcquisitioning()) {
			Util::timerStart();
			frame = camera1.getFrame();
			std::cout << "@ Get frame: " << Util::timerEnd() << std::endl;

			if (frame == NULL) {
				std::cout << "  > failed getting camera 1 frame" << std::endl;

				continue;
			}

			if (!frame->fresh) {
				std::cout << "  > got old camera 1 frame" << std::endl;

				continue;
			}

			Util::timerStart();

			ImageProcessor::bayerRGGBToI420(
				frame->data,
				dataY, dataU, dataV,
				frame->width, frame->height
			);

			/*libyuv::BayerRGGBToI420(
				frame->data,
				frame->width,
				dataY,
				strideY,
				dataU,
				strideU,
				dataV,
				strideV,
				frame->width,
				frame->height
			);*/

			std::cout << "@ RGGB > I420: " << Util::timerEnd() << std::endl;
			Util::timerStart();

			ImageProcessor::I420ToYUYV(
				dataY, dataU, dataV,
				dataYUYV,
				frame->width, frame->height
			);

			/*int dstIndex = 0;
			int yIndex = 0;
			int dstStride = frame->width * 2;
			int yStride = frame->width;
			int uvStride = frame->width / 2;
			int row = 0;
			int pixelsInRow = 0;
			int elementCount = (frame->width / 2) * (frame->height / 2);

			for (int uvIndex = 0; uvIndex < elementCount; uvIndex++) {
				dataYUYV[dstIndex]     = dataY[yIndex];
				dataYUYV[dstIndex + 1] = dataU[uvIndex];
				dataYUYV[dstIndex + 2] = dataY[yIndex + 1];
				dataYUYV[dstIndex + 3] = dataV[uvIndex];

				dataYUYV[dstIndex + dstStride]     = dataY[yIndex + yStride];
				dataYUYV[dstIndex + dstStride + 1] = dataU[uvIndex];
				dataYUYV[dstIndex + dstStride + 2] = dataY[yIndex + yStride + 1];
				dataYUYV[dstIndex + dstStride + 3] = dataV[uvIndex];

				dstIndex += 4;
				yIndex += 2;
				pixelsInRow += 2;

				if (pixelsInRow == frame->width) {
					row += 2;
					yIndex = row * yStride;
					dstIndex = row * dstStride;
					pixelsInRow = 0;
				}
			}*/

			std::cout << "@ I420 > YUYV: " << Util::timerEnd() << std::endl;

			Util::timerStart();
			ImageProcessor::YUYVToARGB(dataYUYV, argbBuffer, frame->width, frame->height);
			std::cout << "@ YUYV > ARGB: " << Util::timerEnd() << std::endl;

			/*Util::timerStart();

			libyuv::BayerRGGBToARGB(
				frame->data,
				frame->width,
				argbBuffer,
				frame->width * 4,
				frame->width,
				frame->height
			);

			std::cout << "@ RGGB > ARGB: " << Util::timerEnd() << std::endl;*/
			Util::timerStart();

			ImageProcessor::ARGBToRGB24(
				argbBuffer,
				rgbBuffer,
				frame->width, frame->height
			);

			std::cout << "@ ARGB > RGB: " << Util::timerEnd() << std::endl;
			Util::timerStart();

			cameraWindow1->setImage(rgbBuffer, false);

			std::cout << "@ Display: " << Util::timerEnd() << std::endl;

			std::cout << "  > camera 1 frame #" << frame->number << " @ " << frame->width << "x" << frame->height << ", " << fpsCounter.getFps() << "FPS" << (!frame->fresh ? " (not fresh)" : "") << std::endl;
		}

		// camera2
		/*if (camera2.isAcquisitioning()) {
			frame = camera2.getFrame();

			if (frame == NULL) {
				std::cout << "  > failed getting camera 2 frame" << std::endl;

				continue;
			}

			if (!frame->fresh) {
				std::cout << "  > got old camera 2 frame" << std::endl;

				continue;
			}

			libyuv::BayerRGGBToARGB(
				frame->data,
				frame->width,
				argbBuffer,
				frame->width * 4,
				frame->width,
				frame->height
			);

			libyuv::ARGBToRGB24(
				argbBuffer, frame->width * 4,
				rgbBuffer, frame->width * 3,
				frame->width, frame->height
			);

			//cameraWindow2->setImage(rgbBuffer, false);

			std::cout << "  > camera 2 frame #" << frame->number << " @ " << frame->width << "x" << frame->height << ", " << fpsCounter.getFps() << "FPS" << (!frame->fresh ? " (not fresh)" : "") << std::endl;
		}*/


		gui.update();

		fpsCounter.step();
	}

	delete cameraWindow1;
	delete cameraWindow2;

	//camera.open(857735761);

    std::cout << "-- Properly Terminated --" << std::endl;

    return 0;
}
