#include "Gui.h"
#include "DebugRenderer.h"
#include "ImageProcessor.h"

#include <iostream>

LRESULT CALLBACK WinProc(HWND hWnd,UINT message,WPARAM wParam,LPARAM lParam);

Gui::Gui(HINSTANCE instance, Blobber* blobberFront, Blobber* blobberRear, int width, int height) : instance(instance), blobberFront(blobberFront), blobberRear(blobberRear), width(width), height(height) {
	WNDCLASSEX wClass;
	ZeroMemory(&wClass, sizeof(WNDCLASSEX));

	wClass.cbClsExtra = NULL;
	wClass.cbSize = sizeof(WNDCLASSEX);
	wClass.cbWndExtra = NULL;
	wClass.hbrBackground = (HBRUSH)COLOR_WINDOW;
	wClass.hCursor = LoadCursor(NULL, IDC_ARROW);
	wClass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
	wClass.hIconSm = LoadIcon(NULL, IDI_APPLICATION);
	wClass.hInstance = instance;
	wClass.lpfnWndProc = (WNDPROC)WinProc;
	wClass.lpszClassName = "Window Class";
	wClass.lpszMenuName = NULL;
	wClass.style = CS_HREDRAW | CS_VREDRAW;

	if (!RegisterClassEx(&wClass)) {
		int nResult = GetLastError();

		MessageBox(
			NULL,
			"Window class creation failed",
			"Window Class Failed",
			MB_ICONERROR
		);
	}

	ZeroMemory(&msg, sizeof(MSG));

	addMouseListener(this);

	mouseX = 0;
	mouseY = 0;
	mouseDown = false;
	mouseBtn = MouseListener::MouseBtn::LEFT;
	brushRadius = 50;

	frontRGB = createWindow(width, height, "Camera 1 RGB");
	rearRGB = createWindow(width, height, "Camera 2 RGB");
	frontClassification = createWindow(width, height, "Camera 1 classification");
	rearClassification = createWindow(width, height, "Camera 2 classification");
}

Gui::~Gui() {
	for (std::vector<DisplayWindow*>::const_iterator i = windows.begin(); i != windows.end(); i++) {
		delete *i;
	}

	windows.clear();
}

DisplayWindow* Gui::createWindow(int width, int height, std::string name) {
	DisplayWindow* window = new DisplayWindow(instance, width, height, name, this);

	windows.push_back(window);

	return window;
}

bool Gui::update() {
	while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE) != 0) {
		TranslateMessage(&msg);
		DispatchMessage(&msg);

		if (msg.message == WM_QUIT) {
			return false;
		}
	}

	return true;
}

void Gui::addMouseListener(MouseListener* listener) {
	mouseListeners.push_back(listener);
}

void Gui::setFrontImages(unsigned char* rgb, unsigned char* yuyv, unsigned char* dataY, unsigned char* dataU, unsigned char* dataV, unsigned char* classification) {
	DebugRenderer::renderFPS(rgb, fps, true);

	handleColorThresholding(dataY, dataU, dataV, rgb, classification);
	
	frontRGB->setImage(rgb, false);
	frontClassification->setImage(classification, true);
}

void Gui::setRearImages(unsigned char* rgb, unsigned char* yuyv, unsigned char* dataY, unsigned char* dataU, unsigned char* dataV, unsigned char* classification) {
	DebugRenderer::renderFPS(rgb, fps, true);

	handleColorThresholding(dataY, dataU, dataV, rgb, classification);

	rearRGB->setImage(rgb, false);
	rearClassification->setImage(classification, true);
}

void Gui::handleColorThresholding(unsigned char* dataY, unsigned char* dataU, unsigned char* dataV, unsigned char* rgb, unsigned char* classification) {
	DebugRenderer::renderBrush(rgb, mouseX, mouseY, brushRadius, mouseDown, true);
	DebugRenderer::renderBrush(classification, mouseX, mouseY, brushRadius, mouseDown, false);

	if (mouseDown) {
		float stdDev = 2.0f;
		std::string color = "green";

		ImageProcessor::YUYVRange yuyvRange = ImageProcessor::extractColorRange(dataY, dataU, dataV, width, height, mouseX, mouseY, brushRadius, stdDev);
	
		if (mouseBtn == MouseListener::MouseBtn::LEFT) {
			blobberFront->getColor(color)->addThreshold(
				yuyvRange.minY, yuyvRange.maxY,
				yuyvRange.minU, yuyvRange.maxU,
				yuyvRange.minV, yuyvRange.maxV
			);
		} else if (mouseBtn == MouseListener::MouseBtn::RIGHT) {
			blobberFront->getColor(color)->substractThreshold(
				yuyvRange.minY, yuyvRange.maxY,
				yuyvRange.minU, yuyvRange.maxU,
				yuyvRange.minV, yuyvRange.maxV
			);
		} else if (mouseBtn == MouseListener::MouseBtn::MIDDLE) {
			blobberFront->clearColor(color);
		}

		std::cout << "! Range: " << yuyvRange.minY << "-" << yuyvRange.maxY << " " << yuyvRange.minU << "-" << yuyvRange.maxU << " " << yuyvRange.minV << "-" << yuyvRange.maxV << std::endl;
	}
}

void Gui::onMouseMove(int x, int y, DisplayWindow* win) {
	mouseX = x;
	mouseY = y;
}

void Gui::onMouseDown(int x, int y, MouseListener::MouseBtn btn, DisplayWindow* win) {
	mouseDown = true;
	mouseBtn = btn;
}

void Gui::onMouseUp(int x, int y, MouseListener::MouseBtn btn, DisplayWindow* win) {
	mouseDown = false;
	mouseBtn = btn;
}

void Gui::onMouseWheel(int delta, DisplayWindow* win) {
	brushRadius += delta / 120 * 5;

	if (brushRadius < 5) {
		brushRadius = 5;
	}
}

void Gui::emitMouseDown(int x, int y, MouseListener::MouseBtn btn, DisplayWindow* win) {
	for (std::vector<MouseListener*>::const_iterator i = mouseListeners.begin(); i != mouseListeners.end(); i++) {
		(*i)->onMouseDown(x, y, btn, win);
	}
}

void Gui::emitMouseUp(int x, int y, MouseListener::MouseBtn btn, DisplayWindow* win) {
	for (std::vector<MouseListener*>::const_iterator i = mouseListeners.begin(); i != mouseListeners.end(); i++) {
		(*i)->onMouseUp(x, y, btn, win);
	}
}

void Gui::emitMouseMove(int x, int y, DisplayWindow* win) {
	for (std::vector<MouseListener*>::const_iterator i = mouseListeners.begin(); i != mouseListeners.end(); i++) {
		(*i)->onMouseMove(x, y, win);
	}
}

void Gui::emitMouseWheel(int delta, DisplayWindow* win) {
	for (std::vector<MouseListener*>::const_iterator i = mouseListeners.begin(); i != mouseListeners.end(); i++) {
		(*i)->onMouseWheel(delta, win);
	}
}

LRESULT CALLBACK WinProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
	switch(msg) {
		case WM_CREATE:
			SetWindowLong(hWnd, GWL_USERDATA, LONG(LPCREATESTRUCT(lParam)->lpCreateParams));
		break;

		case WM_DESTROY:
			PostQuitMessage(0);
			printf("Destroy\n");

			return 0;
		break;

		default:
			DisplayWindow* displayWindow = (DisplayWindow*)GetWindowLong(hWnd, GWL_USERDATA);

			if (displayWindow != NULL) {
				return displayWindow->handleMessage(hWnd, msg, wParam, lParam);
			} else {
				return DefWindowProc(hWnd, msg, wParam, lParam);
			}
		break;
	}

	return 0;
}