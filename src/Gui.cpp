#include "Gui.h"
#include "DebugRenderer.h"
#include "ImageProcessor.h"
#include "Util.h"

#include <iostream>

LRESULT CALLBACK WinProc(HWND hWnd,UINT message,WPARAM wParam,LPARAM lParam);

Gui::Gui(HINSTANCE instance, Blobber* blobberFront, Blobber* blobberRear, int width, int height) : instance(instance), blobberFront(blobberFront), blobberRear(blobberRear), width(width), height(height), activeWindow(NULL) {
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

	selectedColorName = "";

	Blobber::Color* color;
	Button* button;

	for (int i = 0; i < blobberFront->getColorCount(); i++) {
		color = blobberFront->getColor(i);

		button = createButton(color->name, 20, 40 + i * 18, 160, 1);

		/*if (i == 0) {
			selectedColorName = color->name;
			button->active = true;
		}*/
	}
}

Gui::~Gui() {
	for (std::vector<DisplayWindow*>::const_iterator i = windows.begin(); i != windows.end(); i++) {
		delete *i;
	}

	windows.clear();

	for (std::vector<Element*>::const_iterator i = elements.begin(); i != elements.end(); i++) {
		delete *i;
	}

	elements.clear();
}

DisplayWindow* Gui::createWindow(int width, int height, std::string name) {
	DisplayWindow* window = new DisplayWindow(instance, width, height, name, this);

	windows.push_back(window);

	return window;
}

Gui::Button* Gui::createButton(std::string text, int x, int y, int width, int type, void* data) {
	Button* button = new Button(text, x, y, width, type, data);

	addMouseListener(button);

	elements.push_back(button);

	return button;
}

void Gui::drawElements(unsigned char* image, int width, int height, bool swapRB) {
	for (std::vector<Element*>::const_iterator i = elements.begin(); i != elements.end(); i++) {
		(*i)->draw(image, width, height, swapRB);
	}
}

bool Gui::isMouseOverElement(int x, int y) {
	for (std::vector<Element*>::const_iterator i = elements.begin(); i != elements.end(); i++) {
		if ((*i)->contains(x, y)) {
			return true;
		}
	}

	return false;
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

	drawElements(rgb, width, height, true);
	drawElements(classification, width, height, false);

	if (activeWindow == frontClassification || activeWindow == frontRGB) {
		if (!isMouseOverElement(mouseX, mouseY)) {
			if (selectedColorName.length() > 0) {
				handleColorThresholding(dataY, dataU, dataV, rgb, classification);
			}
		} else {
			handleElements();
		}
	}
	
	frontRGB->setImage(rgb, false);
	frontClassification->setImage(classification, true);
}

void Gui::setRearImages(unsigned char* rgb, unsigned char* yuyv, unsigned char* dataY, unsigned char* dataU, unsigned char* dataV, unsigned char* classification) {
	DebugRenderer::renderFPS(rgb, fps, true);

	drawElements(rgb, width, height, true);
	drawElements(classification, width, height, false);

	if (activeWindow == rearClassification || activeWindow == rearRGB) {
		if (!isMouseOverElement(mouseX, mouseY)) {
			if (selectedColorName.length() > 0) {
				handleColorThresholding(dataY, dataU, dataV, rgb, classification);
			}
		} else {
			handleElements();
		}
	}

	rearRGB->setImage(rgb, false);
	rearClassification->setImage(classification, true);
}

void Gui::handleColorThresholding(unsigned char* dataY, unsigned char* dataU, unsigned char* dataV, unsigned char* rgb, unsigned char* classification) {
	DebugRenderer::renderBrush(rgb, mouseX, mouseY, brushRadius, mouseDown, true);
	DebugRenderer::renderBrush(classification, mouseX, mouseY, brushRadius, mouseDown, false);

	if (mouseDown) {
		float stdDev = 2.0f;

		ImageProcessor::YUYVRange yuyvRange = ImageProcessor::extractColorRange(dataY, dataU, dataV, width, height, mouseX, mouseY, brushRadius, stdDev);
	
		if (mouseBtn == MouseListener::MouseBtn::LEFT) {
			blobberFront->getColor(selectedColorName)->addThreshold(
				yuyvRange.minY, yuyvRange.maxY,
				yuyvRange.minU, yuyvRange.maxU,
				yuyvRange.minV, yuyvRange.maxV
			);
			blobberRear->getColor(selectedColorName)->addThreshold(
				yuyvRange.minY, yuyvRange.maxY,
				yuyvRange.minU, yuyvRange.maxU,
				yuyvRange.minV, yuyvRange.maxV
			);
		} else if (mouseBtn == MouseListener::MouseBtn::RIGHT) {
			blobberFront->getColor(selectedColorName)->substractThreshold(
				yuyvRange.minY, yuyvRange.maxY,
				yuyvRange.minU, yuyvRange.maxU,
				yuyvRange.minV, yuyvRange.maxV
			);
			blobberRear->getColor(selectedColorName)->substractThreshold(
				yuyvRange.minY, yuyvRange.maxY,
				yuyvRange.minU, yuyvRange.maxU,
				yuyvRange.minV, yuyvRange.maxV
			);
		} else if (mouseBtn == MouseListener::MouseBtn::MIDDLE) {
			blobberFront->clearColor(selectedColorName);
			blobberRear->clearColor(selectedColorName);
		}
	}
}

void Gui::handleElements() {
	if (!mouseDown) {
		return;
	}

	Element* element;

	for (std::vector<Element*>::const_iterator i = elements.begin(); i != elements.end(); i++) {
		element = *i;

		if (element->contains(mouseX, mouseY)) {
			onElementClick(element);
		}
	}
}

void Gui::onElementClick(Element* element) {
	Button* button = dynamic_cast<Button*>(element);
	bool unset = false;

	if (button != NULL) {
		if (Util::duration(button->lastInteractionTime) < 0.2) {
			return;
		}

		//std::cout << "! Button '" << button->text << "' clicked" << std::endl;

		if (button->type == 1) {
			if (button->text != selectedColorName) {
				selectedColorName = button->text;
			} else {
				selectedColorName = "";
				unset = true;
			}
		}

		button->lastInteractionTime = Util::millitime();

		Element* el;
		Button* btn;

		for (std::vector<Element*>::const_iterator i = elements.begin(); i != elements.end(); i++) {
			el = *i;
			btn = dynamic_cast<Button*>(el);

			if (btn == NULL) {
				continue;
			}

			if (btn == button) {
				btn->active = unset ? false : true;
			} else {
				btn->active = false;
			}
		}
	}
}

void Gui::onMouseMove(int x, int y, DisplayWindow* win) {
	mouseX = x;
	mouseY = y;

	activeWindow = win;
}

void Gui::onMouseDown(int x, int y, MouseListener::MouseBtn btn, DisplayWindow* win) {
	mouseDown = true;
	mouseBtn = btn;

	activeWindow = win;
}

void Gui::onMouseUp(int x, int y, MouseListener::MouseBtn btn, DisplayWindow* win) {
	mouseDown = false;
	mouseBtn = btn;

	activeWindow = win;
}

void Gui::onMouseWheel(int delta, DisplayWindow* win) {
	brushRadius += delta / 120 * 5;

	if (brushRadius < 5) {
		brushRadius = 5;
	}

	activeWindow = win;
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

Gui::Element::Element() : lastInteractionTime(0) {

}

Gui::Button::Button(std::string text, int x, int y, int width, int type, void* data) : Element(), text(text), x(x), y(y), width(width), type(type), data(data), over(false), active(false) {

}

void Gui::Button::draw(unsigned char* image, int imageWidth, int imageHeight, bool swapRB) {
	canvas.width = imageWidth;
	canvas.height = imageHeight;
	canvas.data = image;
	canvas.swapRB = swapRB;

	if (active) {
		canvas.fillBox(x, y, getWidth(), getHeight(), 255, 0, 0);
		canvas.drawBox(x, y, getWidth(), getHeight(), 255, over ? 0 : 255, over ? 0 : 255);
		canvas.drawText(x + 6, y + 4, text, 255, 255, 255);
	} else {
		canvas.drawBox(x, y, getWidth(), getHeight(), 255, over ? 0 : 255, over ? 0 : 255);
		canvas.drawText(x + 6, y + 4, text, 255, over ? 0 : 255, over ? 0 : 255);
	}
}

int Gui::Button::getWidth() {
	if (width != 0) {
		return width;
	} else {
		return text.length() * 9 + 6 * 2;
	}
}

int Gui::Button::getHeight() {
	return 16;
}

void Gui::Button::onMouseMove(int x, int y, DisplayWindow* win) {
	over = contains(x, y);
}

bool Gui::Button::contains(int px, int py) {
	return px >= x
		&& px <= x + getWidth()
		&& py >= y
		&& py <= y + getHeight();
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
