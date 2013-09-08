#ifndef GUI_H
#define GUI_H

#define _WINSOCKAPI_
#include <windows.h>

#include "DisplayWindow.h"
#include "MouseListener.h"
#include <vector>

class Command;
class Vision;
class Blobber;
class ParticleFilterLocalizer;

class Gui : public MouseListener {

public:
    Gui(HINSTANCE instance, Blobber* blobberFront, Blobber* blobberRear, int width, int height);
    ~Gui();

	DisplayWindow* createWindow(int width, int height, std::string name);
    bool update();
	void addMouseListener(MouseListener* listener);
	void setFPS(int fps) { this->fps = fps; };
	void setFrontImages(unsigned char* rgb, unsigned char* yuyv, unsigned char* dataY, unsigned char* dataU, unsigned char* dataV, unsigned char* classification);
	void setRearImages(unsigned char* rgb, unsigned char* yuyv, unsigned char* dataY, unsigned char* dataU, unsigned char* dataV, unsigned char* classification);
	void onMouseMove(int x, int y, DisplayWindow* win);
	void onMouseDown(int x, int y, MouseListener::MouseBtn btn, DisplayWindow* win);
	void onMouseUp(int x, int y, MouseListener::MouseBtn btn, DisplayWindow* win);
	void onMouseWheel(int delta, DisplayWindow* win);
	void emitMouseDown(int x, int y, MouseListener::MouseBtn btn, DisplayWindow* win);
	void emitMouseUp(int x, int y, MouseListener::MouseBtn btn, DisplayWindow* win);
	void emitMouseMove(int x, int y, DisplayWindow* win);
	void emitMouseWheel(int delta, DisplayWindow* win);

private:
	HINSTANCE instance;
	MSG msg;
	std::vector<MouseListener*> mouseListeners;
	std::vector<DisplayWindow*> windows;
	DisplayWindow* frontRGB;
	DisplayWindow* rearRGB;
	DisplayWindow* frontClassification;
	DisplayWindow* rearClassification;
	Blobber* blobberFront;
	Blobber* blobberRear;
	int width;
	int height;
	int fps;
	int mouseX;
	int mouseY;
	bool mouseDown;
	int brushRadius;
};

#endif // GUI_H
