#ifndef GUI_H
#define GUI_H

#define _WINSOCKAPI_
#include <windows.h>

#include "DisplayWindow.h"
#include "MouseListener.h"
#include <vector>

class Command;
class Vision;
class SoccerBot;
class ParticleFilterLocalizer;

class Gui : public MouseListener {

public:
    Gui(HINSTANCE instance);
    ~Gui();

	DisplayWindow* createWindow(int width, int height, std::string name);
    bool update();
	void addMouseListener(MouseListener* listener);
	void onMouseMove(int x, int y, DisplayWindow* win);
	void onMouseClick(int x, int y, DisplayWindow* win);
	void emitMouseClick(int x, int y, DisplayWindow* win);
	void emitMouseMove(int x, int y, DisplayWindow* win);

private:
	HINSTANCE instance;
	MSG msg;
	std::vector<MouseListener*> mouseListeners;
};

#endif // GUI_H
