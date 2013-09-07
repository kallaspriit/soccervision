#ifndef GUI_H
#define GUI_H

#define _WINSOCKAPI_
#include <windows.h>

#include "DisplayWindow.h"
#include <vector>

class Command;
class Vision;
class SoccerBot;
class ParticleFilterLocalizer;

class Gui {

public:
	class MouseListener {
	public:
		void onMouseMove(int x, int y);
		void onMouseClick(int x, int y);
	};

    Gui(HINSTANCE instance);
    ~Gui();

	DisplayWindow* createWindow(int width, int height, std::string name);
    bool update();
	void addMouseListener(MouseListener* listener);

private:
	HINSTANCE instance;
	MSG msg;
	std::vector<MouseListener*> mouseListeners;
};

#endif // GUI_H
