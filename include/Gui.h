#ifndef GUI_H
#define GUI_H

#define _WINSOCKAPI_
#include <windows.h>

#include "DisplayWindow.h"

class Command;
class Vision;
class SoccerBot;
class ParticleFilterLocalizer;

class Gui {
    public:
        Gui(HINSTANCE instance);
        ~Gui();

		DisplayWindow* createWindow(int width, int height, std::string name);
        bool update();

    private:
		HINSTANCE instance;
		MSG msg;
};

#endif // GUI_H
