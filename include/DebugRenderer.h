#ifndef DEBUGRENDERER_H
#define DEBUGRENDERER_H

#include "Object.h"
#include "Config.h"
#include "Blobber.h"

class Canvas;

class DebugRenderer {

public:
	static void renderFPS(unsigned char* image, int fps, bool swapRB = false, int width = Config::cameraWidth, int height = Config::cameraHeight);
	static void renderBlobs(unsigned char* image, Blobber* blobber, bool swapRB = false, int width = Config::cameraWidth, int height = Config::cameraHeight);
	static void renderBalls(unsigned char* image, const ObjectList& balls, bool swapRB = false, int width = Config::cameraWidth, int height = Config::cameraHeight);
	static void renderGoals(unsigned char* image, const ObjectList& goals, bool swapRB = false, int width = Config::cameraWidth, int height = Config::cameraHeight);
	static void renderBrush(unsigned char* image, int x, int y, int radius, bool active, bool swapRB = false, int width = Config::cameraWidth, int height = Config::cameraHeight);
	static void renderObstructions(unsigned char* image, Obstruction obstruction, bool swapRB = false, int width = Config::cameraWidth, int height = Config::cameraHeight);

};

#endif // DEBUGRENDERER_H