#ifndef DEBUGRENDERER_H
#define DEBUGRENDERER_H

#include "Object.h"
#include "Config.h"
#include "Blobber.h"

class ImageBuffer;

class DebugRenderer {
	public:
		static void renderFPS(unsigned char* image, int fps, bool swapRB = false, int width = Config::cameraWidth, int height = Config::cameraHeight);
		static void renderBlobs(unsigned char* image, Blobber* blobber, bool swapRB = false, int width = Config::cameraWidth, int height = Config::cameraHeight);
		static void renderBalls(unsigned char* image, const ObjectList& balls, bool swapRB = false, int width = Config::cameraWidth, int height = Config::cameraHeight);
		static void renderGoals(unsigned char* image, const ObjectList& goals, bool swapRB = false, int width = Config::cameraWidth, int height = Config::cameraHeight);

	private:
		
};

#endif // DEBUGRENDERER_H