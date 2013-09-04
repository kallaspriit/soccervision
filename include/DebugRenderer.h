#ifndef DEBUGRENDERER_H
#define DEBUGRENDERER_H

#include "Object.h"
#include "Config.h"
#include "Blobber.h"

class ImageBuffer;

class DebugRenderer {
	public:
		static void renderBlobs(unsigned char* image, Blobber* blobber, int width = Config::cameraWidth, int height = Config::cameraHeight);
		static void renderBalls(ImageBuffer* img, const ObjectList& balls);
		static void renderGoals(ImageBuffer* img, const ObjectList& goals);

	private:
		
};

#endif // DEBUGRENDERER_H