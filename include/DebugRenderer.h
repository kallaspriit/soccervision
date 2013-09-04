#ifndef DEBUGRENDERER_H
#define DEBUGRENDERER_H

#include "Object.h"
#include "Blobber.h"

class ImageBuffer;

class DebugRenderer {
	public:
		static void renderBlobs(ImageBuffer* img, Blobber::Blob* blobs, Blobber::Color color);
		static void renderBalls(ImageBuffer* img, const ObjectList& balls);
		static void renderGoals(ImageBuffer* img, const ObjectList& goals);

	private:
		
};

#endif // DEBUGRENDERER_H