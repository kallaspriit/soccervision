#ifndef DEBUGRENDERER_H
#define DEBUGRENDERER_H

#include "Object.h"

class ImageBuffer;

class DebugRenderer {
	public:
		static void render(unsigned char* image, const ObjectList& balls, const ObjectList& goals, bool swapRB = false);

	private:
		static void renderBalls(ImageBuffer* img, const ObjectList& balls);
		static void renderGoals(ImageBuffer* img, const ObjectList& goals);
};

#endif