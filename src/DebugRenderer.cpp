#include "DebugRenderer.h"
#include "ImageBuffer.h"
#include "Maths.h"
#include "Util.h"

void DebugRenderer::renderFPS(unsigned char* image, int fps, bool swapRB, int width, int height) {
	ImageBuffer img = ImageBuffer();

	img.data = image;
	img.width = width;
	img.height = height;
	img.swapRB = swapRB;

	img.drawText(20, 20, "FPS: " + Util::toString(fps));
}

void DebugRenderer::renderBlobs(unsigned char* image, Blobber* blobber, bool swapRB, int width, int height) {
	ImageBuffer img = ImageBuffer();

	img.data = image;
	img.width = width;
	img.height = height;
	img.swapRB = swapRB;

	for (int i = 0; i < blobber->getColorCount(); i++) {
		Blobber::Color* color = blobber->getColor(i);

		if (color == NULL) {
			continue;
		}

		if (
			strcmp(color->name, "ball") != 0
			&& strcmp(color->name, "yellow-goal") != 0
			&& strcmp(color->name, "blue-goal") != 0
		) {
			continue;
		}

		Blobber::Blob* blob = blobber->getBlobs(color->name);

		while (blob != NULL) {
			img.drawBoxCentered(
				(int)blob->centerX, (int)blob->centerY,
				blob->x2 - blob->x1, blob->y2 - blob->y1,
				color->color.red, color->color.green, color->color.blue
			);

			blob = blob->next;
		}
	}
}

void DebugRenderer::renderBalls(unsigned char* image, const ObjectList& balls, bool swapRB, int width, int height) {
	ImageBuffer img = ImageBuffer();

	img.data = image;
	img.width = width;
	img.height = height;
	img.swapRB = swapRB;

	Object* ball = NULL;
    char buf[256];
	int correctedX, correctedY;

    for (ObjectListItc it = balls.begin(); it != balls.end(); it++) {
        ball = *it;

        img.drawBoxCentered(ball->x, ball->y, ball->width, ball->height);
		img.drawLine(ball->x - ball->width / 2, ball->y - ball->height / 2, ball->x + ball->width / 2, ball->y + ball->height / 2);
        img.drawLine(ball->x - ball->width / 2, ball->y + ball->height / 2, ball->x + ball->width / 2, ball->y - ball->height / 2);

		sprintf(buf, "%.2fm  %.1f deg", ball->distance, Math::radToDeg(ball->angle));
        img.drawText(ball->x - ball->width / 2 + 2, ball->y - ball->height / 2 - 19, buf);

		correctedX = ball->x;
		correctedY = ball->y;

		Util::correctCameraPoint(correctedX, correctedY);

		sprintf(buf, "%d x %d", correctedX, correctedY + ball->height / 2);
        img.drawText(ball->x - ball->width / 2 + 2, ball->y - ball->height / 2 - 9, buf);

        int boxArea = ball->width * ball->height;

		/*if (boxArea == 0) {
			continue;
		}

        int density = ball->area * 100 / boxArea;

        sprintf(buf, "%d - %d%%", ball->area, density);
        img.drawText(ball->x - ball->width / 2 + 2, ball->y - ball->height / 2 - 9, buf);*/
    }

	// TEMP - draw centerline
	img.drawLine(img.width / 2, 0, img.width / 2, img.height);
	//img.fillCircleCentered(Config::cameraWidth / 2, Config::cameraHeight / 2, 100, 0, 0, 255);

    /*Blobber::Blob* blob = blobber->getBlobs("ball");

    while (blob != NULL) {
        image->drawBoxCentered(blob->centerX, blob->centerY, blob->x2 - blob->x1, blob->y2 - blob->y1);

        blob = blob->next;
    }*/
}

void DebugRenderer::renderGoals(unsigned char* image, const ObjectList& goals, bool swapRB, int width, int height) {
	ImageBuffer img = ImageBuffer();

	img.data = image;
	img.width = width;
	img.height = height;
	img.swapRB = swapRB;

	Object* goal = NULL;
    char buf[256];
	int r, g, b;

    for (ObjectListItc it = goals.begin(); it != goals.end(); it++) {
        goal = *it;

		if (goal->type == Side::YELLOW) {
			r = 200;
			g = 200;
			b = 0;
		} else {
			r = 0;
			g = 0;
			b = 200;
		}

        img.drawBoxCentered(goal->x, goal->y, goal->width, goal->height, r, g, b);
		img.drawLine(goal->x - goal->width / 2, goal->y - goal->height / 2, goal->x + goal->width / 2, goal->y + goal->height / 2, r, g, b);
        img.drawLine(goal->x - goal->width / 2, goal->y + goal->height / 2, goal->x + goal->width / 2, goal->y - goal->height / 2, r, g, b);

        sprintf(buf, "%.2fm %.1f deg", goal->distance, Math::radToDeg(goal->angle));
        img.drawText(goal->x - goal->width / 2 + 2, goal->y + goal->height / 2 + 2, buf);

		sprintf(buf, "%d x %d, %d", goal->x, goal->y + goal->height / 2, goal->area);
        img.drawText(goal->x - goal->width / 2 + 2, goal->y + goal->height / 2 + 12, buf);

        /*int boxArea = goal->width * goal->height;

		if (boxArea == 0) {
			continue;
		}

        int density = goal->area * 100 / boxArea;

        sprintf(buf, "%d - %d%%", goal->area, density);
        img.drawText(goal->x - goal->width / 2 + 2, goal->y - goal->height / 2 - 9, buf);*/
    }
}

void DebugRenderer::renderBrush(unsigned char* image, int x, int y, int radius, bool active, bool swapRB, int width, int height) {
	ImageBuffer img = ImageBuffer();

	img.data = image;
	img.width = width;
	img.height = height;
	img.swapRB = swapRB;

	img.drawCircle(x, y, radius, 255, active ? 0 : 255, active ? 0 : 255);
}