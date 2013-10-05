#ifndef CAMERATRANSLATOR_H
#define CAMERATRANSLATOR_H

#include <math.h>

class CameraTranslator {

public:
	struct WorldPosition {
		WorldPosition(float dx, float dy, float distance, float angle) : dx(dx), dy(dy), distance(distance), angle(angle) {}

		float dx;
		float dy;
		float distance;
		float angle;
	};

	struct CameraPosition {
		CameraPosition(int x, int y) : x(x), y(y) {}

		int x;
		int y;
	};

	void setConstants(
		float A, float B, float C,
		float k1, float k2, float k3,
		float horizon, int cameraWidth, int cameraHeight);

	WorldPosition getWorldPosition(int cameraX, int cameraY);
	CameraPosition getCameraPosition(float dx, float dy);
	CameraTranslator::CameraPosition CameraTranslator::undistort(int x, int y);
	CameraTranslator::CameraPosition CameraTranslator::distort(int x, int y);

private:
	float A;
	float B;
	float C;
	float k1;
	float k2;
	float k3;
	float horizon;
	int cameraWidth;
	int cameraHeight;

};

#endif