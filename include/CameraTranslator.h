#ifndef CAMERATRANSLATOR_H
#define CAMERATRANSLATOR_H

class CameraTranslator {

public:
	struct WorldPosition {
		WorldPosition(float dx, float dy, float distance, float angle) : dx(dx), dy(dy), distance(distance), angle(angle) {}

		float dx; // in meters
		float dy;
		float distance;
		float angle; // in radians
	};

	struct CameraPosition {
		CameraPosition(int x, int y) : x(x), y(y) {}

		int x;
		int y;
	};

	void setConstants(float A, float B, float C, float horizon) {
		this->A = A;
		this->B = B;
		this->C = C;
		this->horizon = horizon;
	};

	WorldPosition getWorldPosition(int cameraX, int cameraY);
	CameraPosition getCameraPosition(float dx, float dy);

private:
	float A;
	float B;
	float C;
	float horizon;

};

#endif