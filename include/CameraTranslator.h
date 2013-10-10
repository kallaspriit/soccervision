#ifndef CAMERATRANSLATOR_H
#define CAMERATRANSLATOR_H

#include <math.h>

class CameraTranslator {

public:
	struct WorldPosition {
		WorldPosition() : dx(0.0f), dy(0.0f), distance(0.0f), angle(0.0f) {}
		WorldPosition(float dx, float dy, float distance, float angle) : dx(dx), dy(dy), distance(distance), angle(angle) {}

		float dx;
		float dy;
		float distance;
		float angle;
	};

	struct CameraPosition {
		CameraPosition() : x(0), y(0) {}
		CameraPosition(int x, int y) : x(x), y(y) {}

		int x;
		int y;
	};

	CameraTranslator() : A(0.0f), B(0.0f), C(0.0f), k1(0.0f), k2(0.0f), k3(0.0f), horizon(0.0f), cameraWidth(0), cameraHeight(0) {}


	/* Esimese Kaamera konstandid testimiseks:
		A=-65
		B=252000
		C=187
		Horizon=3.0

		float distortionFocus = 1230.9756548915236;
		float k1 = -1.7578689303905209e-001;
		float k2 = -2.5481256902241417e-001;
		float k3 =  7.6993101864245295e-001;
	*/

	void setConstants(
		int cameraWidth, int cameraHeight,
		float A, float B, float C, float horizon, 
		float distortionFocus,
		float k1, float k2, float k3);

	WorldPosition getWorldPosition(int cameraX, int cameraY);
	CameraPosition getCameraPosition(float dx, float dy);
	CameraTranslator::CameraPosition CameraTranslator::undistort(int x, int y);

private:
	int cameraWidth;
	int cameraHeight;

	//World <--> Camera mapping constants
	float A;
	float B;
	float C;
	float horizon;

	//Distortion constants
	float distortionFocus;
	float k1;
	float k2;
	float k3;
};

#endif