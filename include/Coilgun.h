#ifndef COILGUN_H
#define COILGUN_H

class Coilgun {

public:
	Coilgun();
	~Coilgun();

	void kick(int strength = 1500);
	void charge();
	void discharge();
	bool isReady() { return false; } // TODO New communication
	void step(float dt);

private:
	int kickStrength;
	bool doKick;
	bool doDischarge;
	bool charging;
	bool ready;
	double lastKickTime;

};

#endif //COILGUN_H