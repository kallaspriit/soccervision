#ifndef WHEEL_H
#define WHEEL_H

#include <string>

class Wheel {

public:
    Wheel(int id);
    ~Wheel();

    void setTargetOmega(float omega);
    float getTargetOmega() const;
    float getRealOmega() const;
	bool isReady() const { return false; } // TODO Implement connection
	bool isStalled();
    void step(float dt);

    static float omegaToSpeed(float omega);
    static float speedToOmega(float speed);

    //std::string getStateJSON() const;

private:
    int id;
    float targetOmega;
    float realOmega;
    bool ready;
	bool onceOpened;
	double lastMessageTime;
	int stallCounter;

    static const float pidFrequency;
    static const float ticksPerRevolution;

};

#endif // WHEEL_H
