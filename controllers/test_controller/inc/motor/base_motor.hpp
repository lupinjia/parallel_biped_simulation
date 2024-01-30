#pragma once

#include <iostream>

using namespace std;

#define MOTOR_NUM 4

class BaseMotor
{
public:
	virtual void setPosition(int id, double pos) = 0;
	virtual void setPosition(double* posArr) = 0;
	virtual void setTorque(int id, double tor) = 0;
	virtual void setTorque(double* torArr) = 0;
	virtual void update() = 0;
	virtual double getPos(int id) = 0;
	virtual double* getPos() = 0; //all
	virtual double getVelocity(int id) = 0;
	virtual double* getVelocity() = 0; //all


};

