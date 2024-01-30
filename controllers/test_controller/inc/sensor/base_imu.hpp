#pragma once

#include <iostream>

using namespace std;

class BaseIMU
{
public:

	virtual void update() = 0;
	virtual double getAngle(int id) = 0;
	virtual double* getAngle() = 0;
	virtual double getAngleSpeed(int id) = 0;
	virtual double* getAngleSpeed() = 0;
	virtual double getQuat(int id) = 0;
	virtual double* getQuat() = 0;
};

