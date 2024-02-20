#pragma once

#include <iostream>
#include "common/math_types.hpp"
#include "common/enum_class.h"

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
	virtual double getMotorPos(int motorID) = 0;
	virtual Vec2 getLegMotorPos(LegID legID) = 0;
	virtual Vec4 getAllMotorPos() = 0; //all
	virtual double getMotorVelocity(int motorID) = 0;
	virtual Vec2 getLegMotorVelocity(LegID legID) = 0;
	virtual Vec4 getAllMotorVelocity() = 0; //all


};

