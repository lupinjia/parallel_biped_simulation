#pragma once

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include "motor/base_motor.hpp"
#include <string>

using namespace webots;
using namespace std;

class WebotsMotor: public BaseMotor
{

public:

	WebotsMotor(Robot* robot, int samplingPeriod);
	~WebotsMotor();
	void setPosition(int id, double pos);
	void setPosition(double* posArr);
	void setTorque(int id, double tor);
	void setTorque(double* torArr);
	void update();
	double getPos(int id);
	double* getPos(); //all
	double getVelocity(int id);
	double* getVelocity(); //all
private:

	Motor* m_motor[MOTOR_NUM];
	PositionSensor* m_posSensor[MOTOR_NUM];
	double m_curPos[MOTOR_NUM];
	double m_lastPos[MOTOR_NUM];
	double m_vel[MOTOR_NUM];
	double m_timeStep; //second
	const double m_maxTorque = 21;
};

