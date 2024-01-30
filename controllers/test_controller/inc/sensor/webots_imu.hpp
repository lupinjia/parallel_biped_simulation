#pragma once

#include <webots/Robot.hpp>
#include <webots/InertialUnit.hpp>
#include "sensor/base_imu.hpp"

using namespace webots;

class WebotsIMU: public BaseIMU
{
public:

	WebotsIMU(Robot* robot, int timeStep);
	~WebotsIMU();
	void update();
	double getAngle(int id);
	double* getAngle();
	double getAngleSpeed(int id);
	double* getAngleSpeed();
	double getQuat(int id);
	double* getQuat();

private:

	InertialUnit* m_imu;
	double m_timeStep; //second
	double m_curAngle[3];
	double m_lastAngle[3];
	double m_angleSpeed[3];
	double m_curQuat[4];
};

