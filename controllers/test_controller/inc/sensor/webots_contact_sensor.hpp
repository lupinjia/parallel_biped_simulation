#pragma once

#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>
#include "sensor/base_contact_sensor.hpp"

using namespace webots;

class WebotsContactSensor: public BaseContactSensor
{

public:

	WebotsContactSensor(Robot* robot, int timeStep);
	~WebotsContactSensor();
	void update();
	bool getContact(int id);
	bool* getContact(); //all

private:

	TouchSensor* m_touchSensor[FOOT_NUM];
	bool m_contact[FOOT_NUM];
};

