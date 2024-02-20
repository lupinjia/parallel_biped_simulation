#include "sensor/webots_contact_sensor.hpp"

WebotsContactSensor::WebotsContactSensor(Robot* robot, int timeStep)
{
	/********** 初始化设备 **********/
	m_touchSensor[0] = robot->getTouchSensor("Foot_A"); // right foot
	m_touchSensor[1] = robot->getTouchSensor("Foot_B"); // left foot
	for (int i = 0; i < FOOT_NUM; i++)
	{
		m_touchSensor[i]->enable(timeStep);
	}

	/*********** 初始化变量 **********/
	for (int i = 0; i < FOOT_NUM; i++)
	{
		m_contact[i] = 0;
	}

	cout << "webots contact sensor init done" << endl;
}

WebotsContactSensor::~WebotsContactSensor()
{
	for (int i = 0; i < FOOT_NUM; i++)
	{
		delete m_touchSensor[i];
	}
}

void WebotsContactSensor::update()
{
	for (int i = 0; i < FOOT_NUM; i++)
	{
		m_contact[i] = m_touchSensor[i]->getValue();
	}
}

bool WebotsContactSensor::getContact(int id)
{
	return m_contact[id];
}

bool* WebotsContactSensor::getContact()
{
	return m_contact;
}


