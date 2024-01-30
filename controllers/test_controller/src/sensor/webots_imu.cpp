#include "sensor/webots_imu.hpp"

WebotsIMU::WebotsIMU(Robot* robot, int timeStep)
{
	/********** 初始化设备 **********/
	m_imu = robot->getInertialUnit("imu");
	m_imu->enable(timeStep);

	/*********** 初始化变量 **********/
	m_timeStep = (double)timeStep / 1000; //ms->s
	for (int i = 0; i < 3; i++)
	{
		m_curAngle[i] = 0;
		m_lastAngle[i] = 0;
		m_angleSpeed[i] = 0;
	}
	for (int i = 0; i < 4; i++)
	{
		m_curQuat[i] = 0;
	}

	cout << "webots imu init done" << endl;

}

WebotsIMU::~WebotsIMU()
{
	delete m_imu;
}

void WebotsIMU::update()
{
	//姿态角及角速度
	for (int i = 0; i < 3; i++)
	{
		m_lastAngle[i] = m_curAngle[i];
		m_curAngle[i] = m_imu->getRollPitchYaw()[i];
		m_angleSpeed[i] = (m_curAngle[i] - m_lastAngle[i]) / m_timeStep; // rad/s
	}
	//四元数
	for (int i = 0; i < 4; i++)
	{
		m_curQuat[i] = m_imu->getQuaternion()[i];
	}
}

double WebotsIMU::getAngle(int id)
{
	return m_curAngle[id];
}

double* WebotsIMU::getAngle()
{
	return m_curAngle;
}

double WebotsIMU::getAngleSpeed(int id)
{
	return m_angleSpeed[id];
}

double* WebotsIMU::getAngleSpeed()
{
	return m_angleSpeed;
}

double WebotsIMU::getQuat(int id)
{
	return m_curQuat[id];
}

double* WebotsIMU::getQuat()
{
	return m_curQuat;
}
