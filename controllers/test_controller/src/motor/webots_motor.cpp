#include "motor/webots_motor.hpp"

WebotsMotor::WebotsMotor(Robot* robot, int timeStep)
{
	//	m_motor[i] = robot->getMotor("motor_" + to_string(i)); //const char和string用"+"无法实现字符串拼接的效果
	/********** 初始化设备 **********/
	string motorNamePrefix = "motor_";
	string posSensorNamePrefix = "pos_";
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		m_motor[i] = robot->getMotor(motorNamePrefix + to_string(i + 1)); //需要同为string类型用"+"才可以拼接
		m_posSensor[i] = robot->getPositionSensor(posSensorNamePrefix + to_string(i + 1));
		m_posSensor[i]->enable(timeStep);
	}
	/*********** 初始化变量 **********/
	m_timeStep = (double)timeStep / 1000; //ms->s
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		m_curPos[i] = 0;
		m_lastPos[i] = 0;
		m_vel[i] = 0;
	}

	/*m_motor[0] = robot->getMotor("motor_1");
	m_motor[1] = robot->getMotor("motor_2");
	m_motor[2] = robot->getMotor("motor_3");
	m_motor[3] = robot->getMotor("motor_4");*/

	cout << "webots motor init done!" << endl;
}

WebotsMotor::~WebotsMotor()
{
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		delete m_motor[i];
		delete m_posSensor[i];
	}
}

void WebotsMotor::setPosition(int id, double pos)
{
	m_motor[id]->setPosition(pos);
}

void WebotsMotor::setPosition(double* posArr)
{
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		m_motor[i]->setPosition(posArr[i]);
	}
}

void WebotsMotor::setTorque(int id, double tor)
{
	m_motor[id]->setTorque(tor);
}

void WebotsMotor::setTorque(double* torArr)
{
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		m_motor[i]->setTorque(torArr[i]);
	}
}

// update variables
void WebotsMotor::update()
{
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		m_lastPos[i] = m_curPos[i];
		m_curPos[i] = m_posSensor[i]->getValue();
		m_vel[i] = (m_curPos - m_lastPos) / m_timeStep;
	}
}

double WebotsMotor::getPos(int id)
{
	return m_curPos[id];
}

double* WebotsMotor::getPos()
{
	return m_curPos;
}

double WebotsMotor::getVelocity(int id)
{
	return m_vel[id];
}

double* WebotsMotor::getVelocity()
{
	return m_vel;
}
