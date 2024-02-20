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

double WebotsMotor::getMotorPos(int motorID)
{
	return m_curPos[motorID];
}

Vec2 WebotsMotor::getLegMotorPos(LegID legID)
{
	Vec2 motorPos;
	if (legID == LegID::RIGHT_LEG)
	{
		motorPos(0) = m_curPos[0]; //theta
		motorPos(1) = m_curPos[1]; //fai
	}
	else if (legID == LegID::LEFT_LEG)
	{
		motorPos(0) = m_curPos[3]; //theta
		motorPos(1) = m_curPos[2]; //fai
	}
	return motorPos;
}

Vec4 WebotsMotor::getAllMotorPos()
{
	//先右腿后左腿,依次为right_theta, right_fai, left_theta, left_fai
	Vec4 motorPos;
	motorPos(0) = m_curPos[0];
	motorPos(1) = m_curPos[1];
	motorPos(2) = m_curPos[3];
	motorPos(3) = m_curPos[2];
	return motorPos;
}

double WebotsMotor::getMotorVelocity(int motorID)
{
	return m_vel[motorID];
}

Vec2 WebotsMotor::getLegMotorVelocity(LegID legID)
{
	Vec2 motorVel;
	if (legID == LegID::RIGHT_LEG)
	{
		motorVel(0) = m_vel[0]; //theta
		motorVel(1) = m_vel[1]; //fai
	}
	else if (legID == LegID::LEFT_LEG)
	{
		motorVel(0) = m_vel[3]; //theta
		motorVel(1) = m_vel[2]; //fai
	}
	return motorVel;
}

Vec4 WebotsMotor::getAllMotorVelocity()
{
	//先右腿后左腿,依次为right_theta, right_fai, left_theta, left_fai
	Vec4 motorVel;
	motorVel(0) = m_vel[0];
	motorVel(1) = m_vel[1];
	motorVel(2) = m_vel[3];
	motorVel(3) = m_vel[2];
	return motorVel;
}

