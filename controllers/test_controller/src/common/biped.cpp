#include "common/biped.hpp"

Biped::Biped(Robot* robot, int timeStep): m_robot(robot), m_timeStep(timeStep)
{
	m_leg0 = new ParallelLeg(0);
	m_leg1 = new ParallelLeg(1);
	m_motor = new WebotsMotor(m_robot, m_timeStep);
	m_imu = new WebotsIMU(m_robot, m_timeStep);
	m_contactSensor = new WebotsContactSensor(m_robot, m_timeStep);
}

Biped::~Biped()
{
	delete m_leg0;
	delete m_leg1;
	delete m_motor;
	delete m_imu;
	delete m_contactSensor;
	delete m_robot;
}

void Biped::update()
{
	m_motor->update();
	m_imu->update();
	//m_contactSensor->
}

void Biped::setLegPosition(LegID legID, Vec2 angle)
{
	if (legID == LegID::RIGHT_LEG) // right leg
	{
		// 解算得到的关节角度到给电机的角度命令有一个偏移
		m_motor->setPosition(0, angle(0) - INITIAL_THETA); //theta
		m_motor->setPosition(1, angle(1) - INITIAL_FAI); //fai
	}
	else if (legID == LegID::LEFT_LEG) // left
	{
		m_motor->setPosition(3, angle(0) - INITIAL_THETA); //theta
		m_motor->setPosition(2, angle(1) - INITIAL_FAI); //fai
	}
}

void Biped::setLegPosition(Vec2 angleRight, Vec2 angleLeft)
{
	setLegPosition(LegID::RIGHT_LEG, angleRight);
	setLegPosition(LegID::LEFT_LEG, angleLeft);
}

Vec2 Biped::calcLegFK(LegID legID, Vec2 jointAngle)
{
	if (legID == LegID::RIGHT_LEG)
	{
		return m_leg0->calcForwardKinematics(jointAngle);
	}
	else if (legID == LegID::LEFT_LEG)
	{
		return m_leg1->calcForwardKinematics(jointAngle);
	}
}

Vec2 Biped::calcLegIK(LegID legID, Vec2 footPos)
{
	if (legID == LegID::RIGHT_LEG)
	{
		return m_leg0->calcInverseKinematics(footPos);
	}
	else if (legID == LegID::LEFT_LEG)
	{
		return m_leg1->calcInverseKinematics(footPos);
	}
}

Vec2 Biped::getLegPosition(LegID legID)
{
	Vec2 motorPos = m_motor->getLegMotorPos(legID);
	motorPos(0) += INITIAL_THETA;
	motorPos(1) += INITIAL_FAI;
	return motorPos;
}

Vec4 Biped::getLegPosition()
{
	Vec4 motorPos = m_motor->getAllMotorPos();
	motorPos(0) += INITIAL_THETA;
	motorPos(1) += INITIAL_FAI;
	motorPos(2) += INITIAL_THETA;
	motorPos(3) += INITIAL_FAI;
	return motorPos;
}




