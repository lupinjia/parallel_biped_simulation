#pragma once

#include <webots/Robot.hpp>
#include "common/parallel_leg.hpp"
#include "motor/webots_motor.hpp"
#include "sensor/webots_imu.hpp"
#include "sensor/webots_contact_sensor.hpp"

class Biped
{
public:
	/**
	 * @brief Robot类整合了腿部ParallelLeg, 电机motor, 惯性测量单元imu, 接触传感器contact sensor.
	 * 
	 * \param robot
	 * \param timeStep
	 */
	Biped(Robot* robot, int timeStep);
	~Biped();
	/**
	 * @brief 传感器更新测量
	 * 
	 */
	void update();
	/*
	* 腿部位置控制
	*/
	void setLegPosition(LegID legID, Vec2 angle);
	void setLegPosition(Vec2 angleRight, Vec2 angleLeft);
	/*
	* 腿部力矩控制
	*/

	/**
	 * 腿部正逆运动学解算.
	 */
	Vec2 calcLegFK(LegID legID, Vec2 jointAngle);
	Vec2 calcLegIK(LegID legID, Vec2 footPos);

	/**
	 * .
	 */
	Vec2 getLegPosition(LegID legID);
	Vec4 getLegPosition();

private:
	Robot* m_robot;
	int m_timeStep;
	ParallelLeg* m_leg0; // right leg
	ParallelLeg* m_leg1; // left leg
	BaseMotor* m_motor;
	BaseIMU* m_imu;
	BaseContactSensor* m_contactSensor;
};

