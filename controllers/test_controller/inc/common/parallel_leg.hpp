#pragma once

#include <common/math_types.hpp>

#define INITIAL_THETA 2.885f
#define INITIAL_FAI   0.2489f


class ParallelLeg
{
public:
	ParallelLeg(int legID);
	~ParallelLeg();
	Vec2 calcForwardKinematics(Vec2 jointAngle);
	Vec2 calcInverseKinematics(Vec2 footPos);
	Mat2 calcJacobian(Vec2 jointAngle);
	Vec2 calcJointTorque(Vec2 footForce, Vec2 jointAngle);

private:
	/********** link length[m] **********/
	const double m_linkOCLength = 0.1;
	const double m_linkOALength = 0.1;
	const double m_linkCBLength = 0.2;
	const double m_linkABLength = 0.2;
	const double m_linkCDLength = 0.23;
	const double m_linkBDLength = 0.03;

	int m_legID;


};

