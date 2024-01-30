#pragma once

#include <common/math_types.hpp>


class ParallelLeg
{
public:
	ParallelLeg(int legID);
	~ParallelLeg();
	Vec2 calcForwardKinematics(Vec2 jointAngle);
	Vec2 calcInverseKinematics(Vec2 footPos);
	Mat2 calcJacobian(Vec2 jointAngle);

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

