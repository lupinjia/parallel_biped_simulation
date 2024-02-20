#include "common/parallel_leg.hpp"

ParallelLeg::ParallelLeg(int legID): m_legID(legID)
{

}

ParallelLeg::~ParallelLeg()
{
}

//计算髋部坐标系下的足端坐标
Vec2 ParallelLeg::calcForwardKinematics(Vec2 jointAngle)
{
	double theta = jointAngle(0);
	double fai = jointAngle(1);
	double linkACLength = sqrt(pow(m_linkOALength,2) + pow(m_linkOCLength,2) - 2 * m_linkOALength * m_linkOCLength * cos(theta - fai));
	double gamma1 = acos((pow(m_linkOCLength,2) + pow(linkACLength,2) - pow(m_linkOALength,2)) / (2 * m_linkOCLength * linkACLength));
	double gamma2 = acos((pow(linkACLength, 2) + pow(m_linkCBLength, 2) - pow(m_linkABLength, 2)) / (2 * linkACLength * m_linkCBLength));
	double alpha = gamma1 + gamma2 - fai;
	double x = m_linkOCLength * cos(fai) - m_linkCDLength * cos(alpha);
	double z = -(m_linkOCLength * sin(fai) + m_linkCDLength * sin(alpha));
	Vec2 footPos;
	footPos(0) = x;
	footPos(1) = z;
	return footPos;
}

//接收髋部坐标系下的足端坐标,计算关节角度
Vec2 ParallelLeg::calcInverseKinematics(Vec2 footPos)
{
	double x = footPos(0);
	double z = footPos(1);
	//求解fai
	double linkODLength = sqrt(pow(x, 2) + pow(z, 2));
	double angleOCD = acos((pow(m_linkOCLength, 2) + pow(m_linkCDLength, 2) - pow(linkODLength, 2)) / (2 * m_linkOCLength * m_linkCDLength));
	double k1 = m_linkOCLength - m_linkCDLength * cos(angleOCD);
	double k2 = m_linkCDLength * sin(angleOCD);
	double l = sqrt(pow(k1, 2) + pow(k2, 2));
	double gamma = atan2(k2, k1);
	double fai = acos(x / l) - gamma;
	//求解theta
	double alpha = angleOCD - fai;
	double xC = m_linkOCLength * cos(fai);
	double yC = -m_linkOCLength * sin(fai);
	double xB = xC - m_linkCBLength * cos(alpha);
	double yB = yC - m_linkCBLength * sin(alpha);
	double linkOBLength = sqrt(pow(xB, 2) + pow(yB, 2));
	double angleAOB = acos((pow(m_linkOALength, 2) + pow(linkOBLength, 2) - pow(m_linkABLength, 2)) / (2 * m_linkOALength * linkOBLength));
	double angleCOB = acos((pow(linkOBLength, 2) + pow(m_linkOCLength, 2) - pow(m_linkCBLength, 2)) / (2 * linkOBLength * m_linkOCLength));
	double theta = angleAOB + angleCOB + fai;

	Vec2 jointAngle;
	jointAngle(0) = theta;
	jointAngle(1) = fai;
	return jointAngle;
}

Mat2 ParallelLeg::calcJacobian(Vec2 jointAngle)
{
	Mat2 jacobian;
	double theta = jointAngle(0);
	double fai = jointAngle(1);
	jacobian(0,0) = (23 * sin(acos((sqrt(2) * sqrt(1 - cos(fai - theta))) / 2) - fai + acos((sqrt(2) * sqrt(1 - cos(fai - theta))) / 4)) * ((sqrt(2) * sin(fai - theta)) / (4 * sqrt(1 - cos(fai - theta)) * sqrt(cos(fai - theta) / 2 + 0.5)) + (sqrt(2) * sin(fai - theta)) / (8 * sqrt(1 - cos(fai - theta)) * sqrt(cos(fai - theta) / 8 + 7.0 / 8)))) / 100;
	jacobian(0,1) = -sin(fai) / 10 - (23 * sin(acos((sqrt(2) * sqrt(1 - cos(fai - theta))) / 2) - fai + acos((sqrt(2) * sqrt(1 - cos(fai - theta))) / 4)) * ((sqrt(2) * sin(fai - theta)) / (4 * sqrt(1 - cos(fai - theta)) * sqrt(cos(fai - theta) / 2 + 1.0 / 2)) + (sqrt(2) * sin(fai - theta)) / (8 * sqrt(1 - cos(fai - theta)) * sqrt(cos(fai - theta) / 8 + 7.0 / 8)) + 1)) / 100;
	jacobian(1, 0) = -(23 * cos(acos((sqrt(2) * sqrt(1 - cos(fai - theta))) / 2) - fai + acos((sqrt(2) * sqrt(1 - cos(fai - theta))) / 4)) * ((sqrt(2) * sin(fai - theta)) / (4 * sqrt(1 - cos(fai - theta)) * sqrt(cos(fai - theta) / 2 + 1.0 / 2)) + (sqrt(2) * sin(fai - theta)) / (8 * sqrt(1 - cos(fai - theta)) * sqrt(cos(fai - theta) / 8 + 7.0 / 8)))) / 100;
	jacobian(1, 1) = (23 * cos(acos((sqrt(2) * sqrt(1 - cos(fai - theta))) / 2) - fai + acos((sqrt(2) * sqrt(1 - cos(fai - theta))) / 4)) * ((sqrt(2) * sin(fai - theta)) / (4 * sqrt(1 - cos(fai - theta)) * sqrt(cos(fai - theta) / 2 + 1.0 / 2)) + (sqrt(2) * sin(fai - theta)) / (8 * sqrt(1 - cos(fai - theta)) * sqrt(cos(fai - theta) / 8 + 7.0 / 8)) + 1)) / 100 - cos(fai) / 10;

	return jacobian;
}

Vec2 ParallelLeg::calcJointTorque(Vec2 footForce, Vec2 jointAngle)
{
	Vec2 torque;
	Mat2 jacobian = calcJacobian(jointAngle);
	torque = jacobian.transpose() * footForce;
	return torque;
}


