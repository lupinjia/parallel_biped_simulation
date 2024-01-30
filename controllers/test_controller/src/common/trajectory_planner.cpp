#include "common/trajectory_planner.hpp"

TrajectoryPlanner::TrajectoryPlanner(double gaitPeriod, int timeStep, double strideLength, double strideHeight)
{
	m_gaitPeriod = gaitPeriod;
	m_timeStep = (double)timeStep / 1000;
	m_strideLength = strideLength;
	m_strideHeight = strideHeight;
	m_footPos.setZero();
	m_time = 0;
	m_percent = 0;
}

TrajectoryPlanner::~TrajectoryPlanner()
{

}

void TrajectoryPlanner::update()
{
	double a = m_strideLength / 2;
	double h = m_strideHeight;
	// default duty cycle is 50%. first half for swing, second half for stance
	if (m_time >= 0 && m_time <= m_gaitPeriod / 2)
	{
		// 0->1
		m_percent = m_time / (m_gaitPeriod / 2);
		m_footPos(0) = a * (2 * m_percent - 1);
		m_footPos(1) = -4 * pow(m_percent, 2) * h + 4 * m_percent * h;
	}
	else if (m_time > m_gaitPeriod / 2 && m_time <= m_gaitPeriod)
	{
		// 1->0
		m_percent = (m_gaitPeriod - m_time) / (m_gaitPeriod / 2);
		m_footPos(0) = a * (2 * m_percent - 1);
		m_footPos(1) = 0;
	}
	// time progress.放在前面会跳过0
	m_time += m_timeStep;
	if (m_time > m_gaitPeriod)
		m_time = 0;
}

Vec2 TrajectoryPlanner::getFootPos()
{
	return m_footPos;
}
