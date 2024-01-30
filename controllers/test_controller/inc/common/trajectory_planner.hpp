#pragma once

#include <common/math_types.hpp>

class TrajectoryPlanner
{
public:
	TrajectoryPlanner(double gaitPeriod, int timeStep, double strideLength, double strideHeight);
	~TrajectoryPlanner();
	void update();
	Vec2 getFootPos();

private:
	double m_gaitPeriod;
	double m_timeStep;
	double m_strideLength;
	double m_strideHeight;
	Vec2 m_footPos;
	double m_time;
	double m_percent;
};

