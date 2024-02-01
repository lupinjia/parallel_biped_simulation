#pragma once

#include <common/math_types.hpp>

#define X_OFFSET 0
#define Z_OFFSET -0.2f


class TrajectoryPlanner
{
public:
	TrajectoryPlanner(double gaitPeriod, int timeStep, double strideLength, double strideHeight);
	~TrajectoryPlanner();
	void update();
	Vec2 getFootPos();
	Vec2 getInitialFootPos();

private:
	double m_gaitPeriod;
	double m_timeStep;
	double m_strideLength;
	double m_strideHeight;
	Vec2 m_footPos;
	double m_time;
	double m_percent;
};

