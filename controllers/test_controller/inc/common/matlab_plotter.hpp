#pragma once

#include "engine.h"
#include <iostream>
#include <string>
#include <common/enum_class.h>

using namespace std;

class MatlabPlotter
{
public:

	MatlabPlotter(int timeStep, MatlabPlotType plotType);
	~MatlabPlotter();
	/********** for dynamic plot **********/
	void initDynamicPlot();
	void addDynamicData(double yData);
	void addDynamicData(double xData, double yData);

private:

	Engine* m_enginePtr;
	double m_timeStep; //second
	mxArray* m_dynamicYData; //y data for dynamic plot
	mxArray* m_dynamicXData; //x data for dynamic plot
	mxArray* m_dynamicXData2;
	mxArray* m_dynamicYData2;
	MatlabPlotType m_plotType;
};

