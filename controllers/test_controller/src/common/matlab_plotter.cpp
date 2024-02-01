#include "common/matlab_plotter.hpp"

MatlabPlotter::MatlabPlotter(int timeStep, MatlabPlotType plotType): m_plotType(plotType)
{
	/********** 初始化设备 **********/
	if (!(m_enginePtr = engOpen(NULL)))
	{
		cout << "can't start MATLAB engine!" << endl;
		exit(1);
	}
	/********** 初始化变量 **********/
	m_timeStep = (double)timeStep / 1000; //ms->s
	m_dynamicYData = NULL;
	m_dynamicXData = NULL;

	cout << "matlab plotter init done!" << endl;
}

MatlabPlotter::~MatlabPlotter()
{
	engClose(m_enginePtr);
}

//初始化动态绘图
void MatlabPlotter::initDynamicPlot()
{
	if (m_plotType == MatlabPlotType::X_TIME)
	{
		engEvalString(m_enginePtr, "t = 0;");
		engEvalString(m_enginePtr, "y = 0;");
		engEvalString(m_enginePtr, "p = plot(t, y);");
		m_dynamicYData = mxCreateDoubleMatrix(1, 1, mxREAL); // array
		*mxGetPr(m_dynamicYData) = 0; // value init to 0
		engPutVariable(m_enginePtr, "temp_y", m_dynamicYData);
		engEvalString(m_enginePtr, "i = 0;");
	}
	else if (m_plotType == MatlabPlotType::X_SELF_DEFINE)
	{
		engEvalString(m_enginePtr, "x = 0;");
		engEvalString(m_enginePtr, "y = 0;");
		engEvalString(m_enginePtr, "p = plot(x, y);");
		m_dynamicYData = mxCreateDoubleMatrix(1, 1, mxREAL); // array
		m_dynamicXData = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(m_dynamicYData) = 0;
		*mxGetPr(m_dynamicXData) = 0;
		engPutVariable(m_enginePtr, "temp_x", m_dynamicXData);
		engPutVariable(m_enginePtr, "temp_y", m_dynamicYData);
	}
	else if (m_plotType == MatlabPlotType::TWO_X_SELF_DEFINE)
	{
		engEvalString(m_enginePtr, "x1 = 0;");
		engEvalString(m_enginePtr, "x2 = 0;");
		engEvalString(m_enginePtr, "y1 = 0;");
		engEvalString(m_enginePtr, "y2 = 0;");
		engEvalString(m_enginePtr, "p = plot(x1, y1, '-b', x2, y2, '-r');");
		m_dynamicXData = mxCreateDoubleMatrix(1, 1, mxREAL);
		m_dynamicXData2 = mxCreateDoubleMatrix(1, 1, mxREAL);
		m_dynamicYData = mxCreateDoubleMatrix(1, 1, mxREAL);
		m_dynamicYData2 = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(m_dynamicXData) = 0;
		*mxGetPr(m_dynamicXData2) = 0;
		*mxGetPr(m_dynamicYData) = 0;
		*mxGetPr(m_dynamicYData2) = 0;
		engPutVariable(m_enginePtr, "temp_x1", m_dynamicXData);
		engPutVariable(m_enginePtr, "temp_x2", m_dynamicXData2);

	}
	
}

//添加动态数据,并在图像中显示效果
void MatlabPlotter::addDynamicData(double yData)
{
	if (m_plotType == MatlabPlotType::X_TIME)
	{
		engEvalString(m_enginePtr, "i = i + 1;");
		engEvalString(m_enginePtr, "t = [t 0.002*i];");
		*mxGetPr(m_dynamicYData) = yData;
		engPutVariable(m_enginePtr, "temp_y", m_dynamicYData);
		engEvalString(m_enginePtr, "y = [y temp_y];");
		engEvalString(m_enginePtr, "set(p, 'XData', t, 'YData', y)");
		engEvalString(m_enginePtr, "drawnow");
	}
	else
	{
		cout << "Plot Type is wrong!" << endl;
		return;
	}
	
}

void MatlabPlotter::addDynamicData(double xData, double yData)
{
	if (m_plotType == MatlabPlotType::X_SELF_DEFINE)
	{
		*mxGetPr(m_dynamicXData) = xData;
		*mxGetPr(m_dynamicYData) = yData;
		engPutVariable(m_enginePtr, "temp_x", m_dynamicXData);
		engPutVariable(m_enginePtr, "temp_y", m_dynamicYData);
		engEvalString(m_enginePtr, "x = [x temp_x];");
		engEvalString(m_enginePtr, "y = [y temp_y];");
		engEvalString(m_enginePtr, "set(p, 'XData', x, 'YData', y);");
		engEvalString(m_enginePtr, "drawnow");
	}
	else
	{
		cout << "Plot Type is wrong!" << endl;
		return;
	}
}
