// File:          test_controller.cpp
// Date:          20240113
// Description:
// Author:        lupinjia
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <motor/webots_motor.hpp>
#include <common/matlab_plotter.hpp>
#include <common/parallel_leg.hpp>
#include <iostream>
#include <common/trajectory_planner.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

// 测试执行器
void testMotor(double* pos, int* variationDir, WebotsMotor* motor)
{
    if (*pos >= 1.0 || *pos <= -1.0)
          *variationDir *= 0;
    *pos += *variationDir * 0.001;
    cout << *pos << endl;
    motor->setPosition(0, *pos);
    //motor->setPosition(2, -(*pos));

}

// 测试FK,IK,Jacobian
void testKinematics(ParallelLeg* leg)
{
    Vec2 jointAngle;
    jointAngle << 2.885, 0.2489;
    Vec2 footPos;
    footPos << -0.0137, -0.2263;
    Mat2 jac = leg->calcJacobian(jointAngle);
    cout << "calced jacobian" << jac << endl;
    cout << "calced FK" << leg->calcForwardKinematics(jointAngle) << endl;
    cout << "IK" << leg->calcInverseKinematics(footPos) << endl;
}

// visualize the trajectory using matlab
void testTrajectory(TrajectoryPlanner* trajPlanner, MatlabPlotter* plotter)
{
    trajPlanner->update(); //更新轨迹规划器
    Vec2 footPos = trajPlanner->getFootPos(); //得到足端坐标
    double x = footPos(0);
    double y = footPos(1);
    plotter->addDynamicData(x, y); //添加x和y数据
}

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
    Robot* robot = new Robot();
   

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  //ParallelLeg leg0(0);
  WebotsMotor* motor = new WebotsMotor(robot, timeStep);
  //MatlabPlotter* plotter = new MatlabPlotter(timeStep, MatlabPlotType::X_SELF_DEFINE); //matlab初始化非常慢
  //TrajectoryPlanner* trajPlanner = new TrajectoryPlanner(1.0, timeStep, 0.1, 0.1); //步态周期1s,步长0.1m,步高0.1m


  // test motor
  double pos = 0;
  int variationDir = 1;
  //plotter init
  //plotter->initDynamicPlot();
  /********** test Trajectory init **********/


  

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {

      //testTrajectory(trajPlanner, plotter);
      /********** update the measurement value **********/
      //motor->update();

      /********** visualization **********/
      //plotter->addDynamicData(motor->getPos(0));

      /********** test program **********/ 
      testMotor(&pos, &variationDir, motor);
      
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
