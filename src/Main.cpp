/*
  abb_robotnode: Main program
*/

#include "abb_robotnode/RobotController.hpp"

void loggerMain(RobotController* robot);

int main(int argc, char** argv)
{
  // Prepare ROS node and RobotController instance
  ros::init(argc, argv, "abb_robotnode");
  ros::NodeHandle node;
  RobotController robot(&node);

  // Initialize all services and connections
  std::string robotId = (argc == 2 ? argv[1] : "");
  if (!robot.init(robotId))
    exit(-1);

  // Main ROS loop
  ROS_INFO("%s: Running ROS robot node...", robot.msgHeader.c_str());

  // Multithreaded spinner so that callbacks can be handled on separate threads.
  ros::MultiThreadedSpinner spinner(3); // 3 total threads (main+logger+egm)
  spinner.spin();

  // End of program
  robot.joinThreads();

  ROS_INFO("%s: Done.", robot.msgHeader.c_str());
  return 0;
}
