#include "abb_robotnode/RobotController.hpp"

SERVICE_CALLBACK_DEF(GetRobotAngle)
{
  // Send command to robot
  int randNumber = generateRandNumber();
  sprintf(infoMsg, "%.2d %.3d #", 10, randNumber);

  // Return answer to end-user
  if(sendInfo(randNumber)) {
    int ok, idCode;
    double angle;
    sscanf(infoReply, "%*d %d %d %lf", &idCode, &ok, &angle);
    if((bool) ok) {
      res.angle = angle;
      res.success = true;
      res.msg = "Ok.";
    } else {
      res.success = false;
      res.msg = "Wrong answer from robot.";
    }
  } else {
    res.success = false;
    res.msg = "No answer received.";
  }
  return true;
}
