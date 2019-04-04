#include "abb_robotnode/RobotController.hpp"

SERVICE_CALLBACK_DEF(GetGripperPosition)
{
  // Send command to robot
  int randNumber = generateRandNumber();
  sprintf(infoMsg, "%.2d %.3d #", 24, randNumber);

  // Return answer to end-user
  if(sendInfo(randNumber)) {
    int ok, idCode;
    double position;
    sscanf(infoReply, "%*d %d %d %lf", &idCode, &ok, &position);
    if((bool) ok) {
      res.position = position;
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
