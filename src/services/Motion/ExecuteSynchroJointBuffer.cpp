#include "abb_robotnode/RobotController.hpp"

SERVICE_CALLBACK_DEF(ExecuteSynchroJointBuffer)
{
  // Send command to robot
  int randNumber = generateRandNumber();
  sprintf(motionMsg, "%.2d %.3d %.3d #", 40, randNumber, 1);

  // Return answer to end-user
  if(sendMotion(randNumber)) {
    int ok, idCode;
    sscanf(motionReply, "%*d %d %d", &idCode, &ok);
    if((bool) ok) {
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
