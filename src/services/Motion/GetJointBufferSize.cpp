#include "abb_robotnode/RobotController.hpp"

SERVICE_CALLBACK_DEF(GetJointBufferSize)
{
  // Send command to robot
  int randNumber = generateRandNumber();
  sprintf(motionMsg, "%.2d %.3d #", 32, randNumber);

  // Return answer to end-user
  if(sendMotion(randNumber)) {
    int ok, idCode, size;
    sscanf(motionReply, "%*d %d %d %d", &idCode, &ok, &size);
    if((bool) ok) {
      res.size = size;
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
