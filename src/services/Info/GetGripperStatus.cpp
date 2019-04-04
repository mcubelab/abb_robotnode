#include "abb_robotnode/RobotController.hpp"

SERVICE_CALLBACK_DEF(GetGripperStatus)
{
  // Send command to robot
  int randNumber = generateRandNumber();
  sprintf(infoMsg, "%.2d %.3d #", 23, randNumber);

  // Return answer to end-user
  if(sendInfo(randNumber)) {
    int ok, idCode, status;
    sscanf(infoReply, "%*d %d %d %d", &idCode, &ok, &status);
    if((bool) ok) {
      res.calibrated = (bool) status;
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
