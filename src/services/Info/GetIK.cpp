#include "abb_robotnode/RobotController.hpp"

SERVICE_CALLBACK_DEF(GetIK)
{
  // Send command to robot
  int randNumber = generateRandNumber();
  sprintf(infoMsg, "%.2d %.3d %+08.1lf %+08.1lf %+08.1lf %+08.5lf %+08.5lf %+08.5lf %+08.5lf #", 12, randNumber, req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz);

  // Return answer to end-user
  if(sendInfo(randNumber)) {
    int ok, idCode;
    parseJoints(infoReply, idCode, ok, res.joints);
    res.success = (bool) ok;
    res.msg = (res.success ? "Ok." : "Wrong answer from robot.");
  } else {
    res.success = false;
    res.msg = "No answer received.";
  }
  return true;
}
