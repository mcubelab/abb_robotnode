#include "abb_robotnode/RobotController.hpp"

SERVICE_CALLBACK_DEF(GetFK)
{
  // Send command to robot
  int randNumber = generateRandNumber();

  if(prepareJoints(infoMsg, 13, randNumber, req.joints)) {
    if(sendInfo(randNumber)) {
      int ok, idCode;
      sscanf(infoReply, "%*d %d %d %*f %lf %lf %lf %lf %lf %lf %lf", &idCode, &ok, &res.x, &res.y, &res.z, &res.q0, &res.qx, &res.qy, &res.qz);
      res.success = (bool) ok;
      res.msg = (res.success ? "Ok." : "Wrong answer from robot.");
    } else {
      res.success = false;
      res.msg = "No answer received.";
    }
  } else {
    res.success = false;
    res.msg = "Wrong number of joints for this robot.";
  }
  return true;
}
