#include "abb_robotnode/RobotController.hpp"

SERVICE_CALLBACK_DEF(GetCartesian)
{
  // Send command to robot
  int randNumber = generateRandNumber();
  sprintf(infoMsg, "%.2d %.3d #", 3, randNumber);

  // Return answer to end-user
  if(sendInfo(randNumber)) {
    int ok, idCode;
    sscanf(infoReply, "%*d %d %d %*f %lf %lf %lf %lf %lf %lf %lf", &idCode, &ok, &res.x, &res.y, &res.z, &res.q0, &res.qx, &res.qy, &res.qz);
    res.success = (bool) ok;
    res.msg = (res.success ? "Ok." : "Wrong answer from robot.");
  } else {
    res.success = false;
    res.msg = "No answer received.";
  }
  return true;
}
