#include "abb_robotnode/RobotController.hpp"

SERVICE_CALLBACK_DEF(Ping)
{
  // Send command to robot
  int randNumber = generateRandNumber();
  sprintf(infoMsg, "%.2d %.3d #", 0, randNumber);

  // Return answer to end-user
  if(sendInfo(randNumber)) {
    int ok, idCode;
    sscanf(infoReply, "%*d %d %d", &idCode, &ok);
    res.success = (bool) ok;
    res.msg = (res.success ? "Ok." : "Wrong answer from robot.");
  } else {
    res.success = false;
    res.msg = "No answer received.";
  }
  return true;
}
