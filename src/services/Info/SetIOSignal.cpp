#include "abb_robotnode/RobotController.hpp"

SERVICE_CALLBACK_DEF(SetIOSignal)
{
  int randNumber = generateRandNumber();
  sprintf(infoMsg, "%.2d %.1d %.1d %.1d #", 11, randNumber, req.output_num, req.signal);

  if(sendInfo(randNumber)) {
    int ok, idCode;
    sscanf(infoReply, "%*d %d %d", &idCode, &ok);
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
