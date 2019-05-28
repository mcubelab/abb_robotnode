#include "abb_robotnode/RobotController.hpp"

SERVICE_CALLBACK_DEF(SetIOSignal)
{
  int randNumber = generateRandNumber();
  sprintf(motionMsg, "%.2d %.1d %.1d %.1d #", 11, randNumber, req.output_num, req.signal);

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
