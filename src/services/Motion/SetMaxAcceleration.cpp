#include "abb_robotnode/RobotController.hpp"

int RobotController::setMaxAcceleration(double acc, double deacc) {
  int randNumber = generateRandNumber();
  sprintf(motionMsg, "%.2d %.3d %08.2lf %08.2lf #", 15, randNumber, acc, deacc);

  if(sendMotion(randNumber)) {
    int ok, idCode;
    sscanf(motionReply, "%*d %d %d", &idCode, &ok);
    return ((bool) ok ? 1 : -1);
  }
  return 0;
}

SERVICE_CALLBACK_DEF(SetMaxAcceleration)
{
  if(egmMode == EGM_OFF) {
    int result = setMaxAcceleration(req.acc, req.deacc);
    if(result == 1) {
      res.success = true;
      res.msg = "Ok.";
    } else if(result == -1) {
      res.success = false;
      res.msg = "Wrong answer from robot.";
    } else {
      res.success = false;
      res.msg = "No answer received.";
    }
  } else {
    res.success = false;
    res.msg = "EGM needs to be stopped.";
  }
  return true;
}
