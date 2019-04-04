#include "abb_robotnode/RobotController.hpp"

int RobotController::setMaxSpeed(double tcp, double ori, double joints) {
  maxTcpSpeed = tcp;
  maxOriSpeed = ori;
  maxJointSpeed = joints;

  int randNumber = generateRandNumber();
  sprintf(infoMsg, "%.2d %.3d %08.1lf %08.2lf #", 8, randNumber, maxTcpSpeed, maxOriSpeed);

  if(sendInfo(randNumber)) {
    int ok, idCode;
    sscanf(infoReply, "%*d %d %d", &idCode, &ok);
    return ((bool) ok ? 1 : -1);
  }
  return 0;
}

SERVICE_CALLBACK_DEF(SetMaxSpeed)
{
  if(egmMode == EGM_OFF) {
    int result = setMaxSpeed(req.tcp, req.ori, req.joints);
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
