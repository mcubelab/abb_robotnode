#include "abb_robotnode/RobotController.hpp"

SERVICE_CALLBACK_DEF(AddToJointBuffer)
{
  int randNumber = generateRandNumber();

  // Pre-checks
  if(egmMode != EGM_OFF) {
    res.success = false;
    res.msg = "Current EGM mode is incompatible with this task.";
    return true;
  }
  if(!prepareJoints(motionMsg, 37, randNumber, req.joints)) {
    res.success = false;
    res.msg = "Sent joints are not valid.";
    return true;
  }

  if(sendMotion(randNumber)) {
    int ok, idCode;
    sscanf(motionReply, "%*d %d %d", &idCode, &ok);
    res.success = (bool) ok;
    res.msg = (res.success ? "Ok." : "Wrong answer from robot.");
  } else {
    res.success = false;
    res.msg = "No answer received.";
  }
  return true;
}
