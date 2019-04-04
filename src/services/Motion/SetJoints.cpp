#include "abb_robotnode/RobotController.hpp"

SERVICE_CALLBACK_DEF(SetJoints)
{
  // TODO: Validate joints!!
  if(egmMode == EGM_JOINT_POS_AUTO) {
    targetJointsMutex.lock();
    targetJoints.header.seq++;
    targetJoints.position = req.joints;
    targetJointsMutex.unlock();
    res.success = true;
    res.msg = "Ok.";

  } else if(egmMode == EGM_OFF) {
    // If EGM is off, send command via TCP/IP
    int randNumber = generateRandNumber();
    if(prepareJoints(motionMsg, 2, randNumber, req.joints)) {
      if(sendMotion(randNumber)) {
        int ok, idCode;
        sscanf(motionReply, "%*d %d %d", &idCode, &ok);
        res.success = (bool) ok;
        res.msg = (res.success ? "Ok." : "Wrong answer from robot.");
      } else {
        res.success = false;
        res.msg = "No answer received.";
      }
    } else {
      res.success = false;
      res.msg = "Sent joints are not valid.";
    }
  } else {
    res.success = false;
    res.msg = "Current EGM mode is incompatible with this task.";
  }
  return true;
}
