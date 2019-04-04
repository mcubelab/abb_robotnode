#include "abb_robotnode/RobotController.hpp"

SERVICE_CALLBACK_DEF(Stop)
{
  if(!egmMode) {
    // EGM is off
    res.success = false;
    res.msg = "Failed to stop: EGM is disabled.";

  } else if(egmMode == EGM_CART_POS_AUTO) {
    targetPoseMutex.lock();
    targetPose = sentPose;
    targetPoseMutex.unlock();
    res.success = true;
    res.msg = "Ok.";

  } else if(egmMode == EGM_JOINT_POS_AUTO) {
    targetJointsMutex.lock();
    targetJoints = currentJoints;
    targetJointsMutex.unlock();
    res.success = true;
    res.msg = "Ok.";

  } else {
    // Manual mode is on
    res.success = false;
    res.msg = "Failed to stop: manual mode enabled.";
  }
  return true;
}
