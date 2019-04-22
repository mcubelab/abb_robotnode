#include "abb_robotnode/RobotController.hpp"

SERVICE_CALLBACK_DEF(SetEGMMode)
{
  // First, check compatible modes
  if(model == IRB14000 && (req.mode == EGM_CART_POS_AUTO || req.mode == EGM_CART_POS_MANUAL || req.mode == EGM_CART_VEL_MANUAL))
  {
    res.success = false;
    res.msg = "Unsupported mode for this robot model.";
    return true;
  }

  if(req.mode == EGM_CART_POS_AUTO || req.mode == EGM_JOINT_POS_AUTO)
  {
    res.success = false;
    res.msg = "Mode not implemented yet.";
    return true;
  }

  if(egmMode == req.mode) {
    res.success = true;
    res.msg = "No change.";
  } else if((egmMode == EGM_CART_POS_AUTO && req.mode == EGM_CART_POS_MANUAL)
      || (egmMode == EGM_CART_POS_MANUAL && req.mode == EGM_CART_POS_AUTO)
      || (egmMode == EGM_JOINT_POS_AUTO && req.mode == EGM_JOINT_POS_MANUAL)
      || (egmMode == EGM_JOINT_POS_MANUAL && req.mode == EGM_JOINT_POS_AUTO)) {
    egmMode = req.mode;
    res.success = true;
    res.msg = "New EGM status: " + egmModeName() + ".";
  } else {
    // Current and previous modes are different
    // If enabled, stop EGM and wait for confirmation
    if(egmMode) {
      egmMode = 0;

      // Wait for confirmation that EGM has stopped
      if(!receiveMotion(egmRandNumber)) {
        res.success = false;
        res.msg = "EGM was not stopped successfully.";
        return true;
      }
    }

    if(req.mode == EGM_JOINT_POS_AUTO) {
      targetJointsMutex.lock();
      targetJoints = currentJoints;
      // sentJoints = currentJoints;
      targetJointsMutex.unlock();
    } else if(req.mode == EGM_CART_POS_AUTO) {
      targetPoseMutex.lock();
      targetPose = currentPose;
      sentPose = currentPose;
      targetPoseMutex.unlock();
    }

    if(req.mode) {
      {
        std::lock_guard<std::mutex> lk(egmStartMutex);
        egmMode = req.mode;
      }
      egmStartCV.notify_all();

      // Notify robot for EGM activation
      egmRandNumber = generateRandNumber();
      int jointMode = ((req.mode == EGM_JOINT_POS_AUTO || req.mode == EGM_JOINT_POS_MANUAL || req.mode == EGM_JOINT_VEL_MANUAL) ? 1 : 0);
      int velMode = ((req.mode == EGM_JOINT_POS_AUTO || req.mode == EGM_CART_VEL_MANUAL || req.mode == EGM_JOINT_VEL_MANUAL) ? 1 : 0);
      sprintf(motionMsg, "%.2d %.3d %.2d %.2d #", 70, egmRandNumber, jointMode, velMode);

      // Return answer to end-user
      if(sendMotion(egmRandNumber)) {
        int ok, idCode;
        sscanf(motionReply, "%*d %d %d", &idCode, &ok);
        res.success = true;
        res.msg = "New EGM status: " + egmModeName() + ".";
      } else {
        res.success = false;
        res.msg = "EGM was not started successfully.";
      }
    } else {
      res.success = true;
      res.msg = "New EGM status: " + egmModeName() + ".";
    }
  }
  lastEgmTime = ros::Time::now();
  return true;
}
