#include "abb_robotnode/RobotController.hpp"

std::string RobotController::egmModeName() {
  switch(egmMode) {
    case EGM_CART_POS_AUTO:
      return "cartesian pose, automatic";
    case EGM_CART_POS_MANUAL:
      return "cartesian pose, manual";
    case EGM_CART_VEL_MANUAL:
      return "cartesian speed, manual";
    case EGM_JOINT_POS_AUTO:
      return "joint position, automatic";
    case EGM_JOINT_POS_MANUAL:
      return "joint position, manual";
    case EGM_JOINT_VEL_MANUAL:
      return "joint speed, manual";
  }
  return "disabled";
}

SERVICE_CALLBACK_DEF(GetEGMMode)
{
  res.success = true;
  res.mode = egmMode;
  res.msg = "EGM status: " + egmModeName() + ".";
  return true;
}
