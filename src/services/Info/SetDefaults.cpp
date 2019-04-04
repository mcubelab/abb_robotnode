#include "abb_robotnode/RobotController.hpp"

int RobotController::setDefaults() {
  double defWOx, defWOy, defWOz, defWOq0, defWOqx, defWOqy, defWOqz;
  double defTx, defTy, defTz, defTq0, defTqx, defTqy, defTqz;
  double defTmass, defTCGx, defTCGy, defTCGz;
  double defTIx, defTIy, defTIz;
  double defMotionSupervision;
  double defAccelerationStart, defAccelerationEnd;

  int zone, result;
  double speedTCP, speedORI, speedJoints;

  // WorkObject
  node->getParam(root + "/workobjectX", defWOx);
  node->getParam(root + "/workobjectY", defWOy);
  node->getParam(root + "/workobjectZ", defWOz);
  node->getParam(root + "/workobjectQ0", defWOq0);
  node->getParam(root + "/workobjectQX", defWOqx);
  node->getParam(root + "/workobjectQY", defWOqy);
  node->getParam(root + "/workobjectQZ", defWOqz);
  result = setWorkObject(defWOx, defWOy, defWOz, defWOq0, defWOqx, defWOqy, defWOqz);
  if (result != 1)
    return result;

  // Tool
  node->getParam(root + "/toolX", defTx);
  node->getParam(root + "/toolY", defTy);
  node->getParam(root + "/toolZ", defTz);
  node->getParam(root + "/toolQ0", defTq0);
  node->getParam(root + "/toolQX", defTqx);
  node->getParam(root + "/toolQY", defTqy);
  node->getParam(root + "/toolQZ", defTqz);
  result = setTool(defTx, defTy, defTz, defTq0, defTqx, defTqy, defTqz);
  if (result != 1)
    return result;

  // Inertia
  node->getParam(root + "/toolMass", defTmass);
  node->getParam(root + "/toolCGX", defTCGx);
  node->getParam(root + "/toolCGY", defTCGy);
  node->getParam(root + "/toolCGZ", defTCGz);
  node->getParam(root + "/toolIX", defTIx);
  node->getParam(root + "/toolIY", defTIy);
  node->getParam(root + "/toolIZ", defTIz);
  result = setInertia(defTmass, defTCGx, defTCGy, defTCGz, defTIx, defTIy, defTIz);
  if (result != 1)
    return result;

  // Zone
  node->getParam(root + "/zone", zone);
  result = setZone(zone);
  if (result != 1)
    return result;

  // Speed
  node->getParam(root + "/speedTCP", speedTCP);
  node->getParam(root + "/speedORI", speedORI);
  node->getParam(root + "/speedJoints", speedJoints);
  result = setMaxSpeed(speedTCP, speedORI, speedJoints);
  if (result != 1)
    return result;

  // Motion supervision
  node->getParam(root + "/supervision", defMotionSupervision);
  result = setMotionSupervision(defMotionSupervision);
  if (result != 1)
    return result;

  // Acceleration
  node->getParam(root + "/maxAcceleration", defAccelerationStart);
  node->getParam(root + "/maxDeacceleration", defAccelerationEnd);
  result = setMaxAcceleration(defAccelerationStart, defAccelerationEnd);
  if (result != 1)
    return result;

  // If everything is set, our default configuration has been set up correctly
  return 1;
}

SERVICE_CALLBACK_DEF(SetDefaults)
{
  int result = setDefaults();
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
  return true;
}
