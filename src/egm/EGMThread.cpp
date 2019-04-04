/*
  abb_robotnode: EGM thread
*/

#include "abb_robotnode/RobotController.hpp"
#include <abb_robotnode/egm/PracticalSocket.h>

#pragma comment(lib, "libprotobuf.lib")

void RobotController::egmMain()
{
  // Target working frequency
  ros::Rate rate(egmHz);

  // ROS side (inherited from EGMControl/ROSHelper)
  targetPoseHandler = node->subscribe(root + "_TargetPose", 1, &RobotController::egmLoadManualTargetPose, this);
  targetTwistHandler = node->subscribe(root + "_TargetTwist", 1, &RobotController::egmLoadManualTargetTwist, this);
  targetJointsHandler = node->subscribe(root + "_TargetJoints", 1, &RobotController::egmLoadManualTargetJoints, this);

  // For debugging only (consider to remove after tests)
  sentPoseHandler = node->advertise<geometry_msgs::PoseStamped>(root + "_SentPose", 1);
  sentTwistHandler = node->advertise<geometry_msgs::TwistStamped>(root + "_SentTwist", 1);
  sentJointsHandler = node->advertise<sensor_msgs::JointState>(root + "_SentJoints", 1);

  egmSocket = new UDPSocket(egmPort);
  egmFirst = true;

  lastEgmRobot = abb::egm::EgmRobot();
  lastEgmSensor = abb::egm::EgmSensor();

  while(ros::ok()) {
    // Waits signal from service SetEGMMode / function changeEgmMode
    std::unique_lock<std::mutex> lk(egmStartMutex);
    egmStartCV.wait(lk);

    //ROS_INFO("%s: Starting EGM controller (mode: %d)...", msgHeader.c_str(), egmMode);
    egmConnect();
    egmStartTick = egmGetSensorTick();
    ROS_INFO("%s: EGM started (mode: %s).", msgHeader.c_str(), egmModeName().c_str());

    while (ros::ok() && egmMode)
    {
      ros::spinOnce();
      egmSendTarget();
      try {
        egmFlushRobotData();
        currentPoseHandler.publish(currentPose);
        currentJointsHandler.publish(currentJoints);
      }
      catch (SocketException& e) {
        // EGM is terminated by RAPID (timeout)
        ROS_INFO("%s: EGM terminated: %s", msgHeader.c_str(), e.what());
        egmMode = 0;
        break;
      }
      rate.sleep();
    }
    if(!egmMode) {
      ROS_INFO("%s: EGM terminated by user.", msgHeader.c_str());
    }
    egmDisconnect();
  }

  delete egmSocket;
}

void RobotController::egmLoadManualTargetPose(const geometry_msgs::PoseStamped& data)
{
  if(egmMode == EGM_CART_POS_MANUAL) {
    targetPoseMutex.lock();
    targetPose = data;
    targetPoseMutex.unlock();
  }
}

void RobotController::egmLoadManualTargetTwist(const geometry_msgs::TwistStamped& data)
{
  if(egmMode == EGM_CART_VEL_MANUAL) {
    targetTwistMutex.lock();
    targetTwist = data;
    targetTwistMutex.unlock();
  }
}

void RobotController::egmLoadManualTargetJoints(const sensor_msgs::JointState& data)
{
  if(egmMode == EGM_JOINT_POS_MANUAL || egmMode == EGM_JOINT_VEL_MANUAL) {
    targetJointsMutex.lock();
    targetJoints = data;
    targetJointsMutex.unlock();
  }
}

void RobotController::egmConnect()
{
  egmSocket->setBlocking();
  egmTargetSeq = 0;
  egmSensorSeq = 0;
  if(!egmFirst) {
    // Forcing default behavior when no input
    targetPoseMutex.lock();
    targetPose.header.seq = 0;
    targetPoseMutex.unlock();
    targetJointsMutex.lock();
    targetJoints.header.seq = 0;
    targetJointsMutex.unlock();
    egmSendTarget();
  } else {
    egmFirst = false;
  }
  egmFlushRobotData();
  egmTargetSeq = 0;
  egmSensorSeq = 0;
  if(egmMode == EGM_CART_POS_AUTO || egmMode == EGM_CART_POS_MANUAL) {
    targetPoseMutex.lock();
    targetPose = currentPose;
    targetPoseMutex.unlock();
    sentPose = currentPose;
  } else if(egmMode == EGM_JOINT_POS_AUTO || egmMode == EGM_JOINT_POS_MANUAL) {
    targetJointsMutex.lock();
    targetJoints = currentJoints;
    targetJointsMutex.unlock();
    if(egmMode == EGM_JOINT_POS_MANUAL)
      sentJoints = currentJoints;
  }
  egmSocket->setTimeout(3, 500000);
}

void RobotController::egmDisconnect() {
  egmSocket->setBlocking();
}

void RobotController::egmFlushRobotData()
{
  egmMsgSize = egmSocket->recvFrom(egmReply, MAX_BUFFER-1, egmSourceAddr, egmSourcePort);

  if (egmMsgSize < 0) {
    ROS_INFO("%s: Failed to receive EGM message from robot.", msgHeader.c_str());
    return;
  }
  lastEgmRobot.ParseFromArray(egmReply, egmMsgSize);
  lastEgmFeedBack = lastEgmRobot.feedback();
  egmUpdateCurrentPose();
  egmUpdateCurrentJoints();
}

void RobotController::egmSendTarget()
{
  if(egmMode == EGM_CART_POS_AUTO || egmMode == EGM_CART_POS_MANUAL) {
    egmSendTargetPose();
    sentPoseHandler.publish(sentPose);
  } else if(egmMode == EGM_CART_VEL_MANUAL) {
    egmSendTargetTwist();
    sentTwistHandler.publish(sentTwist);
  } else {
    egmSendTargetJoints();
    sentJointsHandler.publish(sentJoints);
  }
}

void RobotController::egmUpdateCurrentPose()
{
  currentPoseMutex.lock();
  currentPose.header.stamp = ros::Time::now();
  currentPose.header.frame_id = "world";
  currentPose.pose.position.x = lastEgmFeedBack.cartesian().pos().x();
  currentPose.pose.position.y = lastEgmFeedBack.cartesian().pos().y();
  currentPose.pose.position.z = lastEgmFeedBack.cartesian().pos().z();
  currentPose.pose.orientation.x = lastEgmFeedBack.cartesian().orient().u1();
  currentPose.pose.orientation.y = lastEgmFeedBack.cartesian().orient().u2();
  currentPose.pose.orientation.z = lastEgmFeedBack.cartesian().orient().u3();
  currentPose.pose.orientation.w = lastEgmFeedBack.cartesian().orient().u0();
  currentPoseMutex.unlock();
}

void RobotController::egmUpdateCurrentJoints()
{
  currentJointsMutex.lock();
  currentJoints.header.stamp = ros::Time::now();
  for (int i = 0; i < 6; i++)
    currentJoints.position[i] = lastEgmFeedBack.joints().joints(i)*RAD2DEG;
  if(model == IRB14000)
    currentJoints.position[6] =  lastEgmFeedBack.externaljoints().joints(0)*RAD2DEG;
  currentJointsMutex.unlock();
}

void RobotController::egmSendTargetPose()
{
  targetPoseMutex.lock();
  sentPose.header.stamp = ros::Time::now();
  if(targetPose.header.seq != egmTargetSeq) {
    // TODO: add validation
    sentPose.pose = targetPose.pose;
    egmTargetSeq = targetPose.header.seq;
  }
  egmSentPoseToEgmSensor();
  targetPoseMutex.unlock();
  lastEgmSensor.SerializeToString(&egmMsg);
  egmSocket->sendTo(egmMsg.c_str(), egmMsg.length(), egmSourceAddr, egmSourcePort);
}

void RobotController::egmSendTargetTwist()
{
  targetTwistMutex.lock();
  sentTwist.header.stamp = ros::Time::now();
  if(targetTwist.header.seq == egmTargetSeq) {
    sentTwist.twist = geometry_msgs::Twist();
  } else {
    // TODO: add validation
    sentTwist.twist = targetTwist.twist;
    egmTargetSeq = targetTwist.header.seq;
  }
  egmSentTwistToEgmSensor();
  targetTwistMutex.unlock();
  lastEgmSensor.SerializeToString(&egmMsg);
  egmSocket->sendTo(egmMsg.c_str(), egmMsg.length(), egmSourceAddr, egmSourcePort);
}

void RobotController::egmSendTargetJoints()
{
  sentJoints.header.stamp = ros::Time::now();
  targetJointsMutex.lock();
  currentTargetJoints = targetJoints;
  targetJointsMutex.unlock();

  if(egmMode == EGM_JOINT_VEL_MANUAL) {
    sentJoints.position = std::vector<double>();
    if(currentTargetJoints.header.seq == egmTargetSeq) {
      sentJoints.velocity = std::vector<double>((model == IRB14000 ? 7 : 6), 0.0);
    } else {
      // TODO: add validation
      // READ JOINT NAMES!!
      sentJoints.velocity = currentTargetJoints.velocity;
      egmTargetSeq = currentTargetJoints.header.seq;
    }
    egmSentJointsVelocityToEgmSensor();
  } else if(egmMode == EGM_JOINT_POS_AUTO) {
    sentJoints.position = std::vector<double>();
    sentJoints.velocity = std::vector<double>((model == IRB14000 ? 7 : 6), 0.0);
    double maxdj = 0;
    for(int i = 0; i < (model == IRB14000 ? 7 : 6); ++i)
      if(fabs(currentTargetJoints.position[i]-currentJoints.position[i]) > maxdj)
        maxdj = fabs(currentTargetJoints.position[i]-currentJoints.position[i]);

    if(maxdj > 0.5) {
      for(int i = 0; i < (model == IRB14000 ? 7 : 6); ++i) {
        sentJoints.velocity[i] = (currentTargetJoints.position[i]-currentJoints.position[i])/maxdj*maxJointSpeed;
      }
    } else {
      sentJoints.velocity = std::vector<double>((model == IRB14000 ? 7 : 6), 0.0);
    }
    egmSentJointsVelocityToEgmSensor();
  } else {
    if(currentTargetJoints.header.seq != egmTargetSeq) {
      // TODO: add validation
      // READ JOINT NAMES!!
      sentJoints.position = currentTargetJoints.position;
      egmTargetSeq = currentTargetJoints.header.seq;
    }
    egmSentJointsToEgmSensor();
  }
  lastEgmSensor.SerializeToString(&egmMsg);
  egmSocket->sendTo(egmMsg.c_str(), egmMsg.length(), egmSourceAddr, egmSourcePort);
}

uint32_t RobotController::egmGetSensorTick()
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now))
    return 0;
  return now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

void RobotController::egmSentPoseToEgmSensor()
{
  lastEgmSensor.mutable_header()->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
  lastEgmSensor.mutable_header()->set_seqno(egmSensorSeq++);
  lastEgmSensor.mutable_header()->set_tm(egmGetSensorTick()-egmStartTick);
  lastEgmSensor.clear_planned();
  lastEgmSensor.clear_speedref();
  lastEgmSensor.mutable_planned()->mutable_cartesian()->mutable_pos()->set_x(sentPose.pose.position.x);
  lastEgmSensor.mutable_planned()->mutable_cartesian()->mutable_pos()->set_y(sentPose.pose.position.y);
  lastEgmSensor.mutable_planned()->mutable_cartesian()->mutable_pos()->set_z(sentPose.pose.position.z);
  lastEgmSensor.mutable_planned()->mutable_cartesian()->mutable_orient()->set_u0(sentPose.pose.orientation.w);
  lastEgmSensor.mutable_planned()->mutable_cartesian()->mutable_orient()->set_u1(sentPose.pose.orientation.x);
  lastEgmSensor.mutable_planned()->mutable_cartesian()->mutable_orient()->set_u2(sentPose.pose.orientation.y);
  lastEgmSensor.mutable_planned()->mutable_cartesian()->mutable_orient()->set_u3(sentPose.pose.orientation.z);
}

void RobotController::egmSentTwistToEgmSensor()
{
  lastEgmSensor.mutable_header()->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
  lastEgmSensor.mutable_header()->set_seqno(egmSensorSeq++);
  lastEgmSensor.mutable_header()->set_tm(egmGetSensorTick()-egmStartTick);
  lastEgmSensor.clear_planned();
  lastEgmSensor.clear_speedref();
  currentPoseMutex.lock();
  lastEgmSensor.mutable_planned()->mutable_cartesian()->mutable_pos()->set_x(currentPose.pose.position.x);
  lastEgmSensor.mutable_planned()->mutable_cartesian()->mutable_pos()->set_y(currentPose.pose.position.y);
  lastEgmSensor.mutable_planned()->mutable_cartesian()->mutable_pos()->set_z(currentPose.pose.position.z);
  lastEgmSensor.mutable_planned()->mutable_cartesian()->mutable_orient()->set_u0(currentPose.pose.orientation.w);
  lastEgmSensor.mutable_planned()->mutable_cartesian()->mutable_orient()->set_u1(currentPose.pose.orientation.x);
  lastEgmSensor.mutable_planned()->mutable_cartesian()->mutable_orient()->set_u2(currentPose.pose.orientation.y);
  lastEgmSensor.mutable_planned()->mutable_cartesian()->mutable_orient()->set_u3(currentPose.pose.orientation.z);
  currentPoseMutex.unlock();
  lastEgmSensor.mutable_speedref()->mutable_cartesians()->add_value(sentTwist.twist.linear.x);
  lastEgmSensor.mutable_speedref()->mutable_cartesians()->add_value(sentTwist.twist.linear.y);
  lastEgmSensor.mutable_speedref()->mutable_cartesians()->add_value(sentTwist.twist.linear.z);
  lastEgmSensor.mutable_speedref()->mutable_cartesians()->add_value(sentTwist.twist.angular.x);
  lastEgmSensor.mutable_speedref()->mutable_cartesians()->add_value(sentTwist.twist.angular.y);
  lastEgmSensor.mutable_speedref()->mutable_cartesians()->add_value(sentTwist.twist.angular.z);
}

void RobotController::egmSentJointsToEgmSensor()
{
  lastEgmSensor.mutable_header()->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
  lastEgmSensor.mutable_header()->set_seqno(egmSensorSeq++);
  lastEgmSensor.mutable_header()->set_tm(egmGetSensorTick()-egmStartTick);
  lastEgmSensor.clear_planned();
  lastEgmSensor.clear_speedref();
  lastEgmSensor.mutable_planned()->mutable_joints()->add_joints(sentJoints.position[0]);
  lastEgmSensor.mutable_planned()->mutable_joints()->add_joints(sentJoints.position[1]);
  lastEgmSensor.mutable_planned()->mutable_joints()->add_joints(sentJoints.position[2]);
  lastEgmSensor.mutable_planned()->mutable_joints()->add_joints(sentJoints.position[3]);
  lastEgmSensor.mutable_planned()->mutable_joints()->add_joints(sentJoints.position[4]);
  lastEgmSensor.mutable_planned()->mutable_joints()->add_joints(sentJoints.position[5]);
  if(model == IRB14000)
    lastEgmSensor.mutable_planned()->mutable_externaljoints()->add_joints(sentJoints.position[6]);
}

void RobotController::egmSentJointsVelocityToEgmSensor()
{
  lastEgmSensor.mutable_header()->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
  lastEgmSensor.mutable_header()->set_seqno(egmSensorSeq++);
  lastEgmSensor.mutable_header()->set_tm(egmGetSensorTick()-egmStartTick);
  lastEgmSensor.clear_planned();
  lastEgmSensor.clear_speedref();
  currentJointsMutex.lock();
  lastEgmSensor.mutable_planned()->mutable_joints()->add_joints(currentJoints.position[0]);
  lastEgmSensor.mutable_planned()->mutable_joints()->add_joints(currentJoints.position[1]);
  lastEgmSensor.mutable_planned()->mutable_joints()->add_joints(currentJoints.position[2]);
  lastEgmSensor.mutable_planned()->mutable_joints()->add_joints(currentJoints.position[3]);
  lastEgmSensor.mutable_planned()->mutable_joints()->add_joints(currentJoints.position[4]);
  lastEgmSensor.mutable_planned()->mutable_joints()->add_joints(currentJoints.position[5]);
  if(model == IRB14000)
    lastEgmSensor.mutable_planned()->mutable_externaljoints()->add_joints(currentJoints.position[6]);
  currentJointsMutex.unlock();
  lastEgmSensor.mutable_speedref()->mutable_joints()->add_joints(sentJoints.velocity[0]);
  lastEgmSensor.mutable_speedref()->mutable_joints()->add_joints(sentJoints.velocity[1]);
  lastEgmSensor.mutable_speedref()->mutable_joints()->add_joints(sentJoints.velocity[2]);
  lastEgmSensor.mutable_speedref()->mutable_joints()->add_joints(sentJoints.velocity[3]);
  lastEgmSensor.mutable_speedref()->mutable_joints()->add_joints(sentJoints.velocity[4]);
  lastEgmSensor.mutable_speedref()->mutable_joints()->add_joints(sentJoints.velocity[5]);
  if(model == IRB14000)
    lastEgmSensor.mutable_speedref()->mutable_externaljoints()->add_joints(sentJoints.position[6]);
}
