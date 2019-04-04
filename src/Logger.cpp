/*
  abb_robotnode: Logger thread
  ----------------------------
  This is the main function for our logger thread. We simply start a ROS
  timer event and get it to call our loggerCallback function at a
  specified interval. This exits when ROS shuts down.
*/

#include "abb_robotnode/RobotController.hpp"

void RobotController::loggerMain()
{
  // Create a timer to look at the log data
  ros::Timer loggerTimer;
  loggerTimer = node->createTimer(ros::Duration(0.004), &RobotController::loggerCallback, this);

  // Now simply wait until the program is shut down
  ros::waitForShutdown();
  loggerTimer.stop();
}

void RobotController::loggerCallback(const ros::TimerEvent&)
{
  int t;
  int code;

  // If EGM is running, data will be updated through that thread
  if(egmMode) return;

  // Read all information from the TCP/IP socket
  if ((t = recv(loggerSocket, loggerReply, MAX_BUFFER-1, 0)) > 0)
  {
    // Add an end character to form our string
    loggerReply[t] = '\0';
    loggerReplyPointer = loggerReply;

    // Each message starts with a '#' character. Read messages one at a time
    while((loggerReplyPointer = strchr(loggerReplyPointer,'#')) != NULL)
    {
      // The number after the start character is the type of message
      sscanf(loggerReplyPointer,"# %d", &code);
      switch(code)
      {
        // Cartesian Message
        case 0:
          {
            int nParams = sscanf(loggerReplyPointer,"# %*d %*s %*s %*s %lf %lf %lf %lf %lf %lf %lf",
                &receivedPose.pose.position.x, &receivedPose.pose.position.y, &receivedPose.pose.position.z,
                &receivedPose.pose.orientation.w, &receivedPose.pose.orientation.x, &receivedPose.pose.orientation.y, &receivedPose.pose.orientation.z);
            if(nParams == 7) {
              currentPoseMutex.lock();
              currentPose.pose = receivedPose.pose;
              currentPose.header.stamp = ros::Time::now();
              currentPoseHandler.publish(currentPose);
              currentPoseMutex.unlock();
            }
            break;
          }

        // Joint Message
        case 1:
          {
            if(model == IRB14000) {
              int nParams = sscanf(loggerReplyPointer,"# %*d %*s %*s %*s %lf %lf %lf %lf %lf %lf %lf",
                  &receivedJoints.position[0], &receivedJoints.position[1], &receivedJoints.position[2], &receivedJoints.position[3],
                  &receivedJoints.position[4], &receivedJoints.position[5], &receivedJoints.position[6]);
              if (nParams == 7) {
                currentJointsMutex.lock();
                currentJoints.position = receivedJoints.position;
                currentJoints.header.stamp = ros::Time::now();
                currentJointsHandler.publish(currentJoints);
                currentJointsMutex.unlock();
              }
            } else {
              int nParams = sscanf(loggerReplyPointer,"# %*d %*s %*s %*s %lf %lf %lf %lf %lf %lf",
                  &receivedJoints.position[0], &receivedJoints.position[1], &receivedJoints.position[2], &receivedJoints.position[3],
                  &receivedJoints.position[4], &receivedJoints.position[5]);
              if (nParams == 6) {
                currentJointsMutex.lock();
                currentJoints.position = receivedJoints.position;
                currentJoints.header.stamp = ros::Time::now();
                currentJointsHandler.publish(currentJoints);
                currentJointsMutex.unlock();
              }
            }
            break;
          }
      }
      // Increment loggerReplyPointer, so we don't look at the same message again
      loggerReplyPointer++;
    }
  }
}
