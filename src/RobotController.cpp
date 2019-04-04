#include "abb_robotnode/RobotController.hpp"

// ------------------------ Initialization ------------------------
RobotController::RobotController(ros::NodeHandle *n)
{
  node = n;
}

RobotController::~RobotController()
{
  for(auto handler : serviceHandlers) {
    handler.shutdown();
  }
}

bool RobotController::init(std::string id)
{
  // Saving prefixes
  name = "robot" + id;
  root = "/robot" + id;
  msgHeader = "ABBrobot" + id;
  motionConnected = false;
  loggerConnected = false;
  egmMode = 0;

  // Read network configuration
  node->getParam(root + "/ip", ip);
  node->getParam(root + "/infoPort", infoPort);
  node->getParam(root + "/motionPort", motionPort);
  node->getParam(root + "/loggerPort", loggerPort);
  node->getParam(root + "/egmPort", egmPort);

  // Read model from parameter
  if(!readModel()) return false;

  // Connection to info robot server
  ROS_INFO("%s: Connecting to info server (%s:%d)...", msgHeader.c_str(), ip.c_str(), infoPort);
  connectServer(infoSocket, infoPort, INFO_SERVER, infoConnected);
  if(!infoConnected) return false;
  ROS_INFO("%s: Connected to info server.", msgHeader.c_str());

  // Connection to motion robot server
  ROS_INFO("%s: Connecting to motion server (%s:%d)...", msgHeader.c_str(), ip.c_str(), motionPort);
  connectServer(motionSocket, motionPort, MOTION_SERVER, motionConnected);
  if(!motionConnected) return false;
  ROS_INFO("%s: Connected to motion server.", msgHeader.c_str());

  prepareLogs();

  egmThread = std::thread(&RobotController::egmMain, this);

  if(setDefaults() != 1) {
    ROS_WARN("%s: Failed to set up default configuration.", msgHeader.c_str());
    return false;
  }

  // Connection to logger robot server
  node->param<bool>(root + "/useLogger", useLogger, true);
  if(useLogger) {
    ROS_INFO("%s: Connecting to logger server (%s:%d)...", msgHeader.c_str(), ip.c_str(), loggerPort);
    connectServer(loggerSocket, loggerPort, LOGGER_SERVER, loggerConnected);
    if(!loggerConnected) return false;
    loggerThread = std::thread(&RobotController::loggerMain, this);
    ROS_INFO("%s: Connected to logger server.", msgHeader.c_str());
  }

  // Advertise ROS services
  ROS_INFO("%s: Advertising ROS services...", msgHeader.c_str());
  advertiseServices();

  // Advertise ROS topics
  ROS_INFO("%s: Advertising ROS topics...", msgHeader.c_str());
  advertiseTopics();

  return true;
}

bool RobotController::readModel()
{
  std::string readModel;
  node->getParam(root + "/model", readModel);
  if(readModel == "irb120")
    model = IRB120;
  else if(readModel == "irb1600")
    model = IRB1600;
  else if(readModel == "irb14000")
    model = IRB14000;
  else {
    ROS_WARN("%s: Unknown robot model (%s).", msgHeader.c_str(), readModel.c_str());
    return false;
  }
  ROS_INFO("%s: Model read successfully (%s = %d).", msgHeader.c_str(), readModel.c_str(), model);
  return true;
}

// ------------------------ ROS ------------------------
void RobotController::advertiseServices()
{
  serviceHandlers = std::vector<ros::ServiceServer>();
  ADVERTISE_SERVICES()
}

void RobotController::advertiseService(std::string name, auto func, int compatibleModels)
{
  if((model & compatibleModels) == model) {
    serviceHandlers.push_back(node->advertiseService(name, func, this));
  } else {
    ROS_INFO("%s service not initialized. Model not compatible. %d notin %d, result is %d", name.c_str(), model, compatibleModels, model & compatibleModels);
  }
}

void RobotController::advertiseTopics()
{
  currentPoseHandler = node->advertise<geometry_msgs::PoseStamped>(name + "_PoseStamped", 100);
  currentJointsHandler = node->advertise<sensor_msgs::JointState>(name + "_JointState", 100);
}

// ------------------------ Communication ------------------------
int RobotController::generateRandNumber()
{
  return (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
}

bool RobotController::connectServer(int& sock, int port, int channel, bool& status)
{
  if(status) {
    ROS_INFO("%s: Channel %d is already connected.", msgHeader.c_str(), channel);
    return false;
  }

  if ((sock = socket(PF_INET, SOCK_STREAM, 0)) == -1)
    ROS_INFO("%s: Problem creating the socket for channel %d. Error %d", msgHeader.c_str(), channel, errno);
  else
  {
    // Now try to connect to the server
    struct sockaddr_in remoteSocket;
    remoteSocket.sin_family = AF_INET;
    remoteSocket.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &remoteSocket.sin_addr.s_addr);
    if(connect(sock, (sockaddr*)&remoteSocket, sizeof(remoteSocket)) == -1) {
      ROS_INFO("%s: Could not connect to channel %d. Error %d.", msgHeader.c_str(), channel, errno);
    } else {
      status = true;
      return true;
    }
  }
  return false;
}

bool RobotController::sendAndReceive(int& sock, int channel, std::mutex& mutex, char *message, int messageLength, char* reply, int idCode)
{
  mutex.lock();
  if (send(sock, message, messageLength, 0) == -1)
  {
    ROS_WARN("%s: Failed to send command in channel %d. Error %d.", msgHeader.c_str(), channel, errno);
  }
  else
  {
    return receive(sock, channel, mutex, message, messageLength, reply, idCode);
  }
  mutex.unlock();
  return false;
}

bool RobotController::receiveOnly(int& sock, int channel, std::mutex& mutex, char *message, int messageLength, char* reply, int idCode)
{
  // mutex.lock();
  bool res = receive(sock, channel, mutex, message, messageLength, reply, idCode);
  // mutex.unlock();
  return res;
}

bool RobotController::receive(int& sock, int channel, std::mutex& mutex, char *message, int messageLength, char* reply, int idCode)
{
  // Read the reply to the message we just sent, and make sure
  // it's not corrupt, and the command was executed successfully
  int t;
  if ((t=recv(sock, reply, MAX_BUFFER-1, 0)) > 0)
  {
    reply[t] = '\0';
    sprintf(errorReply, "%s",reply);
    int ok, rcvIdCode;
    sscanf(reply,"%*d %d %d", &rcvIdCode, &ok);

    if(idCode!=-1)
    {
      if ((ok == SERVER_OK) && (rcvIdCode == idCode))
      {
        mutex.unlock();
        return true;
      }
      else if ((ok == SERVER_COLLISION) && (rcvIdCode == idCode))
      {
        ROS_WARN("%s: Collision detected in channel %d.", msgHeader.c_str(), channel);
        mutex.unlock();
        return false;
      }
      else if ((ok == SERVER_BAD_IK || ok == SERVER_BAD_FK) && (rcvIdCode == idCode))
      {
        mutex.unlock();
        return false;
      }
      else
      {
        ROS_WARN("%s: Corrupt message in channel %d.", msgHeader.c_str(), channel);
        ROS_WARN("msg = %s, reply = %s, idCode = %d, ok = %d, rcvCode = %d", message, reply, idCode, ok, rcvIdCode);
      }
    }
    else
    {
      if (ok == SERVER_OK)
      {
        mutex.unlock();
        return true;
      }
      else if (ok == SERVER_COLLISION)
      {
        ROS_WARN("%s: Collision Detected.", msgHeader.c_str());
        mutex.unlock();
        return false;

      }
      else if ((ok == SERVER_BAD_IK || ok == SERVER_BAD_FK) && (rcvIdCode == idCode))
      {
        mutex.unlock();
        return false;
      }
      else
      {
        ROS_WARN("%s: Corrupt message.", msgHeader.c_str());
        ROS_WARN("msg = %s, reply = %s, idCode = %d, ok = %d, rcvCode = %d", message, reply, idCode, ok, rcvIdCode);
      }
    }
  }
  else{
    ROS_WARN("%s: Failed to receive answer from ABB robot.", msgHeader.c_str());
    exit(1);
  }
  return false;
}

bool RobotController::sendInfo(int randNumber)
{
  return sendAndReceive(infoSocket, INFO_SERVER, infoMutex, infoMsg, strlen(infoMsg), infoReply, randNumber);
}

bool RobotController::sendMotion(int randNumber)
{
  return sendAndReceive(motionSocket, MOTION_SERVER, motionMutex, motionMsg, strlen(motionMsg), motionReply, randNumber);
}

bool RobotController::receiveMotion(int randNumber)
{
  return receiveOnly(motionSocket, MOTION_SERVER, motionMutex, motionMsg, strlen(motionMsg), motionReply, randNumber);
}

void RobotController::prepareLogs()
{
  // Prepares currentPose, receivedPose
  receivedPose.header.frame_id = "world";
  currentPose.header.frame_id = "world";
  sentPose.header.frame_id = "world";
  sentTwist.header.frame_id = "world";

  // Prepares currentJoints, receivedJoints
  if(model == IRB14000) {
    receivedJoints.position = std::vector<double>(7, 0.0);
    receivedJoints.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    currentJoints.position = std::vector<double>(7, 0.0);
    currentJoints.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    targetJoints.position = std::vector<double>(7, 0.0);
    targetJoints.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    sentJoints.position = std::vector<double>(7, 0.0);
    sentJoints.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
  } else {
    receivedJoints.position = std::vector<double>(6, 0.0);
    receivedJoints.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    currentJoints.position = std::vector<double>(6, 0.0);
    currentJoints.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    targetJoints.position = std::vector<double>(6, 0.0);
    targetJoints.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    sentJoints.position = std::vector<double>(6, 0.0);
    sentJoints.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  }
}

// Threads
void RobotController::joinThreads()
{
  if(useLogger) loggerThread.join();
  egmThread.join();
}
