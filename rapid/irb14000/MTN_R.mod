MODULE MTN_R
  ! Network configuration
  PERS string ipController;
  PERS num motionPortR;

  ! Frame configuration
  PERS tooldata currentToolR;
  PERS wobjdata currentWobjR;
  PERS speeddata currentSpeedR;
  PERS zonedata currentZoneR;

  ! Clock sync
  PERS bool startInfoR;
  PERS bool startLoggerR;
  PERS bool startMotionR;

  ! Mutex for frame modification
  PERS bool frameMutexR;

  ! Communication vars
  VAR socketdev clientSocket;
  VAR socketdev serverSocket;
  VAR egmident egmID1;
  VAR egmstate egmSt1;
  CONST egm_minmax egm_minmax_lin1:=[-0.1,0.1]; !in mm
  CONST egm_minmax egm_minmax_rot1:=[-0.1,0.1];! in degees
  CONST egm_minmax egm_minmax_joint1:=[-0.1,0.1];
  VAR pose posecorEGM:=[[0,0,0],[1,0,0,0]];
  VAR pose posesenEGM:=[[0,0,0],[1,0,0,0]];
  VAR num positiongain;
  PERS bool egmRunning;

  ! Vars for command parsing
  VAR num instructionCode;
  VAR string idCode;
  VAR num params{10};
  VAR num specialParams{5};
  VAR num nParams;

  VAR num collision;
  VAR robtarget cartesianTarget;
  VAR jointtarget jointsTarget;
  VAR robtarget circPoint;
  VAR bool moveComplete;  ! True when program pointer leaves a Move instruction.

  ! Buffered move variables
  CONST num MAX_BUFFER:=512;
  VAR num BUFFER_POS_C:=0;
  VAR num BUFFER_POS_J:=0;
  VAR num BUFFER_POS_G:=0;
  VAR robtarget bufferTargets{MAX_BUFFER};
  VAR speeddata bufferSpeeds{MAX_BUFFER};
  VAR jointtarget bufferJointPos{MAX_BUFFER};
  VAR speeddata bufferJointSpeeds{MAX_BUFFER};
  VAR num bufferHandPoses{MAX_BUFFER};
  VAR num bufferHandPosesC{MAX_BUFFER};
  VAR num bufferHandPosesJ{MAX_BUFFER};

  ! External axis position variables
  VAR extjoint externalAxis;

  ! Error Handler
  VAR errnum ERR_MOTIONSUP:=-1;
  VAR errnum ikErr;

  ! Interrupt to trap the digital output that signals the need to restart the motion of the robot.
  VAR intnum iMotionReset;

  ! Upper and lower joint bounds, in degrees
  VAR num upper_joint_limits{7};
  VAR num lower_joint_limits{7};

  ! Inverse and forward kinematic results
  VAR jointtarget ikResult;
  VAR robtarget fkResult;

  ! Connection status
  VAR bool connected;        ! Client connected
  VAR bool reconnected;      ! Reconnection during the iteration

  ! Synchronized motion
  PERS tasks all_tasks{2} := [["MTN_L"], ["MTN_R"]];
  VAR syncident sync1;
  VAR syncident sync2;
  VAR syncident sync3;
  VAR syncident sync4;
  PERS num syncReq:=1;
  PERS num syncReqJ:=0;

  ! Possible answers to controller
  VAR num ok;
  CONST num SERVER_BAD_MSG:=0;
  CONST num SERVER_OK:=1;
  CONST num SERVER_COLLISION:=2;
  CONST num SERVER_BAD_IK:=3;
  CONST num SERVER_BAD_FK:=4;

  ! Other variables
  VAR string receivedString;
  VAR string sendString;
  VAR string addString;
  VAR robtarget cartesianPose;
  VAR jointtarget jointsPose;
  VAR bool move;
  VAR clock timer;
  VAR bool isHandCalibrated;
  VAR num handSpeed;
  VAR num handPosition;

  ! Handshake between server and client:
  ! - Creates socket.
  ! - Waits for incoming TCP connection.
  PROC ConnectServer()
    VAR string clientIP;
    SocketCreate serverSocket;
    SocketBind serverSocket, ipController, motionPortR;
    SocketListen serverSocket;
    TPWrite "Waiting incoming connections...";
    WHILE SocketGetStatus(clientSocket)<>SOCKET_CONNECTED DO
      SocketAccept serverSocket, clientSocket \ClientAddress:=clientIP \Time:=WAIT_MAX;
      IF SocketGetStatus(clientSocket)<>SOCKET_CONNECTED THEN
        TPWrite "Problem serving a connection. Try reconnecting.";
      ENDIF
      ! Wait 0.5 seconds for the next reconnection
      WaitTime 0.5;
    ENDWHILE
    TPWrite "Connected to "+clientIP+":"+NumToStr(motionPortR, 0);
    connected:=TRUE;

    syncReq:=0;
    syncReqJ:=0;
  ENDPROC

  ! Method to parse the message received from a PC through the socket
  ! Loads values on:
  ! - instructionCode.
  ! - idCode: 3 digit identifier of the command.
  ! - nParams: Number of received parameters.
  ! - params{nParams}: Vector of received params.
  PROC ParseMsg(string msg)
    VAR bool auxOk;
    VAR num ind:=1;
    VAR num newInd;
    VAR num length;
    VAR num indParam:=1;
    VAR string subString;
    VAR bool end:=FALSE;

    length:=StrMatch(msg,1,"#");
    IF length>StrLen(msg) THEN
      !Corrupt message
      nParams:=-1;
    ELSE
      ! Find Instruction code
      newInd:=StrMatch(msg,ind," ")+1;
      subString:=StrPart(msg,ind,newInd-ind-1);
      auxOk:=StrToVal(subString,instructionCode);
      IF auxOk=FALSE THEN
        !Corrupt instruction code
        nParams:=-1;
      ELSE
        ind:=newInd;

        !Find Id Code
        newInd:=StrMatch(msg,ind," ")+1;
        idCode:=StrPart(msg,ind,newInd-ind-1);
        ind:=newInd;

        !Set of parameters (maximum of 8)
        WHILE end=FALSE DO
          newInd:=StrMatch(msg,ind," ")+1;
          IF newInd>length THEN
            end:=TRUE;
          ELSE
            subString:=StrPart(msg,ind,newInd-ind-1);
            auxOk:=StrToVal(subString,params{indParam});
            indParam:=indParam+1;
            ind:=newInd;
          ENDIF
        ENDWHILE
        nParams:=indParam-1;
      ENDIF
    ENDIF
  ENDPROC

  ! Gets joint limits so they can be used later
  PROC ReadJointLimits()
    VAR string path;

    ! Get all of the joint bounds for later use
    FOR i FROM 1 TO 7 DO
      path:="MOC/ARM/rob_R_"+NumToStr(i,0);
      ReadCfgData path,"upper_joint_bound",upper_joint_limits{i};
      ReadCfgData path,"lower_joint_bound",lower_joint_limits{i};

      ! The joint limits are in radians, so convert these to degrees
      upper_joint_limits{i}:=upper_joint_limits{i}*180.0/pi;
      lower_joint_limits{i}:=lower_joint_limits{i}*180.0/pi;
    ENDFOR
  ENDPROC


  PROC main()
    ! Book error number for error handler
    BookErrNo ERR_MOTIONSUP;

    ! Configure the interrupt "iMotionReset"
    ! to traps a raise on the digital output "USER_RESET_MOTION"
    ! Meant to signal the need to restart the motion of the robot.
    ! SetDO USER_RESET_MOTION, 0;
    CONNECT iMotionReset WITH resetMotion;
    !ISignalDO USER_RESET_MOTION,1,iMotionReset;

    ! By default, EGM is not running
    egmRunning:=FALSE;

    ! Initially, we are not changing the frame
    frameMutexR:=FALSE;

    ! Gets joint limits so they can be used later
    ReadJointLimits;

    ! Wait until all tasks are ready
    startMotionR:=TRUE;
    WaitUntil startInfoR \PollRate:=0.01;
    ClkStart timer;

    ! Socket connection
    ConnectServer;

    ! Motion configuration
    MotionSup\On\TuneValue:=100;
    SingArea\Off;
    ! Use \Wrist if we want the robot to charge its course to avoid singularities.
    ConfL\Off;
    ! Use \On if we want the robot to enforce the configuration specified in MoveL
    ConfJ\Off;
    moveComplete:=TRUE;
    collision:=0;


    ! Infinite loop to serve commands
    WHILE TRUE DO
      ! Initialization of program flow variables
      ok:=SERVER_OK;
      reconnected:=FALSE;
      addString:="";

      ! Receive a command
      SocketReceive clientSocket \Str:=receivedString \Time:=WAIT_MAX;
      ParseMsg receivedString;

      ! Execution of the command
      TEST instructionCode
      CASE 0:
        ! Ping
        IF nParams=0 THEN
            ok:=SERVER_OK;
        ELSE
            ok:=SERVER_BAD_MSG;
        ENDIF

      CASE 1:
        ! SetCartesian
        IF nParams=7 THEN
          ! Linear moves. Specify (x, y, z), rotation as quaternion.
          ! For the arm angle, use the current pose
          cartesianPose:=CRobT(\Tool:=currentToolR\WObj:=currentWobjR);
          cartesianTarget:=[[params{1},params{2},params{3}],
                        [params{4},params{5},params{6},params{7}],
                        [0,0,0,0],
                        [cartesianPose.extax.eax_a,9E9,9E9,9E9,9E9,9E9]];
          ok:=SERVER_OK;
          moveComplete:=FALSE;
          MoveL cartesianTarget,currentSpeedR,currentZoneR,currentToolR\WObj:=currentWobjR;
          moveComplete:=TRUE;
        ELSEIF nParams=8 THEN
          ! Linear moves. Specify (x, y, z), rotation as quaternion and
          ! 8th parameter is "arm angle"
          cartesianTarget:=[[params{1},params{2},params{3}],
                        [params{4},params{5},params{6},params{7}],
                        [0,0,0,0],
                        [params{8},9E9,9E9,9E9,9E9,9E9]];
          ok:=SERVER_OK;
          moveComplete:=FALSE;
          MoveL cartesianTarget,currentSpeedR,currentZoneR,currentToolR\WObj:=currentWobjR;
          moveComplete:=TRUE;
        ELSEIF nParams=9 THEN
          ! If there's an extra parameter
          ! it means we want to do a cartesian move interpolating in joints.
          cartesianPose:=CRobT(\Tool:=currentToolR\WObj:=currentWobjR);
          cartesianTarget:=[[params{1},params{2},params{3}],
                         [params{4},params{5},params{6},params{7}],
                         [0,0,0,0],
                         [cartesianPose.extax.eax_a,9E9,9E9,9E9,9E9,9E9]];
          ok:=SERVER_OK;
          moveComplete:=FALSE;
          MoveJ cartesianTarget,currentSpeedR,currentZoneR,currentToolR\WObj:=currentWobjR;
          moveComplete:=TRUE;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 2:
        ! SetJoints
        IF nParams=7 THEN
          move:=true;
          FOR i FROM 1 TO 7 DO
              IF params{i}>upper_joint_limits{i}OR params{i}<lower_joint_limits{i} THEN
                  ! If not, then we'll tell the user that their forward kinematics are invalid
                  ok:=SERVER_BAD_FK;
                  move:=false;
              ENDIF
          ENDFOR
          IF move=TRUE THEN
              jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],
                         [0,9E9,9E9,9E9,9E9,9E9]];
              ok:=SERVER_OK;
              moveComplete:=FALSE;
              MoveAbsJ jointsTarget,currentSpeedR,currentZoneR,currentToolR\Wobj:=currentWobjR;
              moveComplete:=TRUE;
          ENDIF
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 3:
        ! GetCartesian
        IF nParams=0 THEN
          ! Read cartesian pose
          cartesianPose:=CRobT(\Tool:=currentToolR \WObj:=currentWobjR);

          ! Prepare string to send
          addString:=NumToStr(cartesianPose.trans.x,2)+" ";
          addString:=addString+NumToStr(cartesianPose.trans.y,2)+" ";
          addString:=addString+NumToStr(cartesianPose.trans.z,2)+" ";
          addString:=addString+NumToStr(cartesianPose.rot.q1,4)+" ";
          addString:=addString+NumToStr(cartesianPose.rot.q2,4)+" ";
          addString:=addString+NumToStr(cartesianPose.rot.q3,4)+" ";
          addString:=addString+NumToStr(cartesianPose.rot.q4,4);
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 4:
        ! GetJoints
        IF nParams=0 THEN
          ! Read joint coordinates
          jointsPose:=CJointT();

          ! Prepare string to send
          addString:=NumToStr(jointsPose.robax.rax_1,2)+" ";
          addString:=addString+NumToStr(jointsPose.robax.rax_2,2)+" ";
          addString:=addString+NumToStr(jointsPose.robax.rax_3,2)+" ";
          addString:=addString+NumToStr(jointsPose.robax.rax_4,2)+" ";
          addString:=addString+NumToStr(jointsPose.robax.rax_5,2)+" ";
          addString:=addString+NumToStr(jointsPose.robax.rax_6,2)+" ";
          addString:=addString+NumToStr(jointsPose.extax.eax_a,2);
          !End of string
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 5:
        ! SetMotionSupervision
        IF nParams=1 AND params{1}>=0 AND params{1}<=300 THEN
            MotionSup\On\TuneValue:=params{1};
            ok:=SERVER_OK;
        ELSE
            ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 6:
        ! SetTool
        IF nParams=7 THEN
          ! Wait until frame is not being used (e.g. by logger) and then block it
          WHILE (frameMutexR) DO
          ENDWHILE
          frameMutexR:=TRUE;

          ! Set new values and unblock frame again
          currentToolR.tframe.trans.x:=params{1};
          currentToolR.tframe.trans.y:=params{2};
          currentToolR.tframe.trans.z:=params{3};
          currentToolR.tframe.rot.q1:=params{4};
          currentToolR.tframe.rot.q2:=params{5};
          currentToolR.tframe.rot.q3:=params{6};
          currentToolR.tframe.rot.q4:=params{7};
          frameMutexR:=FALSE;
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 7:
        ! SetWorkObject
        IF nParams=7 THEN
          ! Wait until frame is not being used (e.g. by logger) and then block it
          WHILE (frameMutexR) DO
          ENDWHILE
          frameMutexR:=TRUE;
          currentWobjR.oframe.trans.x:=params{1};
          currentWobjR.oframe.trans.y:=params{2};
          currentWobjR.oframe.trans.z:=params{3};
          currentWobjR.oframe.rot.q1:=params{4};
          currentWobjR.oframe.rot.q2:=params{5};
          currentWobjR.oframe.rot.q3:=params{6};
          currentWobjR.oframe.rot.q4:=params{7};
          frameMutexR:=FALSE;
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 8:
        ! SetSpeed
        IF nParams=2 THEN
          currentSpeedR.v_tcp:=params{1};
          currentSpeedR.v_ori:=params{2};
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 9:
        ! SetZone
        IF nParams=4 THEN
          IF params{1}=1 THEN
            currentZoneR.finep:=TRUE;
            currentZoneR.pzone_tcp:=0.0;
            currentZoneR.pzone_ori:=0.0;
            currentZoneR.zone_ori:=0.0;
          ELSE
            currentZoneR.finep:=FALSE;
            currentZoneR.pzone_tcp:=params{2};
            currentZoneR.pzone_ori:=params{3};
            currentZoneR.zone_ori:=params{4};
          ENDIF
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF

      CASE 10:
        ! GetRobotAngle
        IF nParams=0 THEN
            cartesianPose:=CRobT(\Tool:=currentToolR \WObj:=currentWobjR);
            addString:=NumToStr(cartesianPose.extax.eax_a,2);
            ok:=SERVER_OK;
        ELSE
            ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 12:
        ! GetIK
        IF nParams=8 THEN
          ! First, let's make sure the quaternion is normalized
          IF Abs(1.0-Sqrt(params{4}*params{4}+params{5}*params{5}+params{6}*params{6}+params{7}*params{7}))>0.001 THEN
            ! If not, then we cannot find the inverse kinematics for this pose
            ok:=SERVER_BAD_IK;
          ELSE
            ! Otherwise, let's normalize our quaternion
            cartesianPose:=[[params{1},params{2},params{3}],
                NOrient([params{4},params{5},params{6},params{7}]),
                [-1, -1, 0, 11],
                [params{8},9E9,9E9,9E9,9E9,9E9]];
            ok:=SERVER_OK;

            ! Now calculate the joint angles, keeping in mind that if we specified an
            ! impossible configuration, this will generate an error (See error handler below)
            ikResult:=CalcJointT(cartesianPose, currentToolR, \WObj:=currentWobjR, \ErrorNumber:=ikErr);

            ! Store our result in a string to return to the user
            addString:=NumToStr(ikResult.robax.rax_1,2)+" ";
            addString:=addString+NumToStr(ikResult.robax.rax_2,2)+" ";
            addString:=addString+NumToStr(ikResult.robax.rax_3,2)+" ";
            addString:=addString+NumToStr(ikResult.robax.rax_4,2)+" ";
            addString:=addString+NumToStr(ikResult.robax.rax_5,2)+" ";
            addString:=addString+NumToStr(ikResult.robax.rax_6,2)+" ";
            addString:=addString+NumToStr(ikResult.extax.eax_a,2)+" ";
            addString:=addString+NumToStr(ikErr,2);
            !End of string
          ENDIF
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 13:
        ! GetFK
        IF nParams=7 THEN
          ok:=SERVER_OK;

          ! First, let's make sure the specified joint angles are within range
          FOR i FROM 1 TO 7 DO
            IF params{i}>upper_joint_limits{i}OR params{i}<lower_joint_limits{i} THEN
              ! If not, then we'll tell the user that their forward kinematics are invalid
              ok:=SERVER_BAD_FK;
            ENDIF
          ENDFOR

          ! If our joints are within limits, then let's carry on
          IF ok=SERVER_OK THEN
            ! Create a joint target, and then calculate the corresponding cartesian pose
            jointsPose:=[[params{1},params{2},params{3},params{4},params{5},params{6}],
                    [params{7},9E9,9E9,9E9,9E9,9E9]];
            fkResult:=CalcRobT(jointsPose,currentToolR,\WObj:=currentWobjR);

            ! Now add this pose to our return string
            addString:=NumToStr(fkResult.trans.x,2)+" ";
            addString:=addString+NumToStr(fkResult.trans.y,2)+" ";
            addString:=addString+NumToStr(fkResult.trans.z,2)+" ";
            addString:=addString+NumToStr(fkResult.rot.q1,4)+" ";
            addString:=addString+NumToStr(fkResult.rot.q2,4)+" ";
            addString:=addString+NumToStr(fkResult.rot.q3,4)+" ";
            addString:=addString+NumToStr(fkResult.rot.q4,4);
          ENDIF
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 14:
        ! SetInertia
        IF nParams=7 THEN
          currentToolR.tload.mass:=params{1};
          currentToolR.tload.cog.x:=params{2};
          currentToolR.tload.cog.y:=params{3};
          currentToolR.tload.cog.z:=params{4};
          currentToolR.tload.ix:=params{5};
          currentToolR.tload.iy:=params{6};
          currentToolR.tload.iz:=params{7};
          currentToolR.tload.aom.q1:=1.0;
          currentToolR.tload.aom.q2:=0.0;
          currentToolR.tload.aom.q3:=0.0;
          currentToolR.tload.aom.q4:=0.0;
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 15:
        ! SetAcceleration
        IF nParams=2 THEN
          PathAccLim TRUE \AccMax:=params{1},TRUE \DecelMax:=params{2};
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 16:
        ! Hand Jog In
        IF nParams=0 THEN
          Hand_JogInward;
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 17:
        ! Hand Jog Out
        IF nParams=0 THEN
          Hand_JogOutward;
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 18:
        ! Hand Move To
        IF nParams=1 THEN
          Hand_MoveTo params{1};
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 19:
        ! Hand Calibrate
        IF nParams=0 THEN
          Hand_DoCalibrate;
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 20:
        ! Hand Stop
        IF nParams=0 THEN
          Hand_Stop;
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF

      CASE 21:
        ! Set max Speed
        IF nParams=1 THEN
          Hand_SetMaxSpeed params{1};
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 22:
        ! Set gripping force
        IF nParams=1 THEN
          Hand_SetHoldForce params{1};
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 23:
        ! Check if gripper is calibrated
        IF nParams=0 THEN
          isHandCalibrated := Hand_IsCalibrated();
          IF isHandCalibrated THEN
              addString:=NumToStr(1,2);
          ELSE
              addString:=NumToStr(0,2);
          ENDIF
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 24:
        ! Get current gripper position
        IF nParams=0 THEN
          handPosition := Hand_GetActualPos();
          addString:=NumToStr(handPosition,2);
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 25:
        ! Hand Grip Inward
        IF nParams=1 THEN
          Hand_GripInward \holdForce:=params{1};
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 26:
        ! Hand Grip Outward
        IF nParams=1 THEN
          Hand_GripOutward \holdForce:=params{1};
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 30:
        ! AddBuffer
        IF nParams=7 OR nParams=8 THEN
          cartesianTarget:=[[params{1},params{2},params{3}],
                          [params{4},params{5},params{6},params{7}],
                          [0,0,0,0], externalAxis];
          IF BUFFER_POS_C<MAX_BUFFER THEN
            BUFFER_POS_C:=BUFFER_POS_C+1;
            bufferTargets{BUFFER_POS_C}:=cartesianTarget;
            bufferSpeeds{BUFFER_POS_C}:=currentSpeedR;
            IF nParams=8 THEN
              bufferHandPosesC{BUFFER_POS_C}:=params{8};
            ENDIF
          ENDIF
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 31:
        ! ClearBuffer
        IF nParams=0 THEN
          BUFFER_POS_C:=0;
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 32:
        ! GetBufferSize
        IF nParams=0 THEN
          addString:=NumToStr(BUFFER_POS_C,2);
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 33:
        ! ExecuteBuffer
        IF nParams=0 OR nParams=2 THEN
          IF nParams=2 AND params{1}=1 AND syncReq=0 THEN
            ! Sends message now to say it's ready
            sendString:=NumToStr(instructionCode,0);
            sendString:=sendString+" "+idCode;
            sendString:=sendString+" "+NumToStr(ok,0);
            sendString:=sendString+" "+NumToStr(ClkRead(timer),2);
            sendString:=sendString+" "+addString+ByteToStr(10\Char);
            SocketSend clientSocket\Str:=sendString;
            reconnected:=TRUE;
            syncReq:=1;
          ENDIF
          IF nParams=2 AND params{1}=1 THEN
            SyncMoveOn sync1, all_tasks;
          ENDIF
          FOR i FROM 1 TO (BUFFER_POS_C) DO
            IF collision=0 THEN
              moveComplete:=FALSE;
              MoveL bufferTargets{i}, bufferSpeeds{i},currentZoneR,currentToolR\WObj:=currentWobjR;
              IF nParams=2 AND params{2}=1 THEN
                Hand_MoveTo bufferHandPosesC{i};
              ENDIF
              moveComplete:=TRUE;
            ENDIF
          ENDFOR
          IF nParams=2 AND params{1}=1 THEN
            SyncMoveOff sync2;
            syncReq:=0;
          ENDIF
          ok:=SERVER_OK;
        ELSE
            ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 35:
        ! Specify circPoint for circular move, and then wait on toPoint
        IF nParams=7 THEN
          circPoint:=[[params{1},params{2},params{3}],
                  [params{4},params{5},params{6},params{7}],
                  [0,0,0,0],
                  externalAxis];
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 36:
        ! specify toPoint, and use circPoint specified previously
        IF nParams=7 THEN
          cartesianTarget:=[[params{1},params{2},params{3}],
                          [params{4},params{5},params{6},params{7}],
                          [0,0,0,0],
                          externalAxis];
          MoveC circPoint,cartesianTarget,currentSpeedR,currentZoneR,currentToolR\WObj:=currentWobjR;
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 37:
        ! AddJointBuffer
        IF nParams=7 THEN
          jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],
                  [params{7},9E9,9E9,9E9,9E9,9E9]];
          IF BUFFER_POS_J<MAX_BUFFER THEN
              BUFFER_POS_J:=BUFFER_POS_J+1;
              bufferJointPos{BUFFER_POS_J}:=jointsTarget;
              bufferJointSpeeds{BUFFER_POS_J}:=currentSpeedR;
          ENDIF
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 38:
        ! ClearJointBuffer
        IF nParams=0 THEN
          BUFFER_POS_J:=0;
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 39:
      ! GetJointBufferSize
        IF nParams=0 THEN
          addString:=NumToStr(BUFFER_POS_J,2);
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 40:
        ! ExecuteJointBuffer
        IF nParams=1 THEN
          IF params{1}=1 AND syncReqJ=0 THEN
            ! Sends message now to say it's ready
            sendString:=NumToStr(instructionCode,0);
            sendString:=sendString+" "+idCode;
            sendString:=sendString+" "+NumToStr(ok,0);
            sendString:=sendString+" "+NumToStr(ClkRead(timer),2);
            sendString:=sendString+" "+addString+ByteToStr(10\Char);
            SocketSend clientSocket\Str:=sendString;
            reconnected:=TRUE;
            syncReqJ:=1;
          ENDIF
          moveComplete:=FALSE;
          !Trapezoidal velocity
          bufferJointSpeeds{1}.v_tcp:=bufferJointSpeeds{1}.v_tcp*0.50;
          bufferJointSpeeds{1}.v_ori:=bufferJointSpeeds{1}.v_ori*0.50;
          bufferJointSpeeds{2}.v_tcp:=bufferJointSpeeds{2}.v_tcp*0.75;
          bufferJointSpeeds{2}.v_ori:=bufferJointSpeeds{2}.v_ori*0.75;
          bufferJointSpeeds{BUFFER_POS_J-1}.v_tcp:=bufferJointSpeeds{BUFFER_POS_J-1}.v_tcp*0.75;
          bufferJointSpeeds{BUFFER_POS_J-1}.v_ori:=bufferJointSpeeds{BUFFER_POS_J-1}.v_ori*0.75;
          bufferJointSpeeds{BUFFER_POS_J}.v_tcp:=bufferJointSpeeds{BUFFER_POS_J}.v_tcp*0.50;
          bufferJointSpeeds{BUFFER_POS_J}.v_ori:=bufferJointSpeeds{BUFFER_POS_J}.v_ori*0.50;
          !Trapezoidal velocity

          FOR i FROM 1 TO (BUFFER_POS_J) DO
            IF collision=0 THEN
              IF params{1}=1 THEN
                SyncMoveOn sync3, all_tasks;
                IF i=BUFFER_POS_J THEN
                  moveComplete:=FALSE;
                  MoveAbsJ bufferJointPos{i} \ID:=i,bufferJointSpeeds{i},currentZoneR,currentToolR,\Wobj:=currentWobjR;
                  moveComplete:=TRUE;
                ELSE
                  moveComplete:=FALSE;
                  MoveAbsJ bufferJointPos{i} \ID:=i,bufferJointSpeeds{i},z1,currentToolR,\Wobj:=currentWobjR;
                  moveComplete:=TRUE;
                ENDIF
                SyncMoveOff sync4;
              ELSE
                IF i=BUFFER_POS_J THEN
                  moveComplete:=FALSE;
                  MoveAbsJ bufferJointPos{i},bufferJointSpeeds{i},currentZoneR,currentToolR,\Wobj:=currentWobjR;
                  moveComplete:=TRUE;
                ELSE
                  moveComplete:=FALSE;
                  MoveAbsJ bufferJointPos{i},bufferJointSpeeds{i},z1,currentToolR,\Wobj:=currentWobjR;
                  moveComplete:=TRUE;
                ENDIF
              ENDIF
            ENDIF
          ENDFOR
          syncReqJ:=0;
          ok:=SERVER_OK;
        ELSEIF nParams=0 THEN
          moveComplete:=FALSE;
          !Trapezoidal velocity
          bufferJointSpeeds{1}.v_tcp:=bufferJointSpeeds{1}.v_tcp*0.50;
          bufferJointSpeeds{1}.v_ori:=bufferJointSpeeds{1}.v_ori*0.50;
          bufferJointSpeeds{2}.v_tcp:=bufferJointSpeeds{2}.v_tcp*0.75;
          bufferJointSpeeds{2}.v_ori:=bufferJointSpeeds{2}.v_ori*0.75;
          bufferJointSpeeds{BUFFER_POS_J-1}.v_tcp:=bufferJointSpeeds{BUFFER_POS_J-1}.v_tcp*0.75;
          bufferJointSpeeds{BUFFER_POS_J-1}.v_ori:=bufferJointSpeeds{BUFFER_POS_J-1}.v_ori*0.75;
          bufferJointSpeeds{BUFFER_POS_J}.v_tcp:=bufferJointSpeeds{BUFFER_POS_J}.v_tcp*0.50;
          bufferJointSpeeds{BUFFER_POS_J}.v_ori:=bufferJointSpeeds{BUFFER_POS_J}.v_ori*0.50;
          !Trapezoidal velocity

          FOR i FROM 1 TO (BUFFER_POS_J) DO
            IF collision=0 THEN
              IF i=BUFFER_POS_J THEN
                moveComplete:=FALSE;
                MoveAbsJ bufferJointPos{i},bufferJointSpeeds{i},currentZoneR,currentToolR,\Wobj:=currentWobjR;
                moveComplete:=TRUE;
              ELSE
                moveComplete:=FALSE;
                MoveAbsJ bufferJointPos{i},bufferJointSpeeds{i},z1,currentToolR,\Wobj:=currentWobjR;
                moveComplete:=TRUE;
              ENDIF
            ENDIF
          ENDFOR
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 41:
          ! Hand Turn on Blow 1
          IF nParams=0 THEN
            Hand_TurnOnBlow1;
            ok:=SERVER_OK;
          ELSE
            ok:=SERVER_BAD_MSG;
          ENDIF


      CASE 42:
          ! Hand Turn off Blow 1
          IF nParams=0 THEN
            Hand_TurnOffBlow1;
            ok:=SERVER_OK;
          ELSE
            ok:=SERVER_BAD_MSG;
          ENDIF


      CASE 43:
          ! Hand Turn on Vacuum 1
          IF nParams=0 THEN
            Hand_TurnOnVacuum1;
            ok:=SERVER_OK;
          ELSE
            ok:=SERVER_BAD_MSG;
          ENDIF


      CASE 44:
          ! Hand Turn off Vacuum 1
          IF nParams=0 THEN
            Hand_TurnOffVacuum1;
            ok:=SERVER_OK;
          ELSE
            ok:=SERVER_BAD_MSG;
          ENDIF


      CASE 45:
        ! Get vacuum pressure
        IF nParams=0 THEN
          handPosition := Hand_GetVacuumPressure1();
          addString:=NumToStr(handPosition,2);
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 70:
        ! SetEGMMode / ActivateEGM
        IF nParams=2 THEN
          IF params{2}=1 THEN
            ! Velocity mode
            positiongain:=0;
          ELSE
            ! Position mode with lower delay
            positiongain:=1;
          ENDIF

          ! Sending confirmation before activating EGM
          sendString:=NumToStr(instructionCode,0);
          sendString:=sendString+" "+idCode;
          sendString:=sendString+" "+NumToStr(ok,0);
          sendString:=sendString+" "+NumToStr(ClkRead(timer),2);
          sendString:=sendString+" "+addString+ByteToStr(10\Char);
          SocketSend clientSocket\Str:=sendString;

          IF params{1}=1 THEN
            ! Joint mode
            jointsTarget:=CJointT();

            ConfJ\Off;
            SingArea\Off;
            MoveAbsJ jointsTarget, v50, fine, currentToolR \Wobj:=currentWobjR;

            WaitTime 0.5;

            EGMReset egmID1;
            EGMGetId egmID1;
            egmSt1 := EGMGetState(egmID1);
            TPWrite "EGM started (joint, vel: "+NumToStr(params{2}, 0)+").";

            IF egmSt1 <= EGM_STATE_CONNECTED THEN
                ! Set up the EGM data source: UdpUc server using device "EGMtest" and configuration "default"
                EGMSetupUC ROB_R, egmID1, "default", "EGMSensorR" \Joint, \CommTimeout:=1 ;
            ENDIF

            ! Copied examples from IRC5 Application manual
            EGMActJoint egmID1 \J1:=egm_minmax_joint1 \J2:=egm_minmax_joint1 \J3:=egm_minmax_joint1
                               \J4:=egm_minmax_joint1 \J5:=egm_minmax_joint1 \J6:=egm_minmax_joint1
                               \J7:=egm_minmax_joint1 \LpFilter:=20 \MaxSpeedDeviation:=500;

            EGMRunJoint egmID1, EGM_STOP_HOLD \J1 \J2 \J3 \J4 \J5 \J6 \J7 \CondTime:=86400
                               \RampInTime:=0.05 \PosCorrGain:=positiongain;
          ENDIF

          IF egmSt1 = EGM_STATE_CONNECTED THEN
            ! EGM finished by timeout
            EGMReset egmID1;
          ENDIF
          egmRunning:=FALSE;

          ok:=SERVER_OK;
          ! To avoid sending a second confirmation, we set reconnected to TRUE
          reconnected:=TRUE;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 99:
        ! CloseServer
        IF nParams=0 THEN
          TPWrite "Connection closed by client.";
          connected:=FALSE;
          ! Closing the server
          SocketClose clientSocket;
          SocketClose serverSocket;

          ! Reinitiate the server
          ConnectServer;
          connected:=TRUE;
          reconnected:=TRUE;
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      DEFAULT:
        TPWrite "Illegal instruction code";
        ok:=SERVER_BAD_MSG;
      ENDTEST

      ! Finally we compose the acknowledge string to send back to the client
      abort:
      IF connected=TRUE THEN
        IF reconnected=FALSE THEN
          IF collision=1 THEN
            ok:=SERVER_COLLISION;
            TPWrite "Sending message back with collision";
          ENDIF
          sendString:=NumToStr(instructionCode,0);
          sendString:=sendString+" "+idCode;
          sendString:=sendString+" "+NumToStr(ok,0);
          sendString:=sendString+" "+NumToStr(ClkRead(timer),2);
          sendString:=sendString+" "+addString+ByteToStr(10\Char);
          SocketSend clientSocket\Str:=sendString;
        ENDIF
      ENDIF
    ENDWHILE
  ERROR (LONG_JMP_ALL_ERR)
    TEST ERRNO
    CASE ERR_UDPUC_COMM:
      ! No UdpUc data received (exception generated when user stops EGM controller in computer side)
      TPWrite "EGM finished by user.";
      EGMReset egmID1;
      egmRunning:=FALSE;
      sendString:=NumToStr(instructionCode,0);
      sendString:=sendString+" "+idCode;
      sendString:=sendString+" "+NumToStr(ok,0);
      sendString:=sendString+" "+NumToStr(ClkRead(timer),2);
      sendString:=sendString+" "+addString+ByteToStr(10\Char);
      SocketSend clientSocket\Str:=sendString;
      TRYNEXT;
    CASE ERR_MOTIONSUP:
      !TPWrite "Motion supervision error.";
      ! Stop the robot motion
      !StopMove;

      ! Clear the current path from any residual motions in the path queue.
      !ClearPath;

      ! Just in case, set the target pose of the object to current location
      ! When we retry the execution of the program, it will do a MoveL instruction to that target.
      cartesianTarget:=CRobT(\Tool:=currentToolR\WObj:=currentWobjR);
      jointsTarget:=CJointT();

      ! Enable the motion of the robot
      StartMove;

      ! Retry execution of the program.
      !RETRY;
      TRYNEXT;
      !May be we should do here TRYNEXT?

    CASE ERR_SOCK_CLOSED:
      TPWrite "Connection closed by client.";
      connected:=FALSE;

      ! Closing the server
      SocketClose clientSocket;
      SocketClose serverSocket;

      ! Reinitiate the server
      ConnectServer;
      reconnected:=TRUE;
      connected:=TRUE;
      TRYNEXT;

    CASE ERR_ROBLIMIT:
      ! Out of reach cartesian target.
      ok:=SERVER_BAD_IK;

      ! Skip the instruction computing the IK that caused the error
      TRYNEXT;

    DEFAULT:
      TPWrite "Unknown error (" +NumtoStr(ERRNO,0)+ "). Restarting.";
      connected:=FALSE;
      ! Closing the server
      SocketClose clientSocket;
      SocketClose serverSocket;
      ! Reinitiate the server
      ConnectServer;
      reconnected:=TRUE;
      connected:=TRUE;
      RETRY;
    ENDTEST
  ENDPROC

  TRAP resetMotion
      ! Routine triggered when the digital output USER_RESET_MOTION is set to 1.
      ! The trigger is raised by the INIT task when a motion suppervision interrrupt is triggered.
      ! It signals the need to restart the motion of the robot.

      ! Note that the motion encoutered a collision
      ! This will show up in the message reply.
      ok:=SERVER_COLLISION;
      collision:=1;

      IF moveComplete=TRUE THEN
          !Stop the robot motion
          StopMove;

          ! If the move instruction is complete, we do not need to raise ERR_MOTIONSUP
          ! to restart the motion of the robot.
          TPWrite "Motion: ------";
          TPWrite "Motion: Motion suppervision error happened after";
          TPWrite "Motion: move instruction was completed.";

          !We clear the current path, to remove any residual motions in the path queue.
          ClearPath;

          !Restart robot motion execution.
          StartMove;

          TPWrite "Motion: Recovered.";
          TPWrite "Motion: ------";
          ! Prepare system for the next time there is a collision
          !SetDO USER_START_OUTPUT,0;
          !SetDO USER_RESET_MOTION,0;

      ELSE
          !Stop the robot motion
          StopMove;

          TPWrite "Motion: ------";
          TPWrite "Motion: Motion suppervision error happened before";
          TPWrite "Motion: move instruction was completed.";

          !We clear the current path, to remove any residual motions in the path queue.
          ClearPath;

          TPWrite "Motion: Recovered.";
          TPWrite "Motion: ------";

          ! We signal the restart of the robot motion by raising the ERR_MOTIONSUP error.
          ! It will be handled by the error handler in the main procedure.
          ! Prepare system for the next time there is a collision
          !SetDO USER_START_OUTPUT,0;
          !SetDO USER_RESET_MOTION,0;
          RAISE ERR_MOTIONSUP;
      ENDIF


  ERROR
      RAISE ;
  ENDTRAP
ENDMODULE
