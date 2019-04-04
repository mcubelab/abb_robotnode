MODULE INF_R
  ! Network configuration
  PERS string ipController;
  PERS num infoPortR;
  PERS num loggerWaitTime:=0.1;

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
  PERS bool frameMutexL;

  ! Communication vars
  VAR socketdev clientSocket;
  VAR socketdev serverSocket;

  ! Vars for command parsing
  VAR num instructionCode;
  VAR string idCode;
  VAR num params{10};
  VAR num specialParams{5};
  VAR num nParams;

  ! External axis position variables
  VAR extjoint externalAxis;

  ! Error Handler
  VAR errnum ikErr;

  ! Upper and lower joint bounds, in degrees
  VAR num upper_joint_limits{7};
  VAR num lower_joint_limits{7};

  ! Inverse and forward kinematic results
  VAR jointtarget ikResult;
  VAR robtarget fkResult;

  ! Connection status
  VAR bool connected;        ! Client connected
  VAR bool reconnected;      ! Reconnection during the iteration

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
    SocketBind serverSocket, ipController, infoPortR;
    SocketListen serverSocket;
    WHILE SocketGetStatus(clientSocket)<>SOCKET_CONNECTED DO
      SocketAccept serverSocket, clientSocket \ClientAddress:=clientIP \Time:=WAIT_MAX;
      IF SocketGetStatus(clientSocket)<>SOCKET_CONNECTED THEN
        TPWrite "Problem serving a connection. Try reconnecting.";
      ENDIF
      ! Wait 0.5 seconds for the next reconnection
      WaitTime 0.5;
    ENDWHILE
    TPWrite "Connected to "+clientIP+":"+NumToStr(infoPortR, 0);
    connected:=TRUE;
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
    ! Initially, we are not changing the frame
    frameMutexL:=FALSE;

    ! Gets joint limits so they can be used later
    ReadJointLimits;

    ! Wait until all tasks are ready
    WaitUntil startLoggerR \PollRate:=0.01;
    WaitUntil startMotionR \PollRate:=0.01;
    startInfoR:=TRUE;
    ClkStart timer;

    ! Socket connection
    ConnectServer;

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


      CASE 6:
        ! SetTool
        IF nParams=7 THEN
          ! Wait until frame is not being used (e.g. by logger) and then block it
          WHILE (frameMutexL) DO
          ENDWHILE
          frameMutexL:=TRUE;

          ! Set new values and unblock frame again
          currentToolR.tframe.trans.x:=params{1};
          currentToolR.tframe.trans.y:=params{2};
          currentToolR.tframe.trans.z:=params{3};
          currentToolR.tframe.rot.q1:=params{4};
          currentToolR.tframe.rot.q2:=params{5};
          currentToolR.tframe.rot.q3:=params{6};
          currentToolR.tframe.rot.q4:=params{7};
          frameMutexL:=FALSE;
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 7:
        ! SetWorkObject
        IF nParams=7 THEN
          ! Wait until frame is not being used (e.g. by logger) and then block it
          WHILE (frameMutexL) DO
          ENDWHILE
          frameMutexL:=TRUE;
          currentWobjR.oframe.trans.x:=params{1};
          currentWobjR.oframe.trans.y:=params{2};
          currentWobjR.oframe.trans.z:=params{3};
          currentWobjR.oframe.rot.q1:=params{4};
          currentWobjR.oframe.rot.q2:=params{5};
          currentWobjR.oframe.rot.q3:=params{6};
          currentWobjR.oframe.rot.q4:=params{7};
          frameMutexL:=FALSE;
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
ENDMODULE
