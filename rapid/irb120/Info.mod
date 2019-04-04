MODULE Info
  ! Network configuration
  PERS string ipController:="192.168.0.191";
  PERS num infoPort:=5000;
  PERS num motionPort:=5001;
  PERS num loggerPort:=5002;
  PERS num loggerWaitTime:=0.1;

  ! Frame configuration
  PERS tooldata currentTool:=[TRUE,[[0,0,0],[1,0,0,0]],[0.1,[-0.1,8.3,310.4],[1,0,0,0],0.023,0.02,0.036]];
  PERS wobjdata currentWobj:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
  PERS speeddata currentSpeed:=[50,10,5000,1000];
  PERS zonedata currentZone:=[FALSE,0.3,0.3,0.3,0.03,0.3,0.03];

  ! Clock sync
  PERS bool startInfo;
  PERS bool startLogger;
  PERS bool startMotion;

  ! Mutex for frame modification
  PERS bool frameMutex;

  ! Communication vars
  VAR socketdev clientSocket;
  VAR socketdev serverSocket;

  ! Vars for command parsing
  VAR num instructionCode;
  VAR string idCode;
  VAR num params{10};
  VAR num specialParams{5};
  VAR num nParams;

  ! Upper and lower joint bounds, in degrees
  VAR num upper_joint_limits{6};
  VAR num lower_joint_limits{6};

  !// Inverse and forward kinematic results
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

  ! Handshake between server and client:
  ! - Creates socket.
  ! - Waits for incoming TCP connection.
  PROC ConnectServer()
      VAR string clientIP;
      SocketCreate serverSocket;
      SocketBind serverSocket, ipController, infoPort;
      SocketListen serverSocket;
      WHILE SocketGetStatus(clientSocket)<>SOCKET_CONNECTED DO
          SocketAccept serverSocket, clientSocket \ClientAddress:=clientIP \Time:=WAIT_MAX;
          IF SocketGetStatus(clientSocket)<>SOCKET_CONNECTED THEN
              TPWrite "Problem serving a connection. Try reconnecting.";
          ENDIF
          ! Wait 0.5 seconds for the next reconnection
          WaitTime 0.5;
      ENDWHILE
      TPWrite "Connected to "+clientIP+":"+NumToStr(infoPort, 0);
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
    FOR i FROM 1 TO 6 DO
      path:="MOC/ARM/rob1_"+NumToStr(i,0);
      ReadCfgData path,"upper_joint_bound",upper_joint_limits{i};
      ReadCfgData path,"lower_joint_bound",lower_joint_limits{i};

      ! The joint limits are in radians, so convert these to degrees
      upper_joint_limits{i}:=upper_joint_limits{i}*180.0/pi;
      lower_joint_limits{i}:=lower_joint_limits{i}*180.0/pi;
    ENDFOR
  ENDPROC

  ! Main program
  PROC main()
    ! Initially, we are not changing the frame
    frameMutex:=FALSE;

    ! Gets joint limits so they can be used later
    ReadJointLimits;

    ! Wait until all tasks are ready
    WaitUntil startLogger \PollRate:=0.01;
    WaitUntil startMotion \PollRate:=0.01;
    startInfo:=TRUE;
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
          cartesianPose:=CRobT(\Tool:=currentTool \WObj:=currentWobj);

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
          addString:=addString+NumToStr(jointsPose.robax.rax_6,2);
          !End of string
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 6:
        ! SetTool
        IF nParams=7 THEN
          ! Wait until frame is not being used (e.g. by logger) and then block it
          WHILE (frameMutex) DO
          ENDWHILE
          frameMutex:=TRUE;

          ! Set new values and unblock frame again
          currentTool.tframe.trans.x:=params{1};
          currentTool.tframe.trans.y:=params{2};
          currentTool.tframe.trans.z:=params{3};
          currentTool.tframe.rot.q1:=params{4};
          currentTool.tframe.rot.q2:=params{5};
          currentTool.tframe.rot.q3:=params{6};
          currentTool.tframe.rot.q4:=params{7};
          frameMutex:=FALSE;
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 7:
        ! SetWorkObject
        IF nParams=7 THEN
          ! Wait until frame is not being used (e.g. by logger) and then block it
          WHILE (frameMutex) DO
          ENDWHILE
          frameMutex:=TRUE;
          currentWobj.oframe.trans.x:=params{1};
          currentWobj.oframe.trans.y:=params{2};
          currentWobj.oframe.trans.z:=params{3};
          currentWobj.oframe.rot.q1:=params{4};
          currentWobj.oframe.rot.q2:=params{5};
          currentWobj.oframe.rot.q3:=params{6};
          currentWobj.oframe.rot.q4:=params{7};
          frameMutex:=FALSE;
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 8:
        ! SetSpeed
        IF nParams=2 THEN
          currentSpeed.v_tcp:=params{1};
          currentSpeed.v_ori:=params{2};
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 9:
        ! SetZone
        IF nParams=4 THEN
          IF params{1}=1 THEN
            currentZone.finep:=TRUE;
            currentZone.pzone_tcp:=0.0;
            currentZone.pzone_ori:=0.0;
            currentZone.zone_ori:=0.0;
          ELSE
            currentZone.finep:=FALSE;
            currentZone.pzone_tcp:=params{2};
            currentZone.pzone_ori:=params{3};
            currentZone.zone_ori:=params{4};
          ENDIF
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF

      CASE 10:
        ! SetIOSignal
        IF nParams=2 THEN
          IF params{2}=1 THEN
            TEST params{1}
            CASE 0:
              SetDO Output0, 1;
              ok:=SERVER_OK;
            DEFAULT:
              ok:=SERVER_BAD_MSG;
            ENDTEST
          ELSEIF params{2}=0 THEN
            TEST params{1}
              CASE 0:
              SetDO Output0, 0;
              ok:=SERVER_OK;
            DEFAULT:
              ok:=SERVER_BAD_MSG;
            ENDTEST
          ELSE
            ok:=SERVER_BAD_MSG;
          ENDIF
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 12:
        ! GetIK
        IF nParams=7 THEN
          ! First, let's make sure the quaternion is normalized
          IF Abs(1.0-Sqrt(params{4}*params{4}+params{5}*params{5}+params{6}*params{6}+params{7}*params{7}))>0.001 THEN
            ! If not, then we cannot find the inverse kinematics for this pose
            ok:=SERVER_BAD_IK;
          ELSE
            ! Otherwise, let's normalize our quaternion
            cartesianPose:=[[params{1},params{2},params{3}],
                NOrient([params{4},params{5},params{6},params{7}]),
                [0,0,0,0],
                [9E9,9E9,9E9,9E9,9E9,9E9]];
            ok:=SERVER_OK;

            ! Now calculate the joint angles, keeping in mind that if we specified an
            ! impossible configuration, this will generate an error (See error handler below)
            ikResult:=CalcJointT(cartesianPose,currentTool,\WObj:=currentWobj);

            ! Store our result in a string to return to the user
            addString:=NumToStr(ikResult.robax.rax_1,2)+" ";
            addString:=addString+NumToStr(ikResult.robax.rax_2,2)+" ";
            addString:=addString+NumToStr(ikResult.robax.rax_3,2)+" ";
            addString:=addString+NumToStr(ikResult.robax.rax_4,2)+" ";
            addString:=addString+NumToStr(ikResult.robax.rax_5,2)+" ";
            addString:=addString+NumToStr(ikResult.robax.rax_6,2);
            !End of string
          ENDIF
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 13:
        ! GetFK
        IF nParams=6 THEN
          ok:=SERVER_OK;

          ! First, let's make sure the specified joint angles are within range
          FOR i FROM 1 TO 6 DO
            IF params{i}>upper_joint_limits{i}OR params{i}<lower_joint_limits{i} THEN
              ! If not, then we'll tell the user that their forward kinematics are invalid
              ok:=SERVER_BAD_FK;
            ENDIF
          ENDFOR

          ! If our joints are within limits, then let's carry on
          IF ok=SERVER_OK THEN
            ! Create a joint target, and then calculate the corresponding cartesian pose
            jointsPose:=[[params{1},params{2},params{3},params{4},params{5},params{6}],
                    [0,9E9,9E9,9E9,9E9,9E9]];
            fkResult:=CalcRobT(jointsPose,currentTool,\WObj:=currentWobj);

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
          currentTool.tload.mass:=params{1};
          currentTool.tload.cog.x:=params{2};
          currentTool.tload.cog.y:=params{3};
          currentTool.tload.cog.z:=params{4};
          currentTool.tload.ix:=params{5};
          currentTool.tload.iy:=params{6};
          currentTool.tload.iz:=params{7};
          currentTool.tload.aom.q1:=1.0;
          currentTool.tload.aom.q2:=0.0;
          currentTool.tload.aom.q3:=0.0;
          currentTool.tload.aom.q4:=0.0;
          ok:=SERVER_OK;
        ELSE
          ok:=SERVER_BAD_MSG;
        ENDIF


      CASE 99:
        ! CloseServer
        IF nParams=0 THEN
          TPWrite "SERVER: Client has closed connection.";
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

      !Finally we compose the acknowledge string to send back to the client
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
        SocketClose clientSocket;
        SocketClose serverSocket;
        ConnectServer;
        reconnected:=TRUE;
        connected:=TRUE;
        TRYNEXT;
    DEFAULT:
        TPWrite "Unknown error (" +NumtoStr(ERRNO,0)+ "). Restarting.";

        connected:=FALSE;
        SocketClose clientSocket;
        SocketClose serverSocket;
        ConnectServer;
        reconnected:=TRUE;
        connected:=TRUE;
        RETRY;
    ENDTEST
  ENDPROC
ENDMODULE
