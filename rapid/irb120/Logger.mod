MODULE Logger
  ! Configuration (defined in Info)
  PERS string ipController;
  PERS num loggerPort;
  PERS num loggerWaitTime;

  ! Clock sync
  PERS bool startInfo;
  PERS bool startLogger;
  PERS bool startMotion;

  ! Mutex for frame modification
  PERS bool frameMutex;

  ! Frames
  PERS tooldata currentTool;
  PERS wobjdata currentWobj;

  ! Communication vars
  VAR socketdev clientSocket;
  VAR socketdev serverSocket;

  ! Other variables
  VAR string data; ! String to be sent
  VAR robtarget position;
  VAR jointtarget joints;
  VAR bool connected;
  VAR string date;
  VAR string time;
  VAR clock timer;

  ! Handshake between server and client:
  ! - Creates socket.
  ! - Waits for incoming TCP connection.
  PROC ConnectServer()
    VAR string clientIP;
    SocketCreate serverSocket;
    SocketBind serverSocket, ipController, loggerPort;
    SocketListen serverSocket;
    WHILE SocketGetStatus(clientSocket)<>SOCKET_CONNECTED DO
      SocketAccept serverSocket, clientSocket \ClientAddress:=clientIP \Time:=WAIT_MAX;
      IF SocketGetStatus(clientSocket)<>SOCKET_CONNECTED THEN
        TPWrite "Problem serving a connection. Try reconnecting.";
      ENDIF
      ! Wait 0.5 seconds for the next reconnection
      WaitTime 0.5;
    ENDWHILE
    TPWrite "Connected to "+clientIP+":"+NumToStr(loggerPort, 0);
    connected:=TRUE;
  ENDPROC

  PROC main()
    ! Wait until all tasks are ready
    startLogger:=TRUE;
    WaitUntil startInfo \PollRate:=0.01;
    ClkStart timer;

    date:=CDate();
    time:=CTime();

    ! Start connection
    ConnectServer;

    ! Infinite loop
    WHILE TRUE DO
      ! Wait if the frame is being changed
      WHILE (frameMutex) DO
      ENDWHILE

      ! 1) Cartesian coordinates
      ! Avoid change of frame while querying position
      frameMutex:=TRUE;
      position:=CRobT(\Tool:=currentTool \WObj:=currentWobj);
      frameMutex:=FALSE;

      ! Prepare string and send
      data:="# 0 ";
      data:=data+date+" "+time+" ";
      data:=data+NumToStr(ClkRead(timer),2)+" ";
      data:=data+NumToStr(position.trans.x,1)+" ";
      data:=data+NumToStr(position.trans.y,1)+" ";
      data:=data+NumToStr(position.trans.z,1)+" ";
      data:=data+NumToStr(position.rot.q1,3)+" ";
      data:=data+NumToStr(position.rot.q2,3)+" ";
      data:=data+NumToStr(position.rot.q3,3)+" ";
      data:=data+NumToStr(position.rot.q4,3);
      IF connected=TRUE THEN
          SocketSend clientSocket\Str:=data;
      ENDIF

      ! 2) Joint coordinates
      joints:=CJointT();

      ! Prepare string and send
      data:="# 1 ";
      data:=data+date+" "+time+" ";
      data:=data+NumToStr(ClkRead(timer),2)+" ";
      data:=data+NumToStr(joints.robax.rax_1,2)+" ";
      data:=data+NumToStr(joints.robax.rax_2,2)+" ";
      data:=data+NumToStr(joints.robax.rax_3,2)+" ";
      data:=data+NumToStr(joints.robax.rax_4,2)+" ";
      data:=data+NumToStr(joints.robax.rax_5,2)+" ";
      data:=data+NumToStr(joints.robax.rax_6,2);
      !End of string
      IF connected=TRUE THEN
          SocketSend clientSocket\Str:=data;
      ENDIF

      ! Wait for desired frequency
      WaitTime loggerWaitTime;
    ENDWHILE
  ERROR
    IF ERRNO=ERR_SOCK_CLOSED THEN
        TPWrite "Client has closed connection.";
    ELSE
        TPWrite "Connection lost due to unknown problem.";
    ENDIF
    connected:=FALSE;

    ! Closing the server
    SocketClose clientSocket;
    SocketClose serverSocket;

    ! Reinitiate the server
    ConnectServer;
    IF ERRNO=ERR_SOCK_CLOSED THEN
        TRYNEXT;
    ELSE
        RETRY;
    ENDIF
  ENDPROC

ENDMODULE
