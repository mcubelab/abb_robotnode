## ABB ROS robot node
ROS package for connecting to ABB robot controllers via TCP/IP, featuring EGM control mode via UDP. Compatible with 6 and 7 DOF ABB robots, including IRB 120, IRB 1600 and IRB 14000 (YuMi). Developed by the MCube Lab at MIT.

A minimal working environment for this package would be given by this `Dockerfile`:

``
  FROM ros:kinetic-ros-base
  RUN apt-get update && apt-get install -y libprotobuf-dev protobuf-compiler protobuf-c-compiler
``
