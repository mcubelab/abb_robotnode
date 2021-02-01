## ABB ROS robot node
ROS package for connecting to ABB robot controllers via TCP/IP, featuring EGM control mode via UDP. Compatible with 6 and 7 DOF ABB robots, including IRB 120, IRB 1600 and IRB 14000 (YuMi). Developed by the MCube Lab at MIT.

For more information about installation, dependencies and available functions, please [check out the wiki](https://github.com/mcubelab/abb_robotnode/wiki).


## Common issue: protobuf version
If you see protobuf related linking errors during compiling, it is possible that cmake cannot find proto2. Type in terminal:
```
protoc --version
```
If you don't see `libprotoc 2.xx`, then you don't have the right version.
Since proto2 is very old, it is not a good idea to overwrite your system default, which is likely to be protoc3.

The solution is to install protoc2 to a custom location and link abb_robotnode to it.

### Step 1: install protoc2 locally
Download source file from https://github.com/protocolbuffers/protobuf/releases/tag/v2.5.0.
```
wget https://github.com/protocolbuffers/protobuf/releases/download/v2.5.0/protobuf-2.5.0.tar.gz
tar -xvf protobuf-2.5.0.tar.gz
```

Then config it to install at a custom location, such as `/home/yifan/local`:
``` bash
cd protobuf-2.5.0
./configure --prefix=/home/yifan/local
make -j8
make install # you should not need sudo
```
Now the install process will create bin, lib, and include folders in `/home/yifan/local` with the library in it.

### Step 2: Expose the proto2 library
Add the following to your .bashrc. Change `/home/yifan/local` to your local folder:
``` bash
export PATH=/home/yifan/local/bin:$PATH
export C_INCLUDE_PATH=/home/yifan/local/include/:$C_INCLUDE_PATH
export CPLUS_INCLUDE_PATH=/home/yifan/local/include/:$CPLUS_INCLUDE_PATH
export LD_LIBRARY_PATH=/home/yifan/local/lib/:$LD_LIBRARY_PATH
```
Then in terminal
``` bash
source ~/.bashrc
```

### Step 3: Compile abb_robotnode with the new proto2 library
Checkout this branch:
```
cd abb_robotnode
git checkout strict_proto2
```
Modify this line in CMakeLists.txt to match your path:
```
set(CMAKE_PREFIX_PATH /home/yifan/local)
```
Now you can proceed to compile.

