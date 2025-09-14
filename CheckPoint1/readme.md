CheckPoint1 consists of two tasks
1. Ubuntu 18.04 setup on Raspberry Pi, ROS melodic installation, SSH remote connection setting
2. Raspberry Pi communication with Arduino uno via ROS

### Task1
1. Download Ubuntu 18.04 img file and burn it into SD card
2. Install Ubuntu on Raspberry Pi and user setting
3. Download ROS and ROS workspace setting (catkin_ws)
4. Set up SSH remote connection via mobaXTerm
5. Use mobaXTerm to test SSH connection by printing out ROS version

### Task2
1. Download Arduino IDE on Raspberry Pi
2. Set up arduino workspace and ROS library
3. Create a c plus plus/python file on ROS workspace and Arduino file on arduino workspace
4. ROS workspace should comprises of following files
   a. node publisher
   b. node subscriber
   c. launch file (roslaunch multiple nodes on one terminal)
5. arduino workspace should comprise of following files
   a. HelloWorld.ino (use for testing arduino connection with Raspberry Pi)
   b. Task2.ino (scbscribe user's input and multiply it by 2 then publish it to Raspberry Pi)

### File structure

```bash
catkin_ws/src/
├── publisher_node.cpp
├── subscriber_node.cpp
├── launch/
│   └── checkpoint1.launch

arduino_ws/
├── HelloWorld/
│   └── HelloWorld.ino
└── Task2/
    └── Task2.ino
```
