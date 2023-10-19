# GM6020_ROS

## 1.About
This is a ros package for RoboMaster GM6020 Motor on ubuntu. The code references [Livox-SDK/livox_scanner](https://github.com/Livox-SDK/livox_scanner) and extracts the motor control portion from it, then encapsulates it within the ROS system.

## 2. Hardware Connettion
Before using this package, please ensure that the motor and computer are properly connected and can communicate successfully.

If you are using NVIDIA's Jetson Xavier series, you can refer to the following article: [Enabling CAN on Nvidia Jetson Xavier Developer Kit](https://medium.com/@ramin.nabati/enabling-can-on-nvidia-jetson-xavier-developer-kit-aaaa3c4d99c9). 

Chinese version: [在 Nvidia Jetson Xavier 开发者套件上启用 CAN 总线](https://steinslab.io/archives/1712)

You can also use a USB-to-CAN adapter for the connection.

## 3.Install

```bash
cd ws/src
git clone git@github.com:pengpengfei97/GM6020_ROS.git
#build
cd ..
catkin_make
source devel/setup.bash
```
## 4.Start

```bash
roscore
rosrun robomaster_gm6020_ros motor_ctr_node
```

Subscribing to the topic "motor_info" with a custom message type "MotorMessage", you can get the motor's angle information (angle) and rotation direction information (direction).


## Note 

After the node starts, it will drive the motor to rotate. Please ensure safety before starting.

<br>

---

<br>

### The new version is about to be released, which will provide PID parameter adjustments and rotation speed settings.