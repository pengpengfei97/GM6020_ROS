# GM6020_ROS

## 1.About
This is a ros package for RoboMaster GM6020 Motor on ubuntu. The code references [Livox-SDK/livox_scanner](https://github.com/Livox-SDK/livox_scanner) and extracts the motor control portion from it, then encapsulates it within the ROS system.

## 2. Hardware Connettion
Before using this package, please ensure that the motor and computer are properly connected and can communicate successfully.

- If you are using NVIDIA's Jetson Xavier series, you can refer to the following article: [Enabling CAN on Nvidia Jetson Xavier Developer Kit](https://medium.com/@ramin.nabati/enabling-can-on-nvidia-jetson-xavier-developer-kit-aaaa3c4d99c9). 

    Chinese version: [在 Nvidia Jetson Xavier 开发者套件上启用 CAN 总线](https://steinslab.io/archives/1712)

- You can also use a USB-to-CAN adapter for the connection. If you are using a USB to CAN adapter, you can input the following command in the terminal:
    ```bash
    sudo modprobe gs_usb && sudo ip link set can0 up type can bitrate 1000000
    ```
    Then use can-utils to read data and test the connection:
    ```bash
    sudo apt-get install can-utils
    can dump can0
    ```
Adjust the DIP switch under the motor, set the motor ID to 7. You can also use other IDs, but please refer to the manual to modify the corresponding value: [Feedback identifier](https://github.com/pengpengfei97/GM6020_ROS/blob/59aea66147ce02a5092528d9b574aa607e1b62a8/robomaster_gm6020_ros/src/motor_ctr.cpp#L260) and [Control identifier](https://github.com/pengpengfei97/GM6020_ROS/blob/59aea66147ce02a5092528d9b574aa607e1b62a8/robomaster_gm6020_ros/src/motor_ctr.cpp#L117)

## 3.Installation

```bash
cd ws/src
git clone git@github.com:pengpengfei97/GM6020_ROS.git
#build
cd ..
catkin_make
```
## 4.Start


```bash
source devel/setup.bash
roslaunch robomaster_gm6020_ros start.launch
```

## 5.Topics
- `/motor_info`

    A custom message type "MotorMessage", you can get the motor's angle information (angle) and rotation direction information (direction, 0: clockwise, 1: counterclockwise).

## 6.Parameters
- `speed`(float,default:1)

    Motor rotation speed.

- `angle`(float,default:180)

    Angle of motor direction change.

- `pid_p`(float,default:10)

    P parameter of the PID controller.

- `pid_i`(float,default:2)

    I parameter of the PID controller.

- `pid_d`(float,default:0)

    D parameter of the PID controller.

- `delay_time`(int,default:1000)

    Delay time for the host to send signals to the motor, it will also affect the topic publishing rate.



## Note 

- The motor will automatically rotate after the node starts. Please ensure safety before starting.
- When the motor rotates to the set angle, it will automatically change direction. If you want it to maintain a single direction of rotation, you can modify function [can_send_thread](https://github.com/pengpengfei97/GM6020_ROS/blob/59aea66147ce02a5092528d9b574aa607e1b62a8/robomaster_gm6020_ros/src/motor_ctr.cpp#L104).
