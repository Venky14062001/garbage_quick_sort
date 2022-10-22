# Garbage Quick Sort
Robot arm that helps user to sort garbage based on plastic, cardboard and metal
## Hardware
* Robotis Dynamixel Robot Servo
* x86_64 Computer with NVIDIA GPU
* Arduino
* Suction Pump
* HCSR04 Ultrasonic Distance Sensor
* USB 2.0 Webcam (640 x 480) resolution

## Software
* Arduino IDE
* ROS Noetic
* Pytorch 1.10.0+cu111
* TorchVision 0.11.0+cu111

Check [here](https://pytorch.org/get-started/previous-versions/) for more versions of Pytorch+CUDA. Get the version that compatible with your GPU
## Get started, setup ROS workspace
```
cd <ROS_WORKSPCE>/src
git clone https://github.com/Venky14062001/garbage_quick_sort.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

## Setup Arduino
```
cd ~/Arduino/libraries/
rosrun rosserial_arduino make_libraries.py .
```

## Run
```
roslaunch garbage_quick_sort_robot gqs_robot_bring_up.launch
rosrun garbage_quick_sort_robot_state_machine gqsMainStateMachine
```
