# Kiwi-Drive (three omni-directional wheel drive)

## ROS rviz and gazebo simulation
![](https://github.com/syedjameel/kiwi_drive/blob/master/gifs/kiwi_locomotion.gif)

## Visualization of kinematics of kiwi drive
![](https://github.com/syedjameel/kiwi_drive/blob/master/scripts/gifs/kiwi_kinematics_basic.gif)


## Requirements
1. ros-noetic
2. gazebo11


## Download the following readily build libraries:
#### To add

### How to Run
   ```shell
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/syedjameel/kiwi_drive.git
    git clone https://github.com/YugAjmera/teleop_keyboard_omni3
    cd ~/catkin_ws
    catkin_make

    roslaunch kiwi_drive gazebo_simulation.launch
    rosrun teleop_keyboard_omni3 teleop_keyboard_omni3.py
  ```

