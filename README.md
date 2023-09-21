# Kiwi-Drive (three omni-directional wheel drive)

## ROS rviz and gazebo simulation
![](https://github.com/syedjameel/kiwi_ws/blob/master/src/kiwi_drive/gifs/kiwi_locomotion.gif)

## Visualization of kinematics of kiwi drive
![](https://github.com/syedjameel/kiwi_ws/blob/master/src/kiwi_drive/scripts/gifs/kiwi_kinematics_basic.gif)


## Requirements
1. ros-noetic
2. gazebo11


## Download the following readily build libraries:
#### To add

### How to Run

#### 1. Setup
   ```shell
    git clone https://github.com/syedjameel/kiwi_ws.git
    cd kiwi_ws
    catkin_make
    source devel/setup.bash
  ```

#### 2. run simulation with teleop keyboard
    *Run all of the following in seperate sourced terminals*

    ```shell
        roslaunch kiwi_drive gazebo_simulation.launch
        rosrun kiwi_path kiwi_odom_path_node
        rosrun teleop_keyboard_omni3 teleop_keyboard_omni3.py
    ```

#### 3. run controls with pid control and move to pose function
    *Run all of the following in seperate sourced terminals*

    ```shell
        roslaunch kiwi_control kiwi_tune_pid.launch
    ```
