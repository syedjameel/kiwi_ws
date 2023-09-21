#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <string>

#include "kiwi_drive/kiwi.hpp"

int main(int argc, char **argv) {
    // setup ROS node
    ros::init(argc, argv, "kiwi_drive_controller_node");
    ros::NodeHandle node(""), private_node("~");

    Kiwi kiwi_controller(&node, "");
    kiwi_controller.SetupSubscription();
    ros::spin();

    return 0;
}