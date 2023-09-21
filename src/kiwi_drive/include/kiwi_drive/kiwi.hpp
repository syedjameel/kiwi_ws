#pragma once

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <string.h>


class Kiwi {
    public:
        Kiwi(ros::NodeHandle *nh, std::string robot_name = "");

        void SetupSubscription();

    private:
        std::string robot_name_;
        std::string motor_b_topic_;
        std::string motor_l_topic_;
        std::string motor_r_topic_;
        std::string cmd_topic_;

    const double KIWI_WHEELBASE = 0.1;  //change it later
    const double KIWI_WHEEL_RADIUS = 0.05;

    ros::NodeHandle *nh_;

    ros::Publisher motor_b_pub_;
    ros::Publisher motor_l_pub_;
    ros::Publisher motor_r_pub_;

    ros::Subscriber cmd_sub_;

    void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
};