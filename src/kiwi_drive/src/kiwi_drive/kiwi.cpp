#include "kiwi_drive/kiwi.hpp"

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

Kiwi::Kiwi(ros::NodeHandle *nh, std::string robot_name): nh_(nh), robot_name_(robot_name) {
    motor_b_topic_ = robot_name_ + "/motor_b_controller/command";
    motor_l_topic_ = robot_name_ + "/motor_l_controller/command";
    motor_r_topic_ = robot_name_ + "/motor_r_controller/command";
    cmd_topic_ = robot_name_ + "/cmd_vel";
}

void Kiwi::SetupSubscription(){
    cmd_sub_ = nh_->subscribe<geometry_msgs::Twist>(cmd_topic_, 50, &Kiwi::TwistCmdCallback, this);

    while (cmd_sub_.getNumPublishers() == 0) {
        ROS_INFO("Waiting for node to subscribe to the topic...");
        ros::Duration(1.0).sleep();
    }   
    ROS_INFO("Node is subscribed to the topic '%s'. Now Continuing...", cmd_topic_.c_str());

    motor_b_pub_ = nh_->advertise<std_msgs::Float64>(motor_b_topic_, 50);
    motor_l_pub_ = nh_->advertise<std_msgs::Float64>(motor_l_topic_, 50);
    motor_r_pub_ = nh_->advertise<std_msgs::Float64>(motor_r_topic_, 50);
}

void Kiwi::TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg){
    std_msgs::Float64 motor_cmd[3];

    double vel_x = msg->linear.x;
    double vel_y = msg->linear.y;
    double w_p = msg->angular.z;

    //Robot center to wheel center
    double L = 0.117068;  //0.102068+0.015;


    double v1 = -vel_x/2.0 -(std::sqrt(3.0)/2.0)*vel_y + L*w_p;
    double v2 = vel_x + L*w_p;
    double v3 = -vel_x/2.0 + (std::sqrt(3.0)/2.0)*vel_y + L*w_p;


    // double left_side_velocity =
    // (linear_vel - angular_vel * KIWI_WHEELBASE) / KIWI_WHEEL_RADIUS;
    // double right_side_velocity =
    // (linear_vel + angular_vel * KIWI_WHEELBASE) / KIWI_WHEEL_RADIUS;

    motor_cmd[0].data = v2;
    motor_cmd[1].data = v3;
    motor_cmd[2].data = v1;

    motor_b_pub_.publish(motor_cmd[0]);
    motor_r_pub_.publish(motor_cmd[1]);
    motor_l_pub_.publish(motor_cmd[2]);
}