#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <kdl/frames.hpp>
#include <dynamic_reconfigure/server.h>
#include <kiwi_control/pidConfig.h>

#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>


using namespace std;

ros::Time previousTime;
double starttimeSec = 0.0, endtimeSec = 0.0;

double dt = 0.0;
double time_fn(ros::Time currentTime);

// double Kp_x = 10.0; double Kp_y = 10.0; double Kp_w = 15.0;
// double Ki_x = 0.1; double Ki_y = 0.1; double Ki_w = 0.1;
// double Kd_x = 1.0; double Kd_y = 1.0; double Kd_w = 2;

double Kp_x = 72, Kp_y = 72, Kp_w = 62;
double Ki_x = 0.5, Ki_y = 0.5, Ki_w = 0.5;
double Kd_x = 9.5, Kd_y = 9.5, Kd_w = 1.5;

nav_msgs::Odometry odom;
void odomCallback(const nav_msgs::Odometry &msg){odom = msg;}

geometry_msgs::PoseStamped goal;
void goalCallback(const geometry_msgs::PoseStamped &msg){goal = msg;}



double quaternion2Yaw(geometry_msgs::Quaternion orientation)
{
    double q0 = orientation.x;
    double q1 = orientation.y;
    double q2 = orientation.z;
    double q3 = orientation.w;

    // float yaw = atan2(2.0*(q1*q2 + q3*q0), q3*q3 - q0*q0 - q1*q1 + q2*q2);

    double yaw = atan2(2.0*(q2*q3 + q0*q1), 1.0 -2.0*(q1*q1 + q2*q2));
    return yaw;
}

geometry_msgs::Twist PIDcontroller(nav_msgs::Odometry odom, geometry_msgs::PoseStamped goal)
{
    
    double pose[3], vel[3], target[3];

    //Pose
    pose[0] = odom.pose.pose.position.x;
    pose[1] = odom.pose.pose.position.y;
    pose[2] = quaternion2Yaw(odom.pose.pose.orientation);//static_cast<float>(yaw);
    // if (pose[2] > 0){pose[2] -= 1.570796327;}
    // if (pose[2] < 0){pose[2] += 1.570796327;}

    ROS_INFO("Pose:  %f,     %f,    %f", pose[0], pose[1], pose[2]);

    //Velocity
    vel[0] = odom.twist.twist.linear.x;
    vel[1] = odom.twist.twist.linear.y;
    vel[2] = odom.twist.twist.angular.z;

    //Goal
    target[0] = goal.pose.position.x;
    target[1] = goal.pose.position.y;
    target[2] = quaternion2Yaw(goal.pose.orientation);//static_cast<float>(yaw2);
    ROS_INFO("Target:  %f,     %f,    %f", target[0], target[1], target[2]);




    //PID control
    double delta_x = target[0] - pose[0];
    double delta_y = target[1] - pose[1];
    double delta_th = target[2] - pose[2];

    // if (delta_th > M_PI/2 && delta_th <-M_PI/2) 
    // {
    //     Kp_x = 5.0; Kp_y = 5.0; Kp_w = 10.0;
    //     Kd_x = 1.0; Kd_y = 1.0; Kd_w = 1.5;

    // }
    // if (delta_th < M_PI/2 && delta_th >-M_PI/2)
    // {
    //     Kp_x = 30.0; Kp_y = 30.0; Kp_w = 10.0;
    //     Kd_x = 1.2; Kd_y = 1.2; Kd_w = 1.2;

    // }

    ROS_INFO("delta th: %f", delta_th);
    ROS_INFO("Kp_x = %f, Kp_y = %f, Kp_w = %f", Kp_x, Kp_y, Kp_w);
    ROS_INFO("Ki_x = %f, Ki_y = %f, Ki_w = %f", Ki_x, Ki_y, Ki_w);
    ROS_INFO("Kd_x = %f, Kd_y = %f, Kd_w = %f", Kd_x, Kd_y, Kd_w);

    double prop_x = Kp_x*delta_x;
    double prop_y = Kp_y*delta_y;
    double prop_th = Kp_w*delta_th;

 

    double integ_x = Ki_x*delta_x*dt;
    double integ_y = Ki_y*delta_y*dt;
    double integ_th = Ki_w*delta_th*dt;


    double diff_x = (Kd_x*delta_x)/dt;
    double diff_y = (Kd_y*delta_y)/dt;
    double diff_th = (Kd_w*delta_th)/dt;

    double control_x = prop_x + integ_x + diff_x;
    double control_y = prop_y + integ_y + diff_y;
    double control_th = prop_th + integ_th + diff_th;


    geometry_msgs::Twist control_twist;
    if (std::sqrt(delta_x*delta_x + delta_y*delta_y) > 0.01 && std::abs(delta_th) > 0.02)
    {
        control_twist.linear.x = control_x*cos(pose[2]) + control_y*sin(pose[2]);
        control_twist.linear.y = -control_x*sin(pose[2]) + control_y*cos(pose[2]);
        control_twist.angular.z = -control_th;
    }
    else
    {
        ROS_INFO("Target is reached!");
        control_twist.linear.x = 0.0;
        control_twist.linear.y = 0.0;
        control_twist.angular.z = 0.0;
    }
    return control_twist;

}

double time_fn(ros::Time currentTime)
{
    double delta_t =  currentTime.toSec() - previousTime.toSec();
    ROS_INFO("total time = %f", delta_t);
    previousTime = currentTime;
    return delta_t;
}

void callback(kiwi_control::pidConfig &config, uint32_t level) {
    // Update your variables with new values from config
    Kp_x = config.Kp_x;
    Kp_y = config.Kp_y;
    Kp_w = config.Kp_w;

    Ki_x = config.Ki_x;
    Ki_y = config.Ki_y;
    Ki_w = config.Ki_w;

    Kd_x = config.Kd_x;
    Kd_y = config.Kd_y;
    Kd_w = config.Kd_w;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move2pose");
    ros::NodeHandle nh("~");

    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 10, goalCallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    dynamic_reconfigure::Server<kiwi_control::pidConfig> server;
    dynamic_reconfigure::Server<kiwi_control::pidConfig>::CallbackType f;

    ros::Rate r(10);
    while(nh.ok())
    {
        // Check the goal for the first time only
        if (goal.header.frame_id == "")
        {
            ros::spinOnce();
            r.sleep();
        }

        geometry_msgs::Twist vel;

        f = boost::bind(&callback, _1, _2);
        server.setCallback(f);

        vel = PIDcontroller(odom, goal);
        cmd_vel_pub.publish(vel);

        dt = time_fn(ros::Time::now());
        ros::spinOnce();
        r.sleep();

    }
    return 0;
}