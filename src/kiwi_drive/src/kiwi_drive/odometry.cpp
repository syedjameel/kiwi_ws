#include <string>

#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>

#include <kdl/frames.hpp>

#include <kiwi_drive/KinematicsForward.h>
#include <kiwi_drive/Velocity.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define _USE_MATH_DEFINES
#include <math.h>

tf2::Quaternion q_orig, q_rot, q_new;

long double duration;

int i;

ros::ServiceClient kinematicsForwardMobile;

ros::ServiceClient kinematicsForwardWorld;

std::string openBaseString = "kiwi";

std::string originString = "base_footprint";

geometry_msgs::Pose2D pose;

geometry_msgs::Pose poseCheat;
geometry_msgs::Twist poseCheatTwist;

geometry_msgs::Pose2D poseMobile;

geometry_msgs::Pose2D poseWorld;

ros::Publisher publisherMobile;

ros::Publisher publisherWorld;

ros::Publisher odom_pub;



long double r;

double rotX, rotY, rotZ;

KDL::Rotation rotation;

kiwi_drive::KinematicsForward service;

ros::Time timeCurrent;
ros::Time timePrevious;


void onEncoderMessage(const kiwi_drive::Velocity::ConstPtr& input){
    service.request.input.v_left  = input->v_left  * r;
    service.request.input.v_back  = input->v_back  * r;
    service.request.input.v_right = input->v_right * r;
    timeCurrent = ros::Time::now();
    duration = (timeCurrent - timePrevious).toSec();
    timePrevious = timeCurrent;
    kinematicsForwardWorld.call(service);
    poseWorld.x     = (service.response.output.x     * duration) + poseWorld.x    ;
    poseWorld.y     = (service.response.output.y     * duration) + poseWorld.y    ;
    poseWorld.theta = (service.response.output.theta * duration) + poseWorld.theta;
    if ((!std::isnan(poseWorld.x)) && (!std::isnan(poseWorld.y)) && (!std::isnan(poseWorld.theta))) {
        publisherWorld.publish(poseWorld);
        poseWorld.x     = poseWorld.x    ;
        poseWorld.y     = poseWorld.y    ;
        poseWorld.theta = poseWorld.theta;
    }
    kinematicsForwardMobile.call(service);
    poseWorld.x     = (service.response.output.x     * duration) + poseMobile.x    ;
    poseWorld.y     = (service.response.output.y     * duration) + poseMobile.y    ;
    poseWorld.theta = (service.response.output.theta * duration) + poseMobile.theta;
    if ((!std::isnan(poseWorld.x)) && (!std::isnan(poseWorld.y)) && (!std::isnan(poseWorld.theta))) {
        publisherMobile.publish(poseWorld);
        poseMobile.x     = poseWorld.x    ;
        poseMobile.y     = poseWorld.y    ;
        poseMobile.theta = poseWorld.theta;
    }
}

void onGazeboMessage(const gazebo_msgs::LinkStates::ConstPtr& input){
    // for (i = 0; i < input->name.size(); i++) {
    //     if (((input->name[i]).find(openBaseString) == std::string::npos) || ((input->name[i]).find(originString) == std::string::npos)) {
    //         continue;
    //     }
        poseCheatTwist = input->twist[1];
        poseCheat = input->pose[1];

        // // Get the original orientation of 'commanded_pose'
        // tf2::convert(poseCheat.orientation , q_orig);

        // double r=0, p=0, y=-M_PI/2;  // Rotate the previous pose by 90 degrees about Z
        // q_rot.setRPY(r, p, y);

        // q_new = q_rot*q_orig;  // Calculate the new orientation

        // q_new.normalize();

        // // Stuff the new rotation back into the pose. This requires conversion into a msg type
        // tf2::convert(q_new, poseCheat.orientation);

        rotation = KDL::Rotation::Quaternion(poseCheat.orientation.x, poseCheat.orientation.y, poseCheat.orientation.z, poseCheat.orientation.w);
        rotation.GetRPY(rotX, rotY, rotZ);



        poseWorld.x     = poseCheat.position.x;
        poseWorld.y     = poseCheat.position.y;
        poseWorld.theta = rotZ;
        publisherWorld.publish(poseWorld);

        // Prepare odometry data
        timeCurrent = ros::Time::now();
        
        geometry_msgs::TransformStamped odom_trans;
        static tf2_ros::TransformBroadcaster br;
        nav_msgs::Odometry odom;
        odom_trans.header.stamp = timeCurrent;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = poseCheat.position.x;
        odom_trans.transform.translation.y = poseCheat.position.y;
        odom_trans.transform.translation.z = 0.0;
        
        odom_trans.transform.rotation.x = poseCheat.orientation.x;
        odom_trans.transform.rotation.y = poseCheat.orientation.y;
        odom_trans.transform.rotation.z = poseCheat.orientation.z;
        odom_trans.transform.rotation.w = poseCheat.orientation.w;

        
        br.sendTransform(odom_trans);
        
        odom.header.stamp = timeCurrent;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = poseCheat.position.x;
        odom.pose.pose.position.y = poseCheat.position.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = poseCheat.orientation.x;
        odom.pose.pose.orientation.y = poseCheat.orientation.y;
        odom.pose.pose.orientation.z = poseCheat.orientation.z;
        odom.pose.pose.orientation.w = poseCheat.orientation.w;
        odom.twist.twist.linear.x = poseCheatTwist.linear.x;
        odom.twist.twist.linear.y = poseCheatTwist.linear.y;
        odom.twist.twist.angular.z = poseCheatTwist.angular.x;

        odom_pub.publish(odom);


    // }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "odometry");
    ros::NodeHandle node;
    while (!ros::Time::waitForValid()) {}
    {
        double parameter;
        if (!node.getParam("parameter/wheel/radius", parameter)) {
            ROS_ERROR("Could not get wheel radius from parameter server.");
            return -1;
        }
        r = parameter;
        if (!node.getParam("parameter/initial/x", parameter)) {
            parameter = 0;
        }
        poseMobile.x = poseWorld.x  = parameter;
        if (!node.getParam("parameter/initial/y", parameter)) {
            parameter = 0;
        }
        poseMobile.y = poseWorld.y  = parameter;
        if (!node.getParam("parameter/initial/theta", parameter)) {
            parameter = 0;
        }
        poseMobile.theta = poseWorld.theta  = parameter;
    }
    ros::Subscriber subscriber;
    {
        std::string poseCheatString = "pose_cheat";
        std::string argument;
        bool poseCheatFound = false;
        for (int j = 0; j < argc; j++) {
            argument = std::string(argv[j]);
            if (poseCheatString.compare(argument) == 0) {
                poseCheatFound = true;
            }
        }
        if (poseCheatFound) {
            // before the gazebotopic is subscribed advertise the odom topic
            odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);
            subscriber = node.subscribe("/gazebo/link_states", 10, onGazeboMessage);
            ROS_INFO("------------------------------- Using /gazebo/link_states for odom ----------------------");
        } else {
            subscriber = node.subscribe("/wheel_velocity", 10, onEncoderMessage);
            ROS_INFO("------------------------------- Using /wheel_velocity for odom ----------------------");
        }
    }

    kinematicsForwardWorld  = node.serviceClient<kiwi_drive::KinematicsForward>("kinematics_forward_world" );
    kinematicsForwardMobile = node.serviceClient<kiwi_drive::KinematicsForward>("kinematics_forward_mobile");
    publisherWorld  = node.advertise<geometry_msgs::Pose2D>("pose/world" , 10);
    publisherMobile = node.advertise<geometry_msgs::Pose2D>("pose/mobile", 10);
    ros::spin();
    return 0;
}
