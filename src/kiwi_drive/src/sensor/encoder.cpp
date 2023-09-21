#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <kiwi_drive/Velocity.h>

long double duration;

int i;

kiwi_drive::Velocity message;

ros::Publisher wheel_vel_pub;


long double theta_back_current;
long double theta_left_current;
long double theta_right_current;
long double theta_back_previous;
long double theta_left_previous;
long double theta_right_previous;



ros::Time timeCurrent;
ros::Time timePrevious;

long double v_back;
long double v_left;
long double v_right;



void onJointStateMessage(const sensor_msgs::JointState::ConstPtr& input){
    timeCurrent = ros::Time::now();
    for (i = 0; i < input->name.size(); i++) {
        if (input->name[i].c_str()[6] == 'l') {
            theta_left_current = input->position[i];
        } else if (input->name[i].c_str()[6] == 'b') {
            theta_back_current = input->position[i];
        } else if (input->name[i].c_str()[6] == 'r') {
            theta_right_current = input->position[i];
        }
    }
    duration = (timeCurrent - timePrevious).toSec();
    v_left  = (theta_left_current  - theta_left_previous ) / duration;
    v_back  = (theta_back_current  - theta_back_previous ) / duration;
    v_right = (theta_right_current - theta_right_previous) / duration;
    message.v_left  = v_left ;
    message.v_back  = v_back ;
    message.v_right = v_right;
    wheel_vel_pub.publish(message);
    timePrevious = timeCurrent;
    theta_left_previous  = theta_left_current ;
    theta_back_previous  = theta_back_current ;
    theta_right_previous = theta_right_current;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "encoder");
    ros::NodeHandle node;
    while (!ros::Time::waitForValid()) {}
    wheel_vel_pub = node.advertise<kiwi_drive::Velocity>("wheel_velocity", 10);
    ros::Subscriber subscriber = node.subscribe("/joint_states", 10, onJointStateMessage);
    ros::spin();
    return 0;
}