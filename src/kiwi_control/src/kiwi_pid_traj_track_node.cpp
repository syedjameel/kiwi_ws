#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

double Kp_x = 72, Kp_y = 72, Kp_w = 62;
double Ki_x = 0.5, Ki_y = 0.5, Ki_w = 0.5;
double Kd_x = 9.5, Kd_y = 9.5, Kd_w = 1.5;

ros::Time previousTime;
double starttimeSec = 0.0, endtimeSec = 0.0;

double dt = 0.0;
double time_fn(ros::Time currentTime);

nav_msgs::Odometry odom;
void odomCallback(const nav_msgs::Odometry &msg) {odom = msg;}

nav_msgs::Path path;
void pathCallback(const nav_msgs::Path &msg) {path = msg;}

ros::Publisher cmd_vel_pub;

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


double time_fn(ros::Time currentTime)
{
    double delta_t =  currentTime.toSec() - previousTime.toSec();
    ROS_INFO("total time = %f", delta_t);
    previousTime = currentTime;
    return delta_t;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "traj_tracker");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
    ros::Subscriber path_sub = nh.subscribe("/path", 10, pathCallback);

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate r(10);
    int count = 0;
    bool done = false;
    while(nh.ok())
    {
        if (odom.header.frame_id == "" || path.header.frame_id == "")
        {
            ros::spinOnce();
            r.sleep();
            continue;
        }

        while(done == false)
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
            geometry_msgs::PoseStamped goal;
            if(count < path.poses.size()) goal = path.poses[count];
            else goal = path.poses.back();
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


            if (std::sqrt(delta_x*delta_x + delta_y*delta_y) > 0.05 && std::abs(delta_th) > 0.05)
            {
                control_twist.linear.x = control_x*cos(pose[2]) + control_y*sin(pose[2]);
                control_twist.linear.y = -control_x*sin(pose[2]) + control_y*cos(pose[2]);
                control_twist.angular.z = -control_th;
            }
            else if (count == (path.poses.size()-1))
            {
                control_twist.linear.x = 0.0;
                control_twist.linear.y = 0.0;
                control_twist.angular.z = 0.0;
                cmd_vel_pub.publish(control_twist);
                done = true;
            }
            else
            {
                ROS_INFO("Target is reached!");
                // control_twist.linear.x = 0.0;
                // control_twist.linear.y = 0.0;
                // control_twist.angular.z = 0.0;
                // cmd_vel_pub.publish(control_twist);
                done = true;
            }
            cmd_vel_pub.publish(control_twist);
            ros::Duration(0.2).sleep();
            dt = time_fn(ros::Time::now());
            ros::spinOnce();
        }
        
        count ++;
        done = false;

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
