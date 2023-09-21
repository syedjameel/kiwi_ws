#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <vector>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>


using namespace std;
double x_radius, y_radius, kx, ky;
string x_func, y_func;

bool path_generator(nav_msgs::Path &path)
{
    double Ts = 0.5;

    path.header.frame_id = "odom";
    path.header.stamp = ros::Time::now();

    for (int i = 0; i < 400; i ++)
    {
        double x, y;
        if (x_func == "cos")
        {
            x = x_radius*cos(M_PI/4 * i * Ts * kx * 0.2);
        }
        else
        {
            x = x_radius*sin(M_PI/4 * i * Ts * kx * 0.2);
        }
        if (y_func == "cos")
        {
            y = y_radius*cos(M_PI/4 * i * Ts * kx * 0.2);
        }
        else
        {
            y = y_radius*sin(M_PI/4 * i * Ts * kx * 0.2);
        }

        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.w = 1.0;

        path.poses.push_back(pose);
    }
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_generator");
    ros::NodeHandle nh("~");

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path", 1);

    ros::param::get("~x_radius", x_radius);
    ros::param::get("~y_radius", y_radius);
    ros::param::get("~kx", kx);
    ros::param::get("~ky", ky);
    ros::param::get("~x_func", x_func);
    ros::param::get("~y_func", y_func);

    cout << x_radius << " " << y_radius << endl;
    nav_msgs::Path path;

    while(!path_generator(path)){}

    ros::Rate r(10);

    while(ros::ok())
    {
        path_pub.publish(path);
        ros::spinOnce();
        r.sleep();
    }

}