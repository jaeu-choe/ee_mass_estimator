#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <iostream>
#include <math.h>
#include <sstream>
#include "least_square.h"

using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::Vector3d;
using Eigen::Quaterniond;

// constant declaration
double const L1 = 1.04;
double const L2 = 1.01;
double const L3 = 1.01;
double const m1 = 1.9012;
double const m2 = 1.9749;
double const m3 = 1.4950;
double const g_const = 9.807;
Vector3d g = Vector3d(0, 0, -g_const);

void link1_pose_callback(geometry_msgs::PoseStamped msg)
{
    //p_vicon_vec[0] = msg_to_Vec3d(msg.pose.position);
    //R_vicon_vec[0] = msg_to_Quaterniond(msg.pose.orientation).toRotationMatrix();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ee_mass_estimator");
    ros::NodeHandle n;

    ros::Subscriber link1_pose_sub = n.subscribe("", 1000, link1_pose_callback);
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);


    int count = 0;
    while(ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "test" << count <<g[2];
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}