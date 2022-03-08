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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ee_mass_estimator");
    ros::NodeHandle n;

    ros::Subscriber odar1_pose_sub = n.subscribe("", 1000, odar1_pose_callback);
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);


    int count = 0;
    while(ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "test" << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        chatter_pub.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}