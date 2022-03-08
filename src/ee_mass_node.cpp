#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <math.h>

using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::Vector3d;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ee_mass_estimator")
    ros::NodeHandle n;

    ros::Rate loop_rate(50);


    int count = 0;
    while(ros::ok())
    {
        
        
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}