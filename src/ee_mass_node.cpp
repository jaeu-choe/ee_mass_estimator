#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <sstream>

#include "least_square.h"
#include "ee_mass_estimator/OdarWrench.h"
#include "ee_mass_estimator/EEMass.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using namespace eigen_to_msgs;

// constant declaration
int n_link = 3;
double const hz = 10.0; // loop rate
double const L1 = 1.04;
double const L2 = 1.01;
double const L3 = 1.01;
double const m1 = 1.9012;
double const m2 = 1.9749;
double const m3 = 1.4950;
double const g_const = 9.807;
Vector3d g = Vector3d(0, 0, -g_const);
double const k0 = 0.5;
double const lambda0 = 1;

// variable declaration
std::vector<Vector3d> p_vicon_vec;
std::vector<Matrix3d> R_vicon_vec;
std::vector<Vector3d> wrench_F;
std::vector<Vector3d> wrench_F0;
std::vector<Vector3d> wrench_T;
double mass, mass_dot;
double P, P_dot, lambda;

// callback function declaration
void link1_pose_callback(geometry_msgs::PoseStamped msg)
{
    p_vicon_vec[0] = msg_to_Vec3d(msg.pose.position);
    R_vicon_vec[0] = msg_to_Quaterniond(msg.pose.orientation).toRotationMatrix();
}

void link2_pose_callback(geometry_msgs::PoseStamped msg)
{
    p_vicon_vec[1] = msg_to_Vec3d(msg.pose.position);
    R_vicon_vec[1] = msg_to_Quaterniond(msg.pose.orientation).toRotationMatrix();
}

void link3_pose_callback(geometry_msgs::PoseStamped msg)
{
    p_vicon_vec[2] = msg_to_Vec3d(msg.pose.position);
    R_vicon_vec[2] = msg_to_Quaterniond(msg.pose.orientation).toRotationMatrix();
}

void link1_wrench_callback(ee_mass_estimator::OdarWrench msg)
{
    wrench_F[0] = Vector3d(msg.linear[0], msg.linear[1], msg.linear[2]);
    wrench_T[0] = Vector3d(msg.angular[0], msg.angular[1], msg.angular[2]);
}

void link2_wrench_callback(ee_mass_estimator::OdarWrench msg)
{
    wrench_F[1] = Vector3d(msg.linear[0], msg.linear[1], msg.linear[2]);
    wrench_T[1] = Vector3d(msg.angular[0], msg.angular[1], msg.angular[2]);
}

void link3_wrench_callback(ee_mass_estimator::OdarWrench msg)
{
    wrench_F[2] = Vector3d(msg.linear[0], msg.linear[1], msg.linear[2]);
    wrench_T[2] = Vector3d(msg.angular[0], msg.angular[1], msg.angular[2]);
}


// main function
int main(int argc, char **argv)
{
    Vector3d p_base, p_ee, link1_vec, link3_vec;
    Vector3d y, y_hat, e1, W;
    p_vicon_vec.resize(n_link);
    R_vicon_vec.resize(n_link);
    wrench_F.resize(n_link);
    wrench_F0.resize(n_link);
    wrench_T.resize(n_link);
    P = 0.1;
    P_dot = 0.0;
    mass = 0.0;
    mass_dot = 0.0;
    lambda = 0.0;

    std::fill(p_vicon_vec.begin(), p_vicon_vec.end(), Vector3d::Zero());
    std::fill(R_vicon_vec.begin(), R_vicon_vec.end(), Matrix3d::Identity());
    std::fill(wrench_F.begin(), wrench_F.end(), Vector3d::Zero());
    std::fill(wrench_T.begin(), wrench_T.end(), Vector3d::Zero());

    ros::init(argc, argv, "ee_mass_estimator");
    ros::NodeHandle n;
    ros::Subscriber link1_pose_sub = n.subscribe("/vrpn_client_node/odar_1/pose", 100, link1_pose_callback);
    ros::Subscriber link2_pose_sub = n.subscribe("/vrpn_client_node/odar_2/pose", 100, link2_pose_callback);
    ros::Subscriber link3_pose_sub = n.subscribe("/vrpn_client_node/odar_3/pose", 100, link3_pose_callback);
    ros::Subscriber link1_wrench_sub = n.subscribe("/mavros_1/odar/wrench", 100, link1_wrench_callback);
    ros::Subscriber link2_wrench_sub = n.subscribe("/mavros_2/odar/wrench", 100, link2_wrench_callback);
    ros::Subscriber link3_wrench_sub = n.subscribe("/mavros_3/odar/wrench", 100, link3_wrench_callback);
    ros::Publisher ee_mass_pub = n.advertise<ee_mass_estimator::EEMass>("/ee_mass_node/mass", 100);
    ros::Rate loop_rate(hz);

    ee_mass_estimator::EEMass msg;
    msg.mass = 0.0;
    int count = 0;
    
    while(ros::ok())
    { 
        // position calculation
        if ((p_vicon_vec[0]==Vector3d(0, 0, 0))&&(p_vicon_vec[1]==Vector3d(0, 0, 0))&&(p_vicon_vec[2]==Vector3d(0, 0, 0)))
        {
            p_base = Vector3d(0, 0, 0);
            p_ee = Vector3d(0, 0, 0);
        }
        else
        {
            link1_vec = L1 / 2 * Vector3d(1, 0, 0);
            p_base = p_vicon_vec[0] - R_vicon_vec[0] * link1_vec;
            p_vicon_vec[0] = p_vicon_vec[0] - p_base;
            p_vicon_vec[1] = p_vicon_vec[1] - p_base;
            p_vicon_vec[2] = p_vicon_vec[2] - p_base; // position from base
            link3_vec = L3 / 2 * Vector3d(1, 0, 0);
            p_ee = p_vicon_vec[2] + R_vicon_vec[2] * link3_vec;
        }

        // force transformation body frame to spatial frame
        wrench_F0[0] = R_vicon_vec[0] * wrench_F[0];
        wrench_F0[1] = R_vicon_vec[1] * wrench_F[1];
        wrench_F0[2] = R_vicon_vec[2] * wrench_F[2];
        
        // bounded gain least square calculation
        W = Vector3d(p_ee[1], -p_ee[0], 0);
        y = p_vicon_vec[0].cross(m1 * g + wrench_F0[0]) + p_vicon_vec[1].cross(m2 * g + wrench_F0[1])
        + p_vicon_vec[2].cross(m3 * g + wrench_F0[2]) + wrench_T[0] + wrench_T[1] + wrench_T[2];
        y = y / g_const;
        y_hat = W * mass;
        e1 = y_hat - y;

        mass_dot = -P * W.transpose() * e1;
        mass += mass_dot / hz;
        lambda = lambda0 * (1 - fabs(P) / k0);
        P_dot = lambda * P - P * (W.transpose() * W)[0] * P;
        P += P_dot / hz;
       
        // publish
        std::stringstream ss;
        //ss << "test " << count << " : " << mass_dot << " " << mass << " " << P << " " << W.transpose() * W 
        //<< " y: " << y.transpose() << " p " << wrench_T[0].transpose(); 
        ss << count << " | mass_dot: " << mass_dot << " mass: " << mass << " P: " << P << " y: " << y_hat.transpose() << " " << y.transpose();
        //ss << count << " " << p_vicon_vec[1].transpose() <<" "<<p_vicon_vec[2].transpose() << p_ee.transpose() ; 
        ROS_INFO("%s", ss.str().c_str());
        
        msg.header.stamp = ros::Time::now();
        msg.header.seq = count;
        msg.mass = mass;
        ee_mass_pub.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}