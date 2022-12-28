#ifndef LEAST_SQUARE_H
#define LEAST_SQUARE_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <ee_mass_estimator/OdarWrench.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>

using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace eigen_to_msgs
{
  geometry_msgs::Point Vec3d_to_msg(Vector3d p)
  {
    geometry_msgs::Point result;
    result.x = p(0);
    result.y = p(1);
    result.z = p(2);
    return result;
  }

  geometry_msgs::Quaternion Quaterniond_to_msg(Quaterniond q)
  {
    geometry_msgs::Quaternion result;
    result.w = q.w();
    result.x = q.x();
    result.y = q.y();
    result.z = q.z();
    return result;
  }

  Vector3d msg_to_Vec3d(geometry_msgs::Point p)
  {
    Vector3d result(p.x, p.y, p.z);
    return result;
  }

  Quaterniond msg_to_Quaterniond(geometry_msgs::Quaternion q)
  {
    // Quaterniond result(Vector4d(q.w, q.x, q.y, q.z));
    Quaterniond result(q.w, q.x, q.y, q.z);
    return result;
  }

}

class bounded_gain
{

};

#endif