#ifndef EIGENUTILS_H_
#define EIGENUTILS_H_

#include <geometry_msgs/Pose.h>
#include <Eigen/Eigen>

class EigenUtils 
{
public:
    
    static Eigen::Vector3f to_eigen_pose(geometry_msgs::Point pose);

    static Eigen::Quaternionf to_eigen_quaternion(geometry_msgs::Quaternion orientation);

    static geometry_msgs::Point from_eigen_pose(Eigen::Vector3f eigen_pose);

    static geometry_msgs::Quaternion from_eigen_quaternion(Eigen::Quaternionf eigen_orientation);
    
};

#endif