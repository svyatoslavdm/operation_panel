#include <operation_panel/Utilities/EigenUtils.h>

Eigen::Vector3f EigenUtils::to_eigen_pose(geometry_msgs::Point pose)
{
    Eigen::Vector3f eigen_pose;
    eigen_pose << pose.x, pose.y, pose.z;
    return eigen_pose; 
    
}

Eigen::Quaternionf EigenUtils::to_eigen_quaternion(geometry_msgs::Quaternion orientation)
{
    Eigen::Quaternionf eigen_orientation;
    eigen_orientation.x() = orientation.x;
    eigen_orientation.y() = orientation.y;
    eigen_orientation.z() = orientation.z;
    eigen_orientation.w() = orientation.w;
    return eigen_orientation; 
}

geometry_msgs::Point EigenUtils::from_eigen_pose(Eigen::Vector3f eigen_pose)
{
    geometry_msgs::Point pose;
    pose.x = eigen_pose.x();
    pose.y = eigen_pose.y();
    pose.z = eigen_pose.z();
    return pose; 
}

geometry_msgs::Quaternion EigenUtils::from_eigen_quaternion(Eigen::Quaternionf eigen_orientation)
{
    geometry_msgs::Quaternion orientation;
    orientation.x = eigen_orientation.x();
    orientation.y = eigen_orientation.y();
    orientation.z = eigen_orientation.z();
    orientation.w = eigen_orientation.w();
    return orientation; 
}