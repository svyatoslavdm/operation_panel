#ifndef OPEN_BAG_MOTION_PLANNER_H_
#define OPEN_BAG_MOTION_PLANNER_H_

#include <ros/ros.h>

#include <operation_panel/ServersManagers/TFServerManager.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>

class OpenBagMotionPlanner
{
public:
    OpenBagMotionPlanner(boost::shared_ptr<TFServer>);
    void Viewpoint(); 
    void PlanBagOpeningMovement();
    void Execution();
    void DeleteTrajectory();    
    
private:
//     ros::NodeHandle nh;
    
    boost::shared_ptr< moveit::planning_interface::MoveGroup > groupPtr;
    const robot_state::JointModelGroup* joint_model_group;
    moveit::planning_interface::MoveGroup::Plan plan;
//     std::vector<Eigen::Vector3f> points;
//     Eigen::Quaternionf q_gripper;
    
//     std::vector<std::vector<geometry_msgs::Pose>> waypointsPoses;
    boost::shared_ptr<TFServer> tfServer;
   
    void WaitForExecution();
    std::vector<geometry_msgs::Pose> CalcOpeningTrajectory(tf::Transform, tf::Transform, tf::Transform);
};

#endif // OPEN_BAG_MOTION_PLANNER_H_
