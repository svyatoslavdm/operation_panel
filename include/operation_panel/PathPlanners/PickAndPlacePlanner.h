#ifndef PICKANDPLACEPLANNER_H
#define PICKANDPLACEPLANNER_H

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <operation_panel/Plans/MoveGroupMotionPlan.h>

#include <operation_panel/Servers/TFServer.h>

#include <operation_panel/GripperControllers/IGripperController.h>

#include <operation_panel/Readers/IGraspReader.h>
#include <operation_panel/Readers/FileReader.h>

#include <operation_panel/GraspsFilters/ValidStateFilter.h>

#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <typeinfo>


class PickAndPlacePlanner //: public IPathPlanner
{
public:
    
    PickAndPlacePlanner(ros::NodeHandle, TFServer*, IGripperController* , MoveGroupMotionPlan*, 
			moveit::planning_interface::MoveGroup*, planning_scene_monitor::PlanningSceneMonitorPtr,
			ICollisionObjectController*);
    
    void Plan();
    
    void SetPath(std::string);
    
    void SetPlanningType(bool);
    
private:
    
    std::string file_path;
    bool manual_grasp_choosing;
    
    ros::NodeHandle nh;
    
    TFServer* tfServer;
    boost::shared_ptr<TFServer> tfServerPtr;
    
    IGripperController* gripper_controller;
    ICollisionObjectController* collision_object_controller;
    MoveGroupMotionPlan* motion_plan;
    
    std::vector <IGripperCommand*> gripper_commands;
    std::vector <ICollisionObjectCommand*> collision_object_commands;
    
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
    planning_scene::PlanningScenePtr planning_scene_ptr;
    
    moveit::planning_interface::MoveGroup* move_group;
    
    std::vector<geometry_msgs::Pose> grasp_candidates;
    std::vector<std::vector<geometry_msgs::Pose> > tool_trajectories;
    
    ICollisionObjectCommand* attach;
    ICollisionObjectCommand* detach;
    ICollisionObjectCommand* add;
    ICollisionObjectCommand* remove;
    ICollisionObjectCommand* empty;
    
    IGripperCommand* close;
    IGripperCommand* open;
    IGripperCommand* empty_gripper;
    
    void CleanTFs();
    
    void ComputeObjectPath();
    void ReadGrasps();
    void SetGraspCandidate();
    void ComputeToolTrajectories();
    void FilterTrajectories(int grasp_count, int point_count);
    void AddCommands();
    
    void GetGraspCandidates();
    void ComputePregraspPoint(int grasp_count, int point_count);
    bool CheckPose(geometry_msgs::Pose _pose);
    
    void RemoveTrajectories(int _grasp_count, int _point_count);
    
};

#endif // PICKANDPLACEPLANNER_H
