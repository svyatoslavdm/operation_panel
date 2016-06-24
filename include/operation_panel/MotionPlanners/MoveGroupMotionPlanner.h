#ifndef MOVEGROUPMOTIONPLANNER_H_
#define MOVEGROUPMOTIONPLANNER_H_

#include <operation_panel/Plans/MoveGroupMotionPlan.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_interface/planning_interface.h>

#include <operation_panel/GraspsFilters/IGraspsFilter.h>
#include <operation_panel/GraspsFilters/CollisionFilter.h>
#include <operation_panel/GraspsFilters/ReachAbilityFilter.h>

#include <operation_panel/CollisionObjects/ICollisionObjectCommand.h>
#include <operation_panel/CollisionObjects/ICollisionObjectController.h>
#include <operation_panel/CollisionObjects/CollisionObjectUtilities.h>

#include <fstream>

class MoveGroupMotionPlanner 
{
public:
    
    MoveGroupMotionPlanner(moveit::planning_interface::MoveGroup*, TFServer*, planning_scene_monitor::PlanningSceneMonitorPtr, ICollisionObjectController*);    
    
    void SetPlan(MoveGroupMotionPlan*);
    
    bool Plan();
    
    void StopPlan();
    
private:
    
    bool object_attached;
    bool first_trajectory;
    
    bool stop_plan;
    
    const std::string PLANNING_SCENE_SERVICE = "get_planning_scene"; 
    
    std::vector<geometry_msgs::Pose> grasp_candidates;
    
    std::vector<std::vector<geometry_msgs::Pose> > tool_trajectories;
    
    std::vector<moveit::planning_interface::MoveGroup::Plan> plans;
    
    std::vector<IGraspsFilter*> grasps_filters;
    
    std::vector<ICollisionObjectCommand*> collision_object_commands;
    
    std::vector<IGripperCommand*> gripper_commands;
    
    std::vector<bool> cartesian;
    
    ICollisionObjectController* object_controller;
    
    TFServer* tfServer;
        
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
    planning_scene::PlanningScenePtr planning_scene_ptr;
    
    moveit::planning_interface::MoveGroup* move_group;
    
    robot_state::RobotState* target_state;
    trajectory_msgs::JointTrajectoryPoint prev_point;
    trajectory_msgs::JointTrajectoryPoint curr_point;
    trajectory_msgs::JointTrajectoryPoint last_point;
    MoveGroupMotionPlan* motion_plan;    
    
    ICollisionObjectCommand* attach;
    ICollisionObjectCommand* detach;
    ICollisionObjectCommand* add;
    ICollisionObjectCommand* remove;
    ICollisionObjectCommand* empty;
    

    bool CheckPose(geometry_msgs::Pose);
    void CheckObjectCommand(ICollisionObjectCommand*);
    void SetStartStateToCurrent();
    bool SetStartStateToPose(geometry_msgs::Pose _pose);
    void SetStartStateToTarget();
    void SetTargetStateToCurrent();
    bool SetTargetStateToPose(geometry_msgs::Pose _pose);
    void ReadMotionPlan();
    void WriteMotionPlan();
    void SetCurrentGrasp(std::vector<geometry_msgs::Pose>);
    void RemoveCurrentGrasp();
    
    bool CheckJointTrajectory(moveit::planning_interface::MoveGroup::Plan);

    
    std::ofstream* file_stream;
    
    void CreateLogFile();
    void CloseLogFile();
    void LogPlan(moveit::planning_interface::MoveGroup::Plan);
};


#endif