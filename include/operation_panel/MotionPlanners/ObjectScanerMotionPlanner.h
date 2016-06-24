#ifndef OBJECT_SCANER_MOTION_PLANNER_H_
#define OBJECT_SCANER_MOTION_PLANNER_H_

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>

class ObjectScanerMotionPlanner
{
public:  
    ObjectScanerMotionPlanner();
    void ArmToViewpoint();
    void RotateTableToStartPosition();
    void RotateTable(int, float);
    
private:
    boost::shared_ptr< moveit::planning_interface::MoveGroup > kuka_group_ptr;
    boost::shared_ptr< moveit::planning_interface::MoveGroup > kawasaki_group_ptr;
    const robot_state::JointModelGroup* kuka_joint_model_group;
    const robot_state::JointModelGroup* kawasaki_joint_model_group;
    moveit::planning_interface::MoveGroup::Plan kuka_plan;
    moveit::planning_interface::MoveGroup::Plan kawasaki_plan;
    
    void KukaExecution();
    void KawasakiExecution();    
    void WaitForKukaExecution();
    void WaitForKawasakiExecution();
};

#endif // OBJECT_SCANER_MOTION_PLANNER_H_
