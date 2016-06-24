#ifndef MOVEGROUPMOTIONPLAN_H
#define MOVEGROUPMOTIONPLAN_H

// #include "IMotionPlan.h"
#include <operation_panel/GripperCommands/IGripperCommand.h>
#include <moveit/move_group_interface/move_group.h>
#include <operation_panel/CollisionObjects/ICollisionObjectCommand.h>

class MoveGroupMotionPlan //: public IMotionPlan
{
public:
    
    void AddPlan(moveit::planning_interface::MoveGroup::Plan);
    
    void AddGripperCommand(IGripperCommand*);
    
    void AddPoint(geometry_msgs::Pose);
    
    void AddCollisionObjectCommand(ICollisionObjectCommand*);
    
    void AddToolTrajectory(std::vector<geometry_msgs::Pose>);
    
    void AddCartesain(bool);
    
    void Clear();
    
    std::vector<moveit::planning_interface::MoveGroup::Plan> GetPlans();
    
    std::vector<IGripperCommand*> GetGripperCommands();
    
    std::vector<ICollisionObjectCommand*> GetObjectCommands();
    
    std::vector<geometry_msgs::Pose> GetPoints();
    
    std::vector<std::vector<geometry_msgs::Pose> > GetToolTrajectories();
    std::vector<bool> GetCartesian();
    
private:
    
    std::vector<geometry_msgs::Pose> points;  // Точки, через которые должен пройти объект
    
    std::vector<std::vector<geometry_msgs::Pose> > tool_trajectories;
    
    std::vector<IGripperCommand*> gripper_commands; // Команды схавту для каждой точки
    
    std::vector<ICollisionObjectCommand*> object_commands; // Команды колизионных объектов
    
    std::vector<moveit::planning_interface::MoveGroup::Plan> plans; // Непосредственно планы движения робота
    
    std::vector<bool> cartesian;
};

#endif // MOVEGROUPMOTIONPLAN_H