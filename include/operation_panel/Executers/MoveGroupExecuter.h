#ifndef MOVEGROUPEXECUTER_H
#define MOVEGROUPEXECUTER_H

#include <operation_panel/Executers/IExecuter.h>
#include <moveit/move_group_interface/move_group.h>
#include <operation_panel/Plans/MoveGroupMotionPlan.h>
#include <operation_panel/Executers/ExecutionWaiter.h>

#include <operation_panel/GripperControllers/WSG50_Controller.h>

class MoveGroupExecuter //: public IExecuter
{
    
public:
    
    MoveGroupExecuter(moveit::planning_interface::MoveGroup*, ICollisionObjectController*, IGripperController* _egn_160_controller);
    
    void SetMotionPlan(MoveGroupMotionPlan*);
    
    void Execute();
    
private:
    
    moveit::planning_interface::MoveGroup* move_group;
    
    ExecutionWaiter* waiter;
    
    MoveGroupMotionPlan* motion_plan;
    
    ICollisionObjectController* object_controller;
    IGripperController* egn_160_controller;
    
    ICollisionObjectCommand* attach;
    ICollisionObjectCommand* detach;
    ICollisionObjectCommand* add;
    ICollisionObjectCommand* remove;
    ICollisionObjectCommand* empty;
    
};

#endif // MOVEGROUPEXECUTER_H
