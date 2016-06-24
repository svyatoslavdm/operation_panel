#ifndef EXECUTIONWAITER_H_
#define EXECUTIONWAITER_H_

#include <moveit/move_group_interface/move_group.h>


class ExecutionWaiter 
{
public:
    
    ExecutionWaiter(moveit::planning_interface::MoveGroup*);
    
    void Wait();
    
    void Sleep(double);
    
private:
    
    moveit::planning_interface::MoveGroup* move_group;
    
    std::vector<double> current_joints;
    std::vector<double> prevous_joints;
    
};

#endif