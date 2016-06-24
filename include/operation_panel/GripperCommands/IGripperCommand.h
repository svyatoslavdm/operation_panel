#ifndef IGRIPPERCOMMAND_H
#define IGRIPPERCOMMAND_H

#include <operation_panel/GripperControllers/IGripperController.h>


class IGripperCommand
{

public:
    
    virtual void Execute() = 0;
    
    IGripperController* GetGripperController();
    
public:    
    
    IGripperController *gripper_controller;

};

#endif // IGRIPPERCOMMAND_H
