#ifndef DONOTHINGCOMMAND_H
#define DONOTHINGCOMMAND_H

#include <operation_panel/GripperCommands/IGripperCommand.h>
#include <operation_panel/GripperControllers/IGripperController.h>


class EmptyGripperCommand : public IGripperCommand
{
public:
    
    EmptyGripperCommand();
    
    void Execute();
        
private:
    
//     IGripperController *gripper_controller;
    
};

#endif // DONOTHINGCOMMAND_H
