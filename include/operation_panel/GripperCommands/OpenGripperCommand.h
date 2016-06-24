#ifndef OPENGRIPPER_H
#define OPENGRIPPER_H

#include <operation_panel/GripperCommands/IGripperCommand.h>
#include <operation_panel/GripperControllers/IGripperController.h>


class OpenGripperCommand : public IGripperCommand
{
public:
    
    OpenGripperCommand(IGripperController*);
    
    void Execute();
        
private:   
    
    
};

#endif // OPENGRIPPER_H
