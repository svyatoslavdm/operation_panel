#ifndef CLOSEGRIPPERCOMMAND_H
#define CLOSEGRIPPERCOMMAND_H

#include <operation_panel/GripperCommands/IGripperCommand.h>
#include <operation_panel/GripperControllers/IGripperController.h>


class CloseGripperCommand : public IGripperCommand
{
public:
    
    CloseGripperCommand(IGripperController*);
    
    void Execute();
        
private:
    
};

#endif // CLOSEGRIPPERCOMMAND_H
