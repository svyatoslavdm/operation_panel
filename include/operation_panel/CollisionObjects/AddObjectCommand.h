#ifndef ADDOBJECTCOMMAND_H_
#define ADDOBJECTCOMMAND_H_

#include <operation_panel/CollisionObjects/ICollisionObjectCommand.h>

class AddObjectCommand : public ICollisionObjectCommand
{
public:    
    
    AddObjectCommand(ICollisionObjectController*);
    
//     void SetCollisionObjectController(ICollisionObjectController*);
    
    void Execute();
    
private:
    
    ICollisionObjectController* controller;
};

#endif