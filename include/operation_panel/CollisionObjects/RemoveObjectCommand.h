#ifndef REMOVEOBJECTCOMMAND_H_
#define REMOVEOBJECTCOMMAND_H_

#include <operation_panel/CollisionObjects/ICollisionObjectCommand.h>

class RemoveObjectCommand : public ICollisionObjectCommand
{
public:
        
    RemoveObjectCommand(ICollisionObjectController*);
    
//     void SetCollisionObjectController(ICollisionObjectController*);
    
    void Execute();
    
private:
    
    ICollisionObjectController* controller;
};

#endif