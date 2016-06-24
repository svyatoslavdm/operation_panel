#ifndef EMPTYOBJECTCOMMAND_H_
#define EMPTYOBJECTCOMMAND_H_

#include <operation_panel/CollisionObjects/ICollisionObjectCommand.h>

class EmptyObjectCommand : public ICollisionObjectCommand
{
public:
            
    EmptyObjectCommand(ICollisionObjectController*);
    
//     void SetCollisionObjectController(ICollisionObjectController*);
    
    void Execute();
    
private:
    
    ICollisionObjectController* controller;
};

#endif