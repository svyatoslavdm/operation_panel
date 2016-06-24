#ifndef ATTACHOBJECTCOMMAND_H_
#define ATTACHOBJECTCOMMAND_H_

#include <operation_panel/CollisionObjects/ICollisionObjectCommand.h>

class AttachObjectCommand : public ICollisionObjectCommand
{
public:
    
    AttachObjectCommand(ICollisionObjectController*);
    
//     void SetCollisionObjectController(ICollisionObjectController*);
    
    void Execute();
    
private:
    
    ICollisionObjectController* controller;
};

#endif