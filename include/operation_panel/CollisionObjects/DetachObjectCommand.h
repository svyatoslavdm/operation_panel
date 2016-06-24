#ifndef DETACHOBJECTCOMMAND_H_
#define DETACHOBJECTCOMMAND_H_

#include <operation_panel/CollisionObjects/ICollisionObjectCommand.h>

class DetachObjectCommand : public ICollisionObjectCommand
{
public:
            
    DetachObjectCommand(ICollisionObjectController*);
    
//     void SetCollisionObjectController(ICollisionObjectController*);
    
    void Execute();
    
private:
    
    ICollisionObjectController* controller;
};

#endif