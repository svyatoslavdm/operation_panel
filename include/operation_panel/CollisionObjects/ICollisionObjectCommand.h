#ifndef ICOLLISIONOBJECTCOMMAND_H_
#define ICOLLISIONOBJECTCOMMAND_H_

#include <operation_panel/CollisionObjects/ICollisionObjectController.h>

class ICollisionObjectCommand 
{
public:
    
//     ICollisionObjectCommand(ICollisionObjectController*);
    
//     virtual void SetCollisionObjectController(ICollisionObjectController*) = 0;
    
    virtual void Execute() = 0;
    
private:

};


#endif