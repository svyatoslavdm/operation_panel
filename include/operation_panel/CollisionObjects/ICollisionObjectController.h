#ifndef ICOLLISIONOBJECTCONTROLLER_H_
#define ICOLLISIONOBJECTCONTROLLER_H_

#include <operation_panel/Servers/TFServer.h>
#include <operation_panel/Servers/MarkerServer.h>
#include <moveit/move_group_interface/move_group.h>

class ICollisionObjectController 
{    
public:
    
//     ICollisionObjectController(moveit::planning_interface::MoveGroup*, TFServer*, MarkerServer*);W
    
    virtual void AddStartStaticCollisionObject() = 0;
    
    virtual void RemoveCollisionObjects() = 0;
    
    virtual void AttachCollisionObject() = 0;
    
    virtual void DetachCollisionObject() = 0;
    
//     virtual void SetObject() = 0;
    
private:
    
};

#endif