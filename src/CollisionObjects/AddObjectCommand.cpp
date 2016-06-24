#include <operation_panel/CollisionObjects/AddObjectCommand.h>
void AddObjectCommand::Execute()
{
    controller->AddStartStaticCollisionObject();
    ros::Duration(1.0).sleep();
}
AddObjectCommand::AddObjectCommand(ICollisionObjectController* _controller) //: ICollisionObjectCommand(_controller)
{
    controller = _controller;
}
