#include <operation_panel/CollisionObjects/RemoveObjectCommand.h>
void RemoveObjectCommand::Execute()
{
    controller->RemoveCollisionObjects();
    ros::Duration(1.0).sleep();
}
RemoveObjectCommand::RemoveObjectCommand(ICollisionObjectController* _controller)// : ICollisionObjectCommand(_controller)
{
    controller = _controller;
}
