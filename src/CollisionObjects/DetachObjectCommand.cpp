#include <operation_panel/CollisionObjects/DetachObjectCommand.h>
void DetachObjectCommand::Execute()
{
    controller->DetachCollisionObject();
    ros::Duration(1.0).sleep();
}
DetachObjectCommand::DetachObjectCommand(ICollisionObjectController* _controller)// : ICollisionObjectCommand(_controller)
{
    controller = _controller;
}
