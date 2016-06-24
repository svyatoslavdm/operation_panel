#include <operation_panel/CollisionObjects/AttachObjectCommand.h>
void AttachObjectCommand::Execute()
{
    controller->AttachCollisionObject();
    ros::Duration(1.0).sleep();
}
AttachObjectCommand::AttachObjectCommand(ICollisionObjectController* _controller)// : ICollisionObjectCommand(_controller)
{
    controller = _controller;
}
