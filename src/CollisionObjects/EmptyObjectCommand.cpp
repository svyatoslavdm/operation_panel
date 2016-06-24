#include <operation_panel/CollisionObjects/EmptyObjectCommand.h>
void EmptyObjectCommand::Execute()
{
//     controller->AddStartStaticCollisionObject();
}
EmptyObjectCommand::EmptyObjectCommand(ICollisionObjectController* _controller)// : ICollisionObjectCommand(_controller)
{
    controller = _controller;
}
