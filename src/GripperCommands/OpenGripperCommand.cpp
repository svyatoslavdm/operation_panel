#include <operation_panel/GripperCommands/OpenGripperCommand.h>

OpenGripperCommand::OpenGripperCommand(IGripperController* _GripperController)
{
    gripper_controller = _GripperController;
}
void OpenGripperCommand::Execute()
{
    gripper_controller->Open();
}
