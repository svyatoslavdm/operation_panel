#include <operation_panel/GripperCommands/CloseGripperCommand.h>

CloseGripperCommand::CloseGripperCommand(IGripperController* _GripperController)
{
    gripper_controller = _GripperController;
}
void CloseGripperCommand::Execute()
{
    gripper_controller->Close();
}