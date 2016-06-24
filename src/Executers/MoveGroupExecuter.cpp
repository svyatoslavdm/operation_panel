#include <operation_panel/Executers/MoveGroupExecuter.h>

#include <operation_panel/CollisionObjects/AddObjectCommand.h>
#include <operation_panel/CollisionObjects/AttachObjectCommand.h>
#include <operation_panel/CollisionObjects/DetachObjectCommand.h>
#include <operation_panel/CollisionObjects/RemoveObjectCommand.h>
#include <operation_panel/CollisionObjects/EmptyObjectCommand.h>

#include <operation_panel/GripperCommands/EmptyGripperCommand.h>
#include <operation_panel/GripperCommands/OpenGripperCommand.h>

#include <typeinfo>


MoveGroupExecuter::MoveGroupExecuter(moveit::planning_interface::MoveGroup* _move_group, ICollisionObjectController* _object_controller, IGripperController* _egn_160_controller)
{
    move_group = _move_group;
    
    waiter = new ExecutionWaiter(move_group);
    
    object_controller = _object_controller;
    add = new AddObjectCommand(object_controller);
    remove = new RemoveObjectCommand(object_controller);
    attach = new AttachObjectCommand(object_controller);
    detach = new DetachObjectCommand(object_controller);
    empty = new EmptyObjectCommand(object_controller);
    
    egn_160_controller = _egn_160_controller;
}
void MoveGroupExecuter::SetMotionPlan(MoveGroupMotionPlan* _motion_plan)
{
    motion_plan = _motion_plan;
}
void MoveGroupExecuter::Execute()
{
    move_group->allowReplanning(false);    
    std::vector<moveit::planning_interface::MoveGroup::Plan>  plans = motion_plan->GetPlans();
    std::vector<IGripperCommand*> gripper_commands = motion_plan->GetGripperCommands();
    std::vector<ICollisionObjectCommand*> object_commands = motion_plan->GetObjectCommands();
    
    WSG50_Controller* wsg_controller_ptr = 0;
    wsg_controller_ptr =  dynamic_cast<WSG50_Controller*>(gripper_commands[1]->GetGripperController());
    wsg_controller_ptr->Start();
    
    std::vector<moveit::planning_interface::MoveGroup::Plan>::iterator plan_it = plans.begin();
    int count = 0;
    
    add->Execute();
    
    for (plan_it; plan_it != plans.end(); ++plan_it)
    {
	ros::Duration(0.2).sleep();
	
	move_group->asyncExecute(*plan_it);
	
	waiter->Wait();
	
	object_commands.at(count)->Execute();	
	
	if (typeid(*(gripper_commands.at(count))) == typeid(OpenGripperCommand))
	{
// 	    egn_160_controller->Close();
	}
	
	gripper_commands.at(count)->Execute();
	
	if (typeid(*(gripper_commands.at(count))) != typeid(EmptyGripperCommand))
	{
	    waiter->Sleep(7.0);
	}
	count++;
    }
    
    remove->Execute();
    
    wsg_controller_ptr->Stop();
}

