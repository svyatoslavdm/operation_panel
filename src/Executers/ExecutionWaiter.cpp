#include <operation_panel/Executers/ExecutionWaiter.h>

ExecutionWaiter::ExecutionWaiter(moveit::planning_interface::MoveGroup* _move_group)
{
    move_group = _move_group;
}
void ExecutionWaiter::Wait()
{
    bool moving = true;
    
    current_joints = move_group->getCurrentJointValues();
    prevous_joints = current_joints;
    
    ros::Duration(1.0).sleep();
    ROS_INFO_STREAM("START WAITING");
    while (moving)
    {
	ros::Duration(1.0).sleep();
	current_joints = move_group->getCurrentJointValues();
	
	int current_joint = 0;
	
	for (std::vector<double>::iterator diff_it = current_joints.begin(); diff_it != current_joints.end(); ++diff_it)
	{
	    double diff = current_joints.at(current_joint) - prevous_joints.at(current_joint);
	    if (fabs(diff) > 0.0005)
	    {
		moving = true;
		break;
	    }
	    else 
	    {
		moving = false;
	    }
	    current_joint++;
	}
	prevous_joints = current_joints;
    }
    ROS_INFO_STREAM("STOP WAITING");
}
void ExecutionWaiter::Sleep(double seconds)
{
    ROS_INFO_STREAM("START SLEEP FOR " << seconds << " SECONDS");
    ros::Duration(seconds).sleep();
    ROS_INFO_STREAM("SLEEP END");
}

