#include <operation_panel/Plans/MoveGroupMotionPlan.h>

void MoveGroupMotionPlan::AddPlan(moveit::planning_interface::MoveGroup::Plan _plan)
{
    plans.push_back(_plan);
}
void MoveGroupMotionPlan::AddGripperCommand(IGripperCommand* _command)
{
    gripper_commands.push_back(_command);
}    
void MoveGroupMotionPlan::AddPoint(geometry_msgs::Pose _pose)
{
    points.push_back(_pose);
}  
void MoveGroupMotionPlan::AddCollisionObjectCommand(ICollisionObjectCommand* _command)
{
    object_commands.push_back(_command);
}
void MoveGroupMotionPlan::AddToolTrajectory(std::vector< geometry_msgs::Pose > tool_trajectory)
{
    tool_trajectories.push_back(tool_trajectory);
}
void MoveGroupMotionPlan::AddCartesain(bool cart)
{
    cartesian.push_back(cart);
}

void MoveGroupMotionPlan::Clear()
{
    plans.clear();
    gripper_commands.clear();
    object_commands.clear();
    points.clear();
    for (auto traj : tool_trajectories)
    {
	traj.clear();
    }
    tool_trajectories.clear();
    cartesian.clear();
}

std::vector<moveit::planning_interface::MoveGroup::Plan> MoveGroupMotionPlan::GetPlans()
{
    return plans;
}
std::vector<IGripperCommand*> MoveGroupMotionPlan::GetGripperCommands()
{
    return gripper_commands;
}
std::vector<ICollisionObjectCommand*> MoveGroupMotionPlan::GetObjectCommands()
{
    return object_commands;
}
std::vector<geometry_msgs::Pose> MoveGroupMotionPlan::GetPoints()
{
    return points;
}
std::vector< std::vector< geometry_msgs::Pose > > MoveGroupMotionPlan::GetToolTrajectories()
{
    return tool_trajectories;
}
std::vector< bool > MoveGroupMotionPlan::GetCartesian()
{
    return cartesian;
}

