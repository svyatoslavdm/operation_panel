#include <operation_panel/MotionPlanners/MoveGroupMotionPlanner.h>

#include <operation_panel/GripperCommands/OpenGripperCommand.h>
#include <operation_panel/GripperCommands/CloseGripperCommand.h>
#include <operation_panel/GripperCommands/EmptyGripperCommand.h>

#include <operation_panel/CollisionObjects/AddObjectCommand.h>
#include <operation_panel/CollisionObjects/AttachObjectCommand.h>
#include <operation_panel/CollisionObjects/DetachObjectCommand.h>
#include <operation_panel/CollisionObjects/RemoveObjectCommand.h>
#include <operation_panel/CollisionObjects/EmptyObjectCommand.h>

#include <typeinfo>

#include <operation_panel/robot_config.h>

MoveGroupMotionPlanner::MoveGroupMotionPlanner(moveit::planning_interface::MoveGroup* _move_group, TFServer* _tfServer, 
					       planning_scene_monitor::PlanningSceneMonitorPtr _planning_scene_monitor,
					       ICollisionObjectController* _object_controller)
{
    move_group = _move_group;
    tfServer = _tfServer;
    planning_scene_monitor = _planning_scene_monitor;
    
    object_controller = _object_controller;
    add = new AddObjectCommand(object_controller);
    remove = new RemoveObjectCommand(object_controller);
    attach = new AttachObjectCommand(object_controller);
    detach = new DetachObjectCommand(object_controller);
    empty = new EmptyObjectCommand(object_controller);
    
    target_state = new robot_state::RobotState(_planning_scene_monitor->getRobotModel());
    
    cartesian.push_back(false);
    cartesian.push_back(true);
    cartesian.push_back(true);
    cartesian.push_back(false);
    cartesian.push_back(true);
    cartesian.push_back(true);
    cartesian.push_back(false);

}
void MoveGroupMotionPlanner::SetPlan(MoveGroupMotionPlan* _motion_plan)
{
    motion_plan = _motion_plan;
}
bool MoveGroupMotionPlanner::Plan()
{
    int traj_num = 0;    
    bool succesful_pose;
    bool joint_speeds_ok;
    first_trajectory = true;
    object_attached = false;    
    stop_plan = false;
    
    moveit::planning_interface::MoveGroup::Plan plan;
    
    CreateLogFile();
    
    ReadMotionPlan();      
    
    move_group->setPlannerId("RRTConnectkConfigDefault");
    for (std::vector<std::vector<geometry_msgs::Pose > >::iterator traj_it = tool_trajectories.begin(); traj_it != tool_trajectories.end(); ++traj_it)
    {
	int pose_num = 0;
	plans.clear();
	SetStartStateToCurrent();	
	SetCurrentGrasp(*traj_it);
	traj_it->push_back(move_group->getCurrentPose().pose);	
	add->Execute();
	
	for (std::vector<geometry_msgs::Pose>::iterator pose_it = traj_it->begin(); pose_it != traj_it->end(); pose_it++)
	{   	    
	    ROS_WARN_STREAM("Start planning traj:" << traj_num << " pose:" << pose_num);
	    	    if (stop_plan)
	    {
		succesful_pose = false;
		break;
	    }
	    if (!SetTargetStateToPose(*pose_it))
	    {
		succesful_pose = false;
		break;	    
	    }
	    
	    move_group->setNumPlanningAttempts(5);
	    move_group->setPlanningTime(5);	 
	    
	    if (cartesian[pose_num])
	    {
		std::vector<geometry_msgs::Pose> waypoints;
		moveit_msgs::RobotTrajectory trajectory;
		waypoints.push_back(*pose_it);
		double frac = move_group->computeCartesianPath(waypoints, 0.01, 2.0, trajectory);	    
		succesful_pose = frac == 1.0;	
		plan.trajectory_ = trajectory;

	    }
	    else
	    {
		succesful_pose = move_group->plan(plan);   
	    }	 
	    	    
	    
	    joint_speeds_ok = CheckJointTrajectory(plan);
	    if (succesful_pose && joint_speeds_ok)    
	    {
		collision_object_commands.at(pose_num)->Execute();
		CheckObjectCommand(collision_object_commands.at(pose_num));
		
		ROS_INFO_STREAM("Path segment #" << pose_num << " planned successfully!");
		plans.push_back(plan);			
		SetStartStateToTarget();
		ros::Duration(0.2).sleep();
	    }
	    else 
	    {
		ROS_INFO_STREAM("Path segment #" << pose_num << " planning FAILED!");
		break;
	    }
	    pose_num++;
	}
	if (object_attached)
	{
	    detach->Execute();
	    object_attached = false;
	}
	remove->Execute();
	RemoveCurrentGrasp();
	if(succesful_pose && joint_speeds_ok)
	{
	    WriteMotionPlan();
	    ROS_INFO_STREAM("Trajectory planning SUCCESFULL!");
	    CloseLogFile();
	    return true;
	}  
	if (stop_plan)
	{
	    succesful_pose = false;
	    break;
	}
	traj_num++;
    }
    ROS_INFO_STREAM("Trajectory planning FAILED!");
    CloseLogFile();
    return false;
}
void MoveGroupMotionPlanner::StopPlan()
{
    stop_plan = true;
}

void MoveGroupMotionPlanner::CheckObjectCommand(ICollisionObjectCommand* command)
{
    if (typeid(*command) == typeid(AttachObjectCommand))
    {
	object_attached = true;
    }
    if (typeid(*command) == typeid(DetachObjectCommand))
    {
	object_attached = false;
    }
}
void MoveGroupMotionPlanner::ReadMotionPlan()
{
    gripper_commands.clear();
    collision_object_commands.clear();
    tool_trajectories.clear();
    
    gripper_commands = motion_plan->GetGripperCommands();
    collision_object_commands = motion_plan->GetObjectCommands();
    tool_trajectories = motion_plan->GetToolTrajectories();
    
    collision_object_commands.push_back(empty);
    gripper_commands.push_back(new EmptyGripperCommand);

    motion_plan->Clear();
}
void MoveGroupMotionPlanner::WriteMotionPlan()
{
    for (auto plan : plans)
    {
	motion_plan->AddPlan(plan);
    }
    for (auto command : gripper_commands)
    {
	motion_plan->AddGripperCommand(command);
    }
    for (auto command : collision_object_commands)
    {
	motion_plan->AddCollisionObjectCommand(command);
    }
    ROS_INFO_STREAM("Trajectory planning ended SUCCESFULLY!");
    
    ROS_INFO_STREAM("motion_plan->GetPoints().size(): " << motion_plan->GetPoints().size());
    ROS_INFO_STREAM("motion_plan->GetPlans().size(): " << motion_plan->GetPlans().size());
    ROS_INFO_STREAM("motion_plan->GetGripperCommands().size(): " << motion_plan->GetGripperCommands().size());
    ROS_INFO_STREAM("motion_plan->GetObjectCommands().size(): " << motion_plan->GetObjectCommands().size());
}

bool MoveGroupMotionPlanner::CheckPose(geometry_msgs::Pose _pose)
{
    planning_scene::PlanningScenePtr planning_scene_ptr = planning_scene_monitor->getPlanningScene();
    robot_state::RobotState& state = planning_scene_ptr->getCurrentStateNonConst(); 
    const robot_state::JointModelGroup *joint_model_group = state.getJointModelGroup(move_group->getName());  
    state.setFromIK(joint_model_group, _pose);
    state.update();
    return planning_scene_ptr->isStateValid(state, move_group->getName());
}
bool MoveGroupMotionPlanner::CheckJointTrajectory(moveit::planning_interface::MoveGroup::Plan plan)
{    
    planning_scene_monitor->requestPlanningSceneState(PLANNING_SCENE_SERVICE); 
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor); 
    ps->getCurrentStateNonConst().update(); 
    robot_state::RobotState state_to_check = ps->getCurrentState(); 
    
    std::vector<double> delta_max = {0.15, 0.15, 0.3, 0.2, 0.35, 0.35};
  
    if (plan.trajectory_.joint_trajectory.points.size() > 0)
    {
	curr_point = prev_point = plan.trajectory_.joint_trajectory.points[0];	

	if (first_trajectory)
	{
	    first_trajectory = false;
	}
	else
	{
	    int position_num = 0;
	    for (auto position : curr_point.positions)
	    {
		double delta = curr_point.positions[position_num] - prev_point.positions[position_num];
		if (fabs(delta) > 0.1)
		{
		    ROS_WARN_STREAM("Start position delta = " << fabs(delta) << " > 0.1");
		    ROS_WARN_STREAM("Dropping trajectory");		    
		    return false;
		}
		position_num++;
	    }
	}

	for (int i = 0; i < plan.start_state_.joint_state.name.size(); i++)
	{
	    if (plan.start_state_.joint_state.name[i] == JOINT1NAME)
		plan.start_state_.joint_state.position[i] = curr_point.positions[0];
	    if (plan.start_state_.joint_state.name[i] == JOINT2NAME)
		plan.start_state_.joint_state.position[i] = curr_point.positions[1];
	    if (plan.start_state_.joint_state.name[i] == JOINT3NAME)
		plan.start_state_.joint_state.position[i] = curr_point.positions[2];
	    if (plan.start_state_.joint_state.name[i] == JOINT4NAME)
		plan.start_state_.joint_state.position[i] = curr_point.positions[3];
	    if (plan.start_state_.joint_state.name[i] == JOINT5NAME)
		plan.start_state_.joint_state.position[i] = curr_point.positions[4];
	    if (plan.start_state_.joint_state.name[i] == JOINT6NAME)
		plan.start_state_.joint_state.position[i] = curr_point.positions[5];
	}	
	LogPlan(plan);
	
	int point_num = 0;
	for (auto point : plan.trajectory_.joint_trajectory.points)
	{
	    curr_point = point;
	    int position_num = 0;
	    
	    //Checking joints delta
	    
	    for (auto position : point.positions)  
	    {
		double delta = curr_point.positions[position_num] - prev_point.positions[position_num];
		if (fabs(delta) > delta_max[position_num])
		{
		    ROS_WARN_STREAM("Position #" << position_num << " delta = " << fabs(delta) << " > " << delta_max[position_num]);
		    ROS_WARN_STREAM("Dropping trajectory");		    
		    return false;
		}
		position_num++;
	    }
	    
	    //Checking point for collisions
// 	    for (int i = 0; i < 6; i++)
// 	    {
// 		const std::string joint_name = "kuka/A" + std::to_string(i + 1);
// 		std::vector<double > position;
// 		position.push_back(curr_point.positions[i]);;
// 		state_to_check.setJointPositions(joint_name, position);
// 	    } 
// 	    collision_detection::CollisionRequest req;
// 	    collision_detection::CollisionResult res;	    
// 	    ps->checkCollision(req, res, state_to_check);
// 	    if (res.collision)
// 	    {
// 		ROS_ERROR_STREAM ("Point #" << point_num << " collision is " << res.collision);
// 		return false;
// 	    }
	    
	    
	    prev_point = curr_point;
	    point_num++;
	}
    }
    last_point = curr_point;
    return true;
}
void MoveGroupMotionPlanner::SetStartStateToCurrent()
{
    planning_scene_monitor->requestPlanningSceneState(PLANNING_SCENE_SERVICE); 
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor); 
    ps->getCurrentStateNonConst().update(); 
    robot_state::RobotState current_state = ps->getCurrentState(); 
    move_group->setStartState(current_state);
}
bool MoveGroupMotionPlanner::SetStartStateToPose(geometry_msgs::Pose _pose)
{
    planning_scene_monitor->requestPlanningSceneState(PLANNING_SCENE_SERVICE); 
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor); 
    ps->getCurrentStateNonConst().update(); 
    robot_state::RobotState current_state = ps->getCurrentState(); 

    const robot_state::JointModelGroup *joint_model_group = current_state.getJointModelGroup(move_group->getName());
    
    if (current_state.setFromIK(joint_model_group, _pose))
    {
	move_group->setStartState(current_state);
	return true;
    }
    return false;    
}
void MoveGroupMotionPlanner::SetStartStateToTarget()
{
    planning_scene_monitor->requestPlanningSceneState(PLANNING_SCENE_SERVICE); 
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor); 
    ps->getCurrentStateNonConst().update(); 
    robot_state::RobotState current_state = ps->getCurrentState(); 

    std::vector<double> joint_states;
    target_state->copyJointGroupPositions(move_group->getName(), joint_states);    
    current_state.setJointGroupPositions(move_group->getName(), joint_states);
    move_group->setStartState(current_state);
    
//     return true;     
}
bool MoveGroupMotionPlanner::SetTargetStateToPose(geometry_msgs::Pose _pose)
{
    planning_scene_monitor->requestPlanningSceneState(PLANNING_SCENE_SERVICE); 
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor); 
    ps->getCurrentStateNonConst().update(); 
    *target_state = ps->getCurrentState(); 

    const robot_state::JointModelGroup *joint_model_group = target_state->getJointModelGroup(move_group->getName());

    if (target_state->setFromIK(joint_model_group, _pose))
    {
	move_group->setJointValueTarget(*target_state);
	return true;
    }
    return false;
}
void MoveGroupMotionPlanner::SetTargetStateToCurrent()
{
    planning_scene_monitor->requestPlanningSceneState(PLANNING_SCENE_SERVICE); 
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor); 
    ps->getCurrentStateNonConst().update(); 
    robot_state::RobotState current_state = ps->getCurrentState(); 
    move_group->setJointValueTarget(current_state);
}
void MoveGroupMotionPlanner::SetCurrentGrasp(std::vector< geometry_msgs::Pose > traj)
{
    std::string parent = "world";
    std::string child = "current_grasp_tool";
    tfServer->set(parent, child, traj.at(1));

}
void MoveGroupMotionPlanner::RemoveCurrentGrasp()
{
    std::string parent = "world";
    std::string child = "current_grasp_tool";
    tfServer->remove(parent, child);
}
void MoveGroupMotionPlanner::CreateLogFile()
{    
    std::string filename = ros::package::getPath("operation_panel") + "/log/plans_log.txt";
    file_stream = new std::ofstream(filename.c_str());
}
void MoveGroupMotionPlanner::CloseLogFile()
{
    file_stream->close();
}
void MoveGroupMotionPlanner::LogPlan(moveit::planning_interface::MoveGroup::Plan plan)
{
    std::vector<double> delta_max = {0.15, 0.15, 0.3, 0.2, 0.3, 0.3};
    
    trajectory_msgs::JointTrajectoryPoint prev_point;
    trajectory_msgs::JointTrajectoryPoint curr_point;
    
    if (file_stream->is_open())
    {
	if (plan.trajectory_.joint_trajectory.points.size() > 0)
	{
	    curr_point = prev_point = plan.trajectory_.joint_trajectory.points[0];
	    int point_num = 0;
	    *file_stream << "---- New trajectory ----\n";
	    
	    *file_stream << "Start state: \n";
	    int start_state_pose_num = 0;
	    for (auto position : plan.start_state_.joint_state.position)
	    {
		*file_stream << "	joint[" << start_state_pose_num << "]: " << position << "\n";
		start_state_pose_num++;
	    }	    
	    *file_stream << "Plan trajectory points: " << plan.trajectory_.joint_trajectory.points.size() << "\n";
	    for (auto point : plan.trajectory_.joint_trajectory.points)
	    {
		curr_point = point;
		*file_stream << "Point #" << point_num << "\n";
		int position_num = 0;
		for (auto position : point.positions)
		{
		    *file_stream << "	joint[" << position_num << "]: " << position << "\n";
		    double delta = curr_point.positions[position_num] - prev_point.positions[position_num];
		    
		    if (fabs(delta) > delta_max[position_num])
		    {
			*file_stream << "		joint[" << position_num << "]: delta > " << delta_max[position_num] << "\n";
		    }
		    position_num++;
		}
		prev_point = curr_point;
		point_num++;
	    }
	}
    }
}



