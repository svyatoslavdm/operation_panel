#include <operation_panel/PathPlanners/PickAndPlacePlanner.h>

#include <operation_panel/GripperCommands/OpenGripperCommand.h>
#include <operation_panel/GripperCommands/CloseGripperCommand.h>
#include <operation_panel/GripperCommands/EmptyGripperCommand.h>

#include <operation_panel/CollisionObjects/AddObjectCommand.h>
#include <operation_panel/CollisionObjects/AttachObjectCommand.h>
#include <operation_panel/CollisionObjects/DetachObjectCommand.h>
#include <operation_panel/CollisionObjects/RemoveObjectCommand.h>
#include <operation_panel/CollisionObjects/EmptyObjectCommand.h>
#include <boost/graph/graph_concepts.hpp>
#include <operation_panel/robot_config.h>
PickAndPlacePlanner::PickAndPlacePlanner(ros::NodeHandle _nh,TFServer* _tfServer, IGripperController* _gripper_controller, 
					 MoveGroupMotionPlan* _motion_plan, moveit::planning_interface::MoveGroup* _move_group, 
					 planning_scene_monitor::PlanningSceneMonitorPtr _planning_scene_monitor,
					 ICollisionObjectController* _object_controller
					)
{
    nh = _nh;
    
    motion_plan = _motion_plan;
    tfServer = _tfServer;
    tfServerPtr.reset(tfServer);
    gripper_controller = _gripper_controller;
    move_group = _move_group;
    planning_scene_monitor = _planning_scene_monitor;
    collision_object_controller = _object_controller;
    
    add = new AddObjectCommand(collision_object_controller);
    remove = new RemoveObjectCommand(collision_object_controller);
    attach = new AttachObjectCommand(collision_object_controller);
    detach = new DetachObjectCommand(collision_object_controller);
    empty = new EmptyObjectCommand(collision_object_controller);
    
    close = new CloseGripperCommand(gripper_controller);
    open = new OpenGripperCommand(gripper_controller);
    empty_gripper = new EmptyGripperCommand();    
    
}
void PickAndPlacePlanner::SetPath(std::string _path)
{
    file_path = _path;
}
void PickAndPlacePlanner::SetPlanningType(bool _manual_grasp_choosing)
{
    manual_grasp_choosing = !_manual_grasp_choosing;
}

void PickAndPlacePlanner::Plan()
{
    motion_plan->Clear();
    
    planning_scene_ptr = planning_scene_monitor->getPlanningScene();
    
    ComputeObjectPath();
    if (!manual_grasp_choosing)
    {
	ReadGrasps();
    }
    else
    {
	SetGraspCandidate();
    }
    ComputeToolTrajectories();
}
void PickAndPlacePlanner::ComputeObjectPath()
{
    gripper_commands.clear();
    collision_object_commands.clear();
    
    geometry_msgs::Pose start_point;
    geometry_msgs::Pose finish_point;
    
    std::string parent = "world";
    std::string child_start = "start_object_marker";
    std::string child_finish = "finish_object_marker";
    
    double height = 0.15;
    
    tfServer->get(parent, child_start, start_point);
    motion_plan->AddPoint(start_point);
    gripper_commands.push_back(close);
    collision_object_commands.push_back(attach);
    
    start_point.position.z += height;
    motion_plan->AddPoint(start_point);
    gripper_commands.push_back(empty_gripper);
    collision_object_commands.push_back(empty);
    
    tfServer->get(parent, child_finish, finish_point);
    finish_point.position.z += height;
    motion_plan->AddPoint(finish_point);
    gripper_commands.push_back(empty_gripper);
    collision_object_commands.push_back(empty);
    
    finish_point.position.z -= height;
    motion_plan->AddPoint(finish_point);
    gripper_commands.push_back(open);
    collision_object_commands.push_back(detach);
    
}
void PickAndPlacePlanner::ReadGrasps()
{
    std::string object_frame = "start_object_marker";
    std::string tool_frame = TOOLNAME;		// r/link_6
    std::string grasp_frame = "wsg_50/palm_link";		// wsg_50/palm_link

    boost::shared_ptr<FileReader> grasps_reader(new FileReader(tfServerPtr, file_path, object_frame, tool_frame, grasp_frame));
    grasps_reader->set_grasp_candidates();
    
    ROS_INFO_STREAM(file_path);
    
    GetGraspCandidates();
}
void PickAndPlacePlanner::GetGraspCandidates()
{    
    std::string parent = "start_object_marker";
    std::string grasp_candidate = "grasp_candidate_";    
    std::string child_tool;
    
    geometry_msgs::Pose pose;
        
    grasp_candidates.clear();
    for (int candidate = 0; candidate < 200; candidate++)
    {
	child_tool = grasp_candidate + std::to_string(candidate);
	if (tfServer->getFromMap(parent, child_tool, pose))
	{
	    grasp_candidates.push_back(pose);
// 	    tfServer->remove(parent, child_tool);
	}
    }
    ROS_INFO_STREAM("Number grasp_candidates: " << grasp_candidates.size());
}
void PickAndPlacePlanner::SetGraspCandidate()
{
    std::string grasp_candidate = "grasp_candidate_0";
    std::string parent = "start_object_marker";
    
    grasp_candidates.clear();
    geometry_msgs::Pose grasp_pose;
    tfServer->get(parent, grasp_candidate, grasp_pose);
    grasp_candidates.push_back(grasp_pose);
}

void PickAndPlacePlanner::ComputeToolTrajectories()
{
    
    std::vector<geometry_msgs::Pose> points = motion_plan->GetPoints();
    
    int point_count = 0; 		// Publish object trajectory
    for (const auto& point : points)
    {
	std::string world = "world";
	std::string obj_traj_point = "object_trajectory_point_" + to_string(point_count);
	tfServer->set(world, obj_traj_point, point);
	point_count++;
    }
    int grasp_count = 0;					// For each grasp candidate publish tool trajectory
    for (const auto& grasp_candidate : grasp_candidates)
    {
	point_count = 0;
	for (const auto& point : points)
	{
	    std::string obj_traj_point = "object_trajectory_point_" + to_string(point_count);
	    std::string tool_trajectory_point = "tool_trajectory_" + to_string(grasp_count) + "_point_" + to_string(point_count + 1);
	    tfServer->set(obj_traj_point, tool_trajectory_point, grasp_candidate);
	    point_count++;
	}
	ComputePregraspPoint(grasp_count, point_count);
	grasp_count++;
    }
    ros::Duration(0.2).sleep();
    FilterTrajectories(grasp_count, point_count + 2);
    
    CleanTFs();
    ROS_INFO_STREAM("CLeaned TFs");
    RemoveTrajectories(grasp_count, point_count + 2);
    ROS_INFO_STREAM("GraspCandidatesRemoved");
    
    AddCommands();
    ROS_INFO_STREAM("AddCommands done");

}
void PickAndPlacePlanner::FilterTrajectories(int _grasp_count, int _point_count)
{
    std::vector<std::vector<geometry_msgs::Pose> > filtered_tool_trajectories;
    for (int grasp_count = 0; grasp_count < _grasp_count; grasp_count++)
    {
	std::vector<geometry_msgs::Pose> trajectory;
	for (int point_count = 0; point_count < _point_count; point_count++)
	{
	    std::string world = "world";
	    std::string tool_trajectory_point = "tool_trajectory_" + to_string(grasp_count) + "_point_" + to_string(point_count);
	    geometry_msgs::Pose point;
	    if (tfServer->get(world, tool_trajectory_point, point))
	    {
		ROS_INFO_STREAM("Checking traj: " << grasp_count << " point:" << point_count);
		if (CheckPose(point))
		{
		    trajectory.push_back(point);
		}
		else
		{
		    break;
		}
	    }
	    else 
	    {
		break;
	    }
	}
	if (trajectory.size() == _point_count)
	{
	    filtered_tool_trajectories.push_back(trajectory);
	}
    }
    ROS_INFO_STREAM("Number of trajectories after filtration: " << filtered_tool_trajectories.size());
    for (const auto& traj : filtered_tool_trajectories)
    {
	motion_plan->AddToolTrajectory(traj);
    }
    ROS_INFO_STREAM("Added tool trajs");    
}
void PickAndPlacePlanner::AddCommands()
{
    gripper_commands.insert(gripper_commands.begin(), empty_gripper);
    gripper_commands.push_back(empty_gripper);
    
    collision_object_commands.insert(collision_object_commands.begin(), empty);
    collision_object_commands.push_back(empty);
    
    for (const auto& command : gripper_commands)
    {
	motion_plan->AddGripperCommand(command);
    }
    for (const auto& command : collision_object_commands)
    {
	motion_plan->AddCollisionObjectCommand(command);
    }
}
bool PickAndPlacePlanner::CheckPose(geometry_msgs::Pose _pose)
{
    robot_state::RobotState& state = planning_scene_ptr->getCurrentStateNonConst(); 
    const robot_state::JointModelGroup *joint_model_group = state.getJointModelGroup(move_group->getName());  
    if (!state.setFromIK(joint_model_group, _pose))
    {	
	ROS_WARN_STREAM("Point rejected cause NO IK");
	return false;
    }
    state.update();  
    bool result = planning_scene_ptr->isStateValid(state, move_group->getName());
    if (!result)
    {
	ROS_WARN_STREAM("Point rejected cause STATE IS INVALID");
    }
    return result;
}


void PickAndPlacePlanner::ComputePregraspPoint(int grasp_count, int point_count)
{
    std::string tool_trajectory_point_1 = "tool_trajectory_" + to_string(grasp_count) + "_point_" + to_string(1);
    std::string pregrasp = "tool_trajectory_" + to_string(grasp_count) + "_point_" + to_string(0);
    
    std::string tool_trajectory_point_4 = "tool_trajectory_" + to_string(grasp_count) + "_point_" + to_string(point_count);
    std::string postgrasp = "tool_trajectory_" + to_string(grasp_count) + "_point_" + to_string(point_count + 1);
    
    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = -0.075; 
    
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    
    tfServer->set(tool_trajectory_point_1, pregrasp, pose);
    tfServer->set(tool_trajectory_point_4, postgrasp, pose);
}
void PickAndPlacePlanner::CleanTFs()
{
    geometry_msgs::Pose pose;
    
    std::string parent;
    std::string child;
    ROS_INFO_STREAM("CleanTFs: started");
    ROS_INFO_STREAM("CleanTFs: removing filtered_tool_trajectories");
    for (int trajectory = 0; trajectory < 400; trajectory++)
    {
	for (int point = 0; point < 10; point++)
	{
	    parent = "world";
	    child = "filtered_tool_trajectory_" + to_string(trajectory) + "_point_" + to_string(point);
	    if (tfServer->getFromMap(parent, child, pose))
	    {
// 		ROS_INFO_STREAM("CleanTFs: trying to remove: " << parent << "->" << child);
		tfServer->remove(parent, child);
// 		ROS_INFO_STREAM("CleanTFs: removed: " << parent << "->" << child);
	    }
	}
    }
    ROS_INFO_STREAM("CleanTFs: removing object_trajectory_points");
    for (int point = 0; point < 10; point++)
    {
	parent = "world";
	child = "object_trajectory_point_" + to_string(point);
	if (tfServer->getFromMap(parent, child, pose))
	{
	    tfServer->remove(parent, child);
	}
    }
    parent = "start_object_marker";
    std::string grasp_candidate = "grasp_candidate_";    
    std::string child_tool;
        
    grasp_candidates.clear();
    
    ROS_INFO_STREAM("CleanTFs: removing grasp_candidates");
    
    for (int candidate = 0; candidate < 400; candidate++)
    {
	child_tool = grasp_candidate + std::to_string(candidate);
	if (tfServer->getFromMap(parent, child_tool, pose))
	{
// 	    ROS_INFO_STREAM("CleanTFs: removing grasp_candidate_" << candidate);
	    tfServer->remove(parent, child_tool);
// 	    ROS_INFO_STREAM("CleanTFs: removed grasp_candidate_" << candidate);
	}
	else
	{
	    
	}
    }
}
void PickAndPlacePlanner::RemoveTrajectories(int _grasp_count, int _point_count)
{
    geometry_msgs::Pose pose;
    ROS_INFO_STREAM("RemoveTrajectories start");
    for (int trajectory = 0; trajectory < _grasp_count; trajectory++)
    {
	int point = 0;
	std::string parent = "tool_trajectory_" + to_string(trajectory) + "_point_" + to_string(point + 1);
	std::string child = "tool_trajectory_" + to_string(trajectory) + "_point_" + to_string(point);
	if (tfServer->getFromMap(parent, child, pose))
	{
	    tfServer->remove(parent, child);
	}	
	for (point = 1; point < 5; point++)
	{
	    parent = "object_trajectory_point_" + to_string(point - 1);
	    child = "tool_trajectory_" + to_string(trajectory) + "_point_" + to_string(point);
	    if (tfServer->getFromMap(parent, child, pose))
	    {
		tfServer->remove(parent, child);
	    }
	}	
	parent = "tool_trajectory_" + to_string(trajectory) + "_point_" + to_string(point -1);
	child = "tool_trajectory_" + to_string(trajectory) + "_point_" + to_string(point);
	if (tfServer->getFromMap(parent, child, pose))
	{
	    tfServer->remove(parent, child);
	}
    }
    ROS_INFO_STREAM("RemoveTrajectories finish");
}



