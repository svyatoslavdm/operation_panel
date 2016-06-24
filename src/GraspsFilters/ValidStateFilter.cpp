#include <operation_panel/GraspsFilters/ValidStateFilter.h>

ValidStateFilter::ValidStateFilter(boost::shared_ptr<planning_scene::PlanningScene> planning_scene_ptr_, boost::shared_ptr<moveit::planning_interface::MoveGroup> group_, boost::shared_ptr<TFServer> server_, std::vector<std::pair<std::string, std::string>> tfs_)
{
    server = server_;
    group = group_;
    planning_scene_ptr = planning_scene_ptr_;
    for (const auto& name : tfs_)
    {
	initial_framenames.push_back(name.second);
    }
    
    object_frame = tfs_.at(0).first;    
}

bool ValidStateFilter::is_valid(geometry_msgs::Pose pose)
{
    robot_state::RobotState& state = planning_scene_ptr->getCurrentStateNonConst(); 
    const robot_state::JointModelGroup *joint_model_group = state.getJointModelGroup(group->getName());  
    state.setFromIK(joint_model_group, pose);
    state.update();
    return planning_scene_ptr->isStateValid(state, group->getName());
}

void ValidStateFilter::filter()
{
    find_points();
    
    for (const auto& point : points)
    {
	if (!is_valid(point.second))
	{
	    server->remove(object_frame, point.first);
	}
    }   
    ROS_INFO("Ended");
}