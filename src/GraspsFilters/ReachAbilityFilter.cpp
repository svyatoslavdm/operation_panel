#include <operation_panel/GraspsFilters/ReachAbilityFilter.h>

ReachAbilityFilter::ReachAbilityFilter(boost::shared_ptr<moveit::planning_interface::MoveGroup> group_, boost::shared_ptr<TFServer> server_, std::vector<std::pair<std::string, std::string>> tfs_)
{
    server = server_;
    group = group_;
    for (const auto& name : tfs_)
    {
	initial_framenames.push_back(name.second);
    }
    
    object_frame = tfs_.at(0).first;    
}

bool ReachAbilityFilter::plan(geometry_msgs::Pose pose)
{
    moveit::planning_interface::MoveGroup::Plan plan;
    group->setPoseTarget(pose);
    return group->plan(plan);
}

void ReachAbilityFilter::filter()
{
    find_points();
    robot_state::RobotState home_state(*group->getCurrentState());
    
    for (const auto& point : points)
    {
	group->setStartState(home_state);
	if (!plan(point.second))
	{
	    server->remove(object_frame, point.first);
	}
    }    
}
