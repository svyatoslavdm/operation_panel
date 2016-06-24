#include <operation_panel/GraspsFilters/CollisionFilter.h>

CollisionFilter::CollisionFilter(boost::shared_ptr<planning_scene::PlanningScene> planning_scene_ptr_, boost::shared_ptr<moveit::planning_interface::MoveGroup> group_, boost::shared_ptr<TFServer> server_, std::vector<std::pair<std::string, std::string>> tfs_)
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

bool CollisionFilter::collision(geometry_msgs::Pose pose)
{
    collision_detection::CollisionRequest request;
    request.distance = false;
    request.contacts = false;
    collision_detection::CollisionResult result;

    robot_state::RobotState& state = planning_scene_ptr->getCurrentStateNonConst(); 
    const robot_state::JointModelGroup *joint_model_group = state.getJointModelGroup(group->getName());  
    state.setFromIK(joint_model_group, pose);
    
//     planning_scene_ptr->checkSelfCollision(request, result, state);
    
    bool collision;
    collision = planning_scene_ptr->isStateColliding(state);
    
//     return result.collision;
    return collision;
}

void CollisionFilter::filter()
{
    find_points();
    for (const auto& point : points)
    {
	if (collision(point.second))
	{
	    ROS_INFO_STREAM("Removing " << object_frame << "->" << point.first << " frame");
	    server->remove(object_frame, point.first);
	}
    }   
}