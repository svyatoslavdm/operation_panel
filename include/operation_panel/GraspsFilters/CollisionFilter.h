#ifndef COLLISIONFILTER_H
#define COLLISIONFILTER_H

#include <operation_panel/GraspsFilters/IGraspsFilter.h>
#include <moveit/collision_detection/collision_common.h>

class CollisionFilter : public IGraspsFilter
{
public:
     
    CollisionFilter( boost::shared_ptr<planning_scene::PlanningScene>, boost::shared_ptr<moveit::planning_interface::MoveGroup>, boost::shared_ptr<TFServer>, std::vector<std::pair<std::string, std::string>>);

    void filter();
    
private:
    
    std::string object_frame;
    boost::shared_ptr<planning_scene::PlanningScene> planning_scene_ptr;
    
    bool collision(geometry_msgs::Pose);
    
};

#endif // COLLISIONFILTER_H