#ifndef VALIDSTATEFILTER_H
#define VALIDSTATEFILTER_H

#include <operation_panel/GraspsFilters/IGraspsFilter.h>
// #include <moveit/collision_detection/collision_common.h>

class ValidStateFilter : public IGraspsFilter
{
public:
     
    ValidStateFilter( boost::shared_ptr<planning_scene::PlanningScene>, boost::shared_ptr<moveit::planning_interface::MoveGroup>, boost::shared_ptr<TFServer>, std::vector<std::pair<std::string, std::string>>);

    void filter();
    
private:
    
    std::string object_frame;
    boost::shared_ptr<planning_scene::PlanningScene> planning_scene_ptr;
    
    bool is_valid(geometry_msgs::Pose);
    
};

#endif // VALIDSTATEFILTER_H