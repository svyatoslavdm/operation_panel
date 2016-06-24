#ifndef REACHABILITYFILTER_H
#define REACHABILITYFILTER_H

#include <operation_panel/GraspsFilters/IGraspsFilter.h>


class ReachAbilityFilter : public IGraspsFilter
{
public:
     
    ReachAbilityFilter(boost::shared_ptr<moveit::planning_interface::MoveGroup>, boost::shared_ptr<TFServer>, std::vector<std::pair<std::string, std::string>>);
    
    void filter();
    
private:
    
    std::string object_frame;
    
    bool plan(geometry_msgs::Pose pose);
    
};

#endif // REACHABILITYFILTER_H

    