#include <operation_panel/GraspsFilters/IGraspsFilter.h>

IGraspsFilter::IGraspsFilter()
{

}

void IGraspsFilter::find_points()
{
    points.clear(); 
    geometry_msgs::Pose pose;   
    
    for (const auto& frame : initial_framenames)
    {
	if (server->get(group->getPlanningFrame(), frame, pose))
	{
	    points.push_back(std::make_pair(frame, pose));
	}
    }
}
