#ifndef IGRASPSFILTER_H
#define IGRASPSFILTER_H

#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>

#include <operation_panel/Servers/TFServer.h>

class IGraspsFilter
{

public:
     
    IGraspsFilter();

    virtual void filter() = 0;
    
protected:
    
    int grasps_num;
    boost::shared_ptr<TFServer> server;
    boost::shared_ptr<moveit::planning_interface::MoveGroup> group;
    
    std::vector<std::pair<std::string, geometry_msgs::Pose>> points;
    
    std::vector<std::string> initial_framenames;
    
    void find_points();
};

#endif // IGRASPSFILTER_H