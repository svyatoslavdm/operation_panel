#ifndef IGRASPREADER_H
#define IGRASPREADER_H

#include <geometry_msgs/Pose.h>
#include <operation_panel/Servers/TFServer.h>

class IGraspReader
{

public:
     
    IGraspReader();

    virtual void set_grasp_candidates() = 0;
    
};

#endif // IGRASPREADER_H
