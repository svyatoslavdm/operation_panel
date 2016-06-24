#ifndef IGRIPPERCONTROLLER_H
#define IGRIPPERCONTROLLER_H

#include <ros/ros.h>

class IGripperController
{

public:
    
    IGripperController(ros::NodeHandle);
    
    virtual void Open() = 0;

    virtual void Close() = 0;

};

#endif // IGRIPPERCONTROLLER_H
