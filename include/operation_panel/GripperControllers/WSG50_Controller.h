#ifndef WSG50_CONTROLLER_H
#define WSG50_CONTROLLER_H

#include <operation_panel/GripperControllers/IGripperController.h>

#include <ros/ros.h>

#include <wsg_50_common/Cmd.h>

class WSG50_Controller : public IGripperController
{
public:
    
    WSG50_Controller(ros::NodeHandle);
    
    void Open();
    
    void Close();
    
    void Start();
    
    void Stop();

private:
    
    ros::NodeHandle nh;
    ros::Timer frame_timer;
    ros::Publisher publisher;
    
    float wsg_50_position;
    
    void FrameCallback(const ros::TimerEvent&);
};

#endif // WSG50_CONTROLLER_H
