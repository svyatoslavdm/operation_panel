#ifndef EGN160_CONTROLLER_H
#define EGN160_CONTROLLER_H

#include <operation_panel/GripperControllers/IGripperController.h>

#include <ros/ros.h>

#include <schunk_egn160_driver/close.h>
#include <schunk_egn160_driver/open.h>

class EGN160_Controller : public IGripperController
{
public:
    EGN160_Controller(ros::NodeHandle);
    
    void Open();

    void Close();
    
private:
    
    ros::NodeHandle nh;
    
    ros::ServiceClient open_client;
    ros::ServiceClient close_client;
};

#endif // EGN160_CONTROLLER_H
