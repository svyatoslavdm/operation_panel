#include <operation_panel/GripperControllers/WSG50_Controller.h>

WSG50_Controller::WSG50_Controller(ros::NodeHandle _nh) : IGripperController(_nh)
{
    nh = _nh;
    
    Open();
    
    publisher = nh.advertise<wsg_50_common::Cmd>("/wsg_50_driver/goal_position", 1);
    frame_timer = nh.createTimer(ros::Duration(0.25), &WSG50_Controller::FrameCallback, this);
    Stop();
}
void WSG50_Controller::Open()
{
    wsg_50_position = 109.0;
}
void WSG50_Controller::Close()
{
    wsg_50_position = 1.0;
}
void WSG50_Controller::FrameCallback(const ros::TimerEvent&)
{
    wsg_50_common::Cmd msg;
    msg.pos = wsg_50_position;
    msg.speed = 30.0;
    publisher.publish(msg);
}
void WSG50_Controller::Start()
{
    frame_timer.start();
}
void WSG50_Controller::Stop()
{
    frame_timer.stop();
}
