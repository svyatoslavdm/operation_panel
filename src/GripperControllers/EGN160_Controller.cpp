#include <operation_panel/GripperControllers/EGN160_Controller.h>

EGN160_Controller::EGN160_Controller(ros::NodeHandle _nh) : IGripperController(_nh)
{
    nh = _nh;
    
    open_client = _nh.serviceClient<schunk_egn160_driver::open>("/schunk_egn160_gripper/open");
    close_client = _nh.serviceClient<schunk_egn160_driver::close>("/schunk_egn160_gripper/close");
}
void EGN160_Controller::Open()
{
    schunk_egn160_driver::open msg;
    open_client.call(msg);
}
void EGN160_Controller::Close()
{
    schunk_egn160_driver::close msg;
    msg.request.grasping_force = 40;
    close_client.call(msg);
}
