#ifndef ACTIONMASTERSLAVECLIENT_H
#define ACTIONMASTERSLAVECLIENT_H

//#include <operation_panel/MasterSlaveAction.h>
#include <master_slave_msgs/MasterSlaveAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <thread>

typedef actionlib::SimpleActionClient<master_slave_msgs::MasterSlaveAction> MasterSlaveClient;

namespace rviz
{
class GuiMasterSlave;

class ActionMasterSlaveClient
{
public:
    typedef master_slave_msgs::MasterSlaveGoal Goal;
    
    ActionMasterSlaveClient();
    void SetGui(GuiMasterSlave*);
    void Send(Goal);
    void Abort();
        
private:
    
    void DoneCallback(const actionlib::SimpleClientGoalState&, const master_slave_msgs::MasterSlaveResultConstPtr&);
    void ActiveCallback();
    void FeedbackCallback(const master_slave_msgs::MasterSlaveFeedbackConstPtr&);

    MasterSlaveClient action_client;    
    GuiMasterSlave* gui;
};

}
#endif