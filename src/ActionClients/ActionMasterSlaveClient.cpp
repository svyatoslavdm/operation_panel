#include <operation_panel/ActionClients/ActionMasterSlaveClient.h>
#include <operation_panel/GUI/MasterSlaveGUI.h>

namespace rviz
{

ActionMasterSlaveClient::ActionMasterSlaveClient() : action_client("master_slave", true)	
{
    action_client.waitForServer(ros::Duration(0.5));
}

void ActionMasterSlaveClient::SetGui(GuiMasterSlave* _gui)
{
    gui = _gui;
}

void ActionMasterSlaveClient::Abort()
{
    action_client.cancelGoal();
}

void ActionMasterSlaveClient::DoneCallback(const actionlib::SimpleClientGoalState& _state, const master_slave_msgs::MasterSlaveResultConstPtr& _result)
{
    gui->SetActionDone(_result->return_code.description);
}

void ActionMasterSlaveClient::ActiveCallback()
{    
}

void ActionMasterSlaveClient::FeedbackCallback(const master_slave_msgs::MasterSlaveFeedbackConstPtr& _feedback)
{
    gui->SetActionState(_feedback->state_description);
}

void ActionMasterSlaveClient::Send(master_slave_msgs::MasterSlaveGoal _goal)
{
    action_client.sendGoal(_goal, boost::bind(&ActionMasterSlaveClient::DoneCallback, this, _1, _2), 
				boost::bind(&ActionMasterSlaveClient::ActiveCallback, this), 
				boost::bind(&ActionMasterSlaveClient::FeedbackCallback, this, _1));
}

}