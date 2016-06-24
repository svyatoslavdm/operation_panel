#include <operation_panel/ActionClients/ActionUnscrewClient.h>
#include <operation_panel/GUI/UnscrewGUI.h>

namespace rviz
{

ActionUnscrewClient::ActionUnscrewClient(GuiUnscrew* _gui) : action_client("unscrew", true),  place_action_client("unscrew_place", true)
{
    gui = _gui;
}

void ActionUnscrewClient::Abort()
{
    action_client.cancelGoal();
}

void ActionUnscrewClient::DoneCallback(const actionlib::SimpleClientGoalState& _state, const unscrew_msgs::UnscrewResultConstPtr& _result)
{
    gui->SetReturnCode(_result->return_code.description);
    gui->SetActionDone(_result->return_code.val, current_goal_.plan_only);
}

void ActionUnscrewClient::ActiveCallback()
{    
}

void ActionUnscrewClient::FeedbackCallback(const unscrew_msgs::UnscrewFeedbackConstPtr& _feedback)
{
    gui->SetActionState(_feedback->state_description);
}

void ActionUnscrewClient::Send(unscrew_msgs::UnscrewGoal _goal)
{
    if(action_client.waitForServer(ros::Duration(0.5)))
    {   
	current_goal_ = _goal;
	
	action_client.sendGoal(_goal, boost::bind(&ActionUnscrewClient::DoneCallback, this, _1, _2), 
				      boost::bind(&ActionUnscrewClient::ActiveCallback, this), 
				      boost::bind(&ActionUnscrewClient::FeedbackCallback, this, _1));
    }
    else
    {
	gui->SetReturnCode("Action server unavailable");
	gui->SetActionDone(-1, false);	
    }
}

void ActionUnscrewClient::Send(unscrew_msgs::PlaceGoal _goal)
{
    if(place_action_client.waitForServer(ros::Duration(0.5)))
    {   
	current_place_goal_ = _goal;
	
	place_action_client.sendGoal(_goal, boost::bind(&ActionUnscrewClient::PlaceDoneCallback, this, _1, _2), 
					    boost::bind(&ActionUnscrewClient::PlaceActiveCallback, this), 
					    boost::bind(&ActionUnscrewClient::PlaceFeedbackCallback, this, _1));
    }
    else
    {
	gui->SetReturnCode("Action server unavailable");
	gui->SetActionDone(-1, false);	
    }
}

void ActionUnscrewClient::PlaceDoneCallback(const actionlib::SimpleClientGoalState& _state, const unscrew_msgs::PlaceResultConstPtr& _result)
{
    gui->SetReturnCode(_result->return_code.description);
    gui->SetActionDone(_result->return_code.val, current_goal_.plan_only);
}

void ActionUnscrewClient::PlaceActiveCallback()
{    
}

void ActionUnscrewClient::PlaceFeedbackCallback(const unscrew_msgs::PlaceFeedbackConstPtr& _feedback)
{
    gui->SetActionState(_feedback->state_description);
}

}