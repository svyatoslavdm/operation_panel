#ifndef ACTIONUNSCREWCLIENT_H
#define ACTIONUNSCREWCLIENT_H

//#include <operation_panel/UnscrewAction.h>
#include <unscrew_msgs/UnscrewAction.h>
#include <unscrew_msgs/PlaceAction.h>
#include <unscrew_msgs/RayStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <thread>

typedef actionlib::SimpleActionClient<unscrew_msgs::UnscrewAction> ActionClient;
typedef actionlib::SimpleActionClient<unscrew_msgs::PlaceAction> PlaceActionClient;

namespace rviz
{
class GuiUnscrew;

class ActionUnscrewClient
{
public:    
    typedef unscrew_msgs::UnscrewGoal Goal;    
    typedef unscrew_msgs::PlaceGoal PlaceGoal;    
    
    ActionUnscrewClient(GuiUnscrew*);
//     void SetGUI(GuiUnscrew*);
    void Send(unscrew_msgs::UnscrewGoal);
    void Abort();
    
    void Send(unscrew_msgs::PlaceGoal);    
        
private:
    
    void DoneCallback(const actionlib::SimpleClientGoalState&, const unscrew_msgs::UnscrewResultConstPtr&);
    void ActiveCallback();
    void FeedbackCallback(const unscrew_msgs::UnscrewFeedbackConstPtr&);

    ActionClient action_client;    
    GuiUnscrew* gui;
    
    unscrew_msgs::UnscrewGoal current_goal_;
    
    
    PlaceActionClient place_action_client;      
    unscrew_msgs::PlaceGoal current_place_goal_;
    
    void PlaceDoneCallback(const actionlib::SimpleClientGoalState&, const unscrew_msgs::PlaceResultConstPtr&);
    void PlaceActiveCallback();
    void PlaceFeedbackCallback(const unscrew_msgs::PlaceFeedbackConstPtr&);    
};

}
#endif