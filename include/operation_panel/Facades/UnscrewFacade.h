#ifndef UNSCREWFACADE_H
#define UNSCREWFACADE_H

#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <QVBoxLayout>

#include <moveit/background_processing/background_processing.h>
#include <interactive_markers/interactive_marker_server.h>

#include <operation_panel/Scene/Scene.h>

#include <operation_panel/MsgsParsers/IMsgsParser.h>
#include <operation_panel/MsgsParsers/ObjectMsgsParser.h>
#include <operation_panel/MsgsParsers/ORKMsgsParser.h>

#include <operation_panel/ServersManagers/MarkerServerManager.h>
#include <operation_panel/ServersManagers/TFServerManager.h>
#include <operation_panel/ServersManagers/InteractiveMarkerServerManager.h>

#include <operation_panel/ActionClients/ActionUnscrewClient.h>
#include <operation_panel/GripperControllers/WSG50_Controller.h>
#include <operation_panel/GripperControllers/EGN160_Controller.h>

#include <operation_panel/Plans/MoveGroupMotionPlan.h>
#include <operation_panel/Executers/ExecutionWaiter.h>
#include <operation_panel/Executers/MoveGroupExecuter.h>
#include <operation_panel/CollisionObjects/CollisionObjectUtilities.h>

namespace rviz
{
class GuiUnscrew;

class UnscrewFacade 
{
public:    
    UnscrewFacade(ros::NodeHandle);

    void SetGUI(GuiUnscrew*);
    void SetClient(ActionUnscrewClient*);
    
    void StartViewPointThread();

    void SetGraspMarker(bool);
    void SetUnscrewGrasp();
    void SetUnscrewAxis();  

    void SetDirection(bool);
    void SetAxisTolerance(float);
    void SetMaxGraspForce(float);
    void SetMaxPullForce(float);    
    void SetMaxCycles(int);
    void SetRotatePerCycle(int); 
    
    void SetManualTune(bool);    
    
    void Plan();    
    void Execute();
    void Abort();  
    
    void Place();     
    
private:
    ros::NodeHandle n;
    GuiUnscrew* gui;
    
    // Servers, managers & other stuff
    std::string object_pose_name;
    std::string object_axis_name;
    std::string file_path;
    std::string object_name;
    visualization_msgs::Marker object_axis_marker;
    
    TFServer* tfServer;
    boost::shared_ptr<TFServer> tfServerPtr;
    TFServerManager* tfServerManager;
    
    MarkerServer* markerServer;
    boost::shared_ptr<MarkerServer> markerServerPtr;    
    MarkerServerManager* markerServerManager;
        
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interaktiveMarkerServerPtr;
    InteractiveMarkerServerManager* interaktiveMarkerServerManager;  
    
    IMsgsParser* msgsParser;
    Scene* scene;
    
//     bool addedObject;
    bool setObjectIntMarker;
    bool setObjectMarker;
    bool setAxisMarker;
    bool setGraspMarker;
           
    // Action client
    ActionUnscrewClient* actionClient;
    
    std::mutex actionMutex;
    std::thread actionThread;
    std::thread placeActionThread;    
    volatile bool isTerminated;
    bool hasGoal;    
    bool ObjectAdhesion;

    // Action goal parameters
    boost::shared_ptr<ActionUnscrewClient::Goal> goalPtr;     
    boost::shared_ptr<ActionUnscrewClient::PlaceGoal> placeGoalPtr;     
    
    int maxCycles;
    bool clockwise;
    bool manualTune;    
    float axisTolerance;
    float graspPosture;
    float maxGraspForce;
    float maxPullOutForce;
    float desiredDistance;
    float rotatePerCycle;
    geometry_msgs::PoseStamped graspPose;
    unscrew_msgs::RayStamped unscrewAxis;
        
    // Moveit motion stuff   
    boost::shared_ptr<moveit::planning_interface::MoveGroup> groupPtr;
    const robot_state::JointModelGroup* jointModelGroup;
    moveit::planning_interface::MoveGroup::Plan plan;
    std::vector<double> viewPointJointValues; 
    ExecutionWaiter* waiter;
    
    moveit::tools::BackgroundProcessing background_process;
    
    void AddBackgroundJob(const boost::function<void()> &, const std::string &);        
    void ViewPointMotion();
    void SetViewpointJoints();    
    void SendActionGoal();    
    void AbortActionGoal();
    void SendPlaceGoal();    
};

}

#endif