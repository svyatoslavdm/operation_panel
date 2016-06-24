#ifndef PICKANDPLACEFACADE_H_
#define PICKANDPLACEFACADE_H_

#include <ros/ros.h>
#include <QHBoxLayout>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/background_processing/background_processing.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <operation_panel/MsgsParsers/IMsgsParser.h>
#include <operation_panel/MsgsParsers/ObjectMsgsParser.h>
#include <operation_panel/MsgsParsers/ORKMsgsParser.h>
#include <operation_panel/MsgsParsers/PoseMsgsParser.h>

#include <operation_panel/Servers/TFServer.h>
#include <operation_panel/Servers/MarkerServer.h>
#include <operation_panel/Servers/CoordsServer.h>

#include <operation_panel/GripperControllers/WSG50_Controller.h>
#include <operation_panel/GripperControllers/EGN160_Controller.h>

#include <operation_panel/Plans/MoveGroupMotionPlan.h>

#include <operation_panel/PathPlanners/PickAndPlacePlanner.h>

#include <operation_panel/MotionPlanners/MoveGroupMotionPlanner.h>

#include <operation_panel/CollisionObjects/ICollisionObjectController.h>
#include <operation_panel/CollisionObjects/CollisionObjectUtilities.h>

#include <interactive_markers/interactive_marker_server.h>
#include <operation_panel/Scene/Scene.h>

#include <operation_panel/ServersManagers/MarkerServerManager.h>
#include <operation_panel/ServersManagers/TFServerManager.h>
#include <operation_panel/ServersManagers/InteractiveMarkerServerManager.h>

#include <operation_panel/Executers/MoveGroupExecuter.h>

#include <ork_gui/GetObject.h>

namespace rviz
{
class GuiPickAndPlace;

class PickAndPlaceFacade 
{
public:
    
    PickAndPlaceFacade(ros::NodeHandle);

    void SetGUI(GuiPickAndPlace*);
    
    void SetObjectDetection(bool);
    void SetObject();
    void SetObjectAdhesion(bool);
    
    void SetGraspChoser(bool);
    
    void SetStartPose(bool);
    void SetFinishPose(bool);
    
    void Plan();
    
    void StopPlan();
    
    void SetCorrectTrajectory(bool);
    
    void Execute();
    
    void OpenStaticGripper();
    void CloseStaticGripper();
    
    bool IsPlanned();
    
    void Clear();
    
    void ClearOctomap();
    
    void test();
    
private:
    bool Object;
    bool ObjectDetection;
    bool ObjectAdhesion;
    bool GraspChooser;
    bool StartPose;
    bool FinishPose;
    bool CorrectTrajectory;
    bool Planned;

    std::string file_path;
    
    ros::NodeHandle nh;
    
    ros::Publisher trajectory_display_publisher;
    
    ros::ServiceClient GetObjectIdServiceClient;
    ros::ServiceClient clear_octomap_service_client_;
    
    GuiPickAndPlace* gui;
    
    Scene* scene;
    
    moveit::planning_interface::MoveGroup* move_group;
    boost::shared_ptr<moveit::planning_interface::MoveGroup> move_groupPtr;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    planning_scene::PlanningScenePtr planning_scene;   
    
    TFServer* tfServer;
    boost::shared_ptr<TFServer> tfServerPtr;
    MarkerServer* markerServer;
    boost::shared_ptr<MarkerServer> markerServerPtr;
    
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interaktiveMarkerServer;
    
    MarkerServerManager* markerServerManager;
    TFServerManager* tfServerManager;
    InteractiveMarkerServerManager* interaktiveMarkerServerManager;  
    
    IMsgsParser* msgs_parser;
    IMsgsParser* ork_parser;
    IMsgsParser* obj_parser;
    
    IGripperController* wsg_50_controller;
    IGripperController* egn_160_controller;
    
    MoveGroupMotionPlan* motion_plan;
    
    IGraspReader* grasps_reader;
    
    PickAndPlacePlanner* pick_and_place_planner;
    MoveGroupMotionPlanner* motion_planner;
    
    ICollisionObjectController* collision_objects_controller;    
    
    MoveGroupExecuter* executer;
    
    visualization_msgs::Marker object;
    
    visualization_msgs::Marker start_object_marker;
    visualization_msgs::Marker finish_object_marker;
    
    std::string start_object_name;
    std::string finish_object_name;
    
    moveit::tools::BackgroundProcessing background_process_;
    
    void addBackgroundJob(const boost::function<void()> &job, const std::string &name);
    void addPlanThread();
    void PlanThread();
    
    void addExecutionThread();
    void ExecutionThread();
    
    void PublishTrajectory();
    void SetReadingPath(std::string);
    void ReadGrasps();
    void SetStartObjectMarker(visualization_msgs::Marker);
    void SetStartObjectMarker();
    void SetFinishObjectMarker(visualization_msgs::Marker);
    void SetFinishObjectMarker();
    
    void DeleteObjectMarkers();
    
    void StartInteractiveServers();

};

}

#endif