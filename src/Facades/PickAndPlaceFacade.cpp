#include <operation_panel/Facades/PickAndPlaceFacade.h>

#include <operation_panel/GUI/PickAndPlaceGUI.h>

#include <operation_panel/CollisionObjects/AddObjectCommand.h>

#include <std_srvs/Empty.h>

#include <operation_panel/robot_config.h>

namespace rviz
{
    
PickAndPlaceFacade::PickAndPlaceFacade(ros::NodeHandle _nh)
{
    nh = _nh;
    
    clear_octomap_service_client_ = nh.serviceClient<std_srvs::Empty>("/clear_octomap");
    move_group = new moveit::planning_interface::MoveGroup(GROUPNAME);
    move_groupPtr.reset(move_group);
    
    boost::shared_ptr<tf::TransformListener> tf_listener_(new tf::TransformListener(ros::Duration(2.0)));
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf_listener_));
    planning_scene_monitor_->startSceneMonitor();  
    tfServer = new TFServer(nh);
    tfServer->start();
    
    tfServerPtr.reset(tfServer);

    markerServer = new MarkerServer(nh, "markers");
    markerServer->start();
    markerServerPtr.reset(markerServer);
    
    interaktiveMarkerServer.reset( new interactive_markers::InteractiveMarkerServer("markers_control_pick_and_place","",false) );
    interaktiveMarkerServer->applyChanges();
    scene = new Scene(nh);    
    markerServerManager = new MarkerServerManager(markerServerPtr, scene);
    tfServerManager = new TFServerManager(markerServerPtr, tfServerPtr, scene);
    interaktiveMarkerServerManager = new InteractiveMarkerServerManager(markerServerPtr, tfServerPtr, interaktiveMarkerServer, scene); 
    collision_objects_controller = new CollisionObjectUtilities(move_group, tfServer, markerServer);
    obj_parser = new ObjectMsgsParser(nh); 
    ork_parser = new PoseMsgsParser(nh);   
    msgs_parser = obj_parser;
    wsg_50_controller = new WSG50_Controller(nh);
    egn_160_controller = new EGN160_Controller(nh);
    motion_plan = new MoveGroupMotionPlan();
    pick_and_place_planner = new PickAndPlacePlanner(nh, tfServer, wsg_50_controller, motion_plan, move_group, planning_scene_monitor_, collision_objects_controller);
    motion_planner = new MoveGroupMotionPlanner(move_group, tfServer, planning_scene_monitor_, collision_objects_controller);
    executer = new MoveGroupExecuter(move_group, collision_objects_controller, egn_160_controller);
    trajectory_display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path_", 1, true);
    GetObjectIdServiceClient = nh.serviceClient<ork_gui::GetObject>("GetObject");
    egn_160_controller->Open(); 
    Object = false;
    GraspChooser = true;
    
    collision_detection::AllowedCollisionMatrix acm = planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrixNonConst();
    
#ifdef KUKA
    acm.setEntry("<octomap>", "operation_table", true);
    acm.setEntry("<octomap>", "cross", true);
    acm.setEntry("<octomap>", "egn_160/base_link", true);
    acm.setEntry("<octomap>", "egn_160/finger_left", true);
    acm.setEntry("<octomap>", "egn_160/finger_right", true);
    acm.setEntry("<octomap>", "egn_160/gripper_left", true);
    acm.setEntry("<octomap>", "egn_160/gripper_right", true);
    acm.setEntry("<octomap>", "egn_160/palm_link", true);
    acm.setEntry("<octomap>", "kaw/base_link", true);
    acm.setEntry("<octomap>", "kaw/link_1", true);
    acm.setEntry("<octomap>", "kaw/link_2", true);
    acm.setEntry("<octomap>", "kaw/link_3", true);
    acm.setEntry("<octomap>", "kaw/link_4", true);
    acm.setEntry("<octomap>", "kaw/link_5", true);
    acm.setEntry("<octomap>", "kaw/link_6", true);
    acm.setEntry("<octomap>", "kaw/fts_link", true);
    acm.setEntry("<octomap>", "rotary_table", true);
    
    acm.setEntry("<octomap>", "kuka/base_link", true);
    acm.setEntry("<octomap>", "kuka/link_1", true);
    acm.setEntry("<octomap>", "kuka/link_2", true);
#endif
#ifdef KAWASAKI
    acm.setEntry("<octomap>", "operation_table", true);
    acm.setEntry("<octomap>", "cross", true);
    acm.setEntry("<octomap>", "egn_160/base_link", true);
    acm.setEntry("<octomap>", "egn_160/finger_left", true);
    acm.setEntry("<octomap>", "egn_160/finger_right", true);
    acm.setEntry("<octomap>", "egn_160/gripper_left", true);
    acm.setEntry("<octomap>", "egn_160/gripper_right", true);
    acm.setEntry("<octomap>", "egn_160/palm_link", true);

    
    acm.setEntry("<octomap>", "r/base_link", true);
    acm.setEntry("<octomap>", "r/link_1", true);
    acm.setEntry("<octomap>", "r/link_2", true);    
    acm.setEntry("<octomap>", "r/link_3", true);
    
    acm.setEntry("<octomap>", "l/base_link", true);
    acm.setEntry("<octomap>", "l/link_1", true);
    acm.setEntry("<octomap>", "l/link_2", true);    
    acm.setEntry("<octomap>", "l/link_3", true);        
#endif
}
void PickAndPlaceFacade::SetGUI(GuiPickAndPlace* _gui)
{
    gui = _gui;    
}
void PickAndPlaceFacade::SetObjectDetection(bool _flag)
{
    ObjectDetection = _flag;

    if (ObjectDetection)
    {
	msgs_parser = ork_parser;
    }
    else
    {
	msgs_parser = obj_parser;
    }
}
void PickAndPlaceFacade::SetObject()
{
    visualization_msgs::Marker _object;
    if (msgs_parser->GetObject(_object))
    {
	SetObjectAdhesion(false);
	
	SetReadingPath(_object.ns);
	
	if (Object)
	{
	    DeleteObjectMarkers();
	}
	
	SetStartObjectMarker(_object);
	SetFinishObjectMarker(_object);
	
	StartInteractiveServers();
    
	gui->OnDetected();
	
	Object = true;
    }
}
void PickAndPlaceFacade::StartInteractiveServers()
{
//     interaktiveMarkerServerManager->Remove(start_object_name); 
//     interaktiveMarkerServerManager->Remove(finish_object_name);
//     
//     tfServerManager->Remove(start_object_name); 
//     tfServerManager->Remove(finish_object_name);

    tfServerManager->CreateObject(start_object_name); 
    tfServerManager->CreateObject(finish_object_name);
    
    interaktiveMarkerServerManager->CreateObject(start_object_name); 
    interaktiveMarkerServerManager->CreateObject(finish_object_name);
    
    interaktiveMarkerServerManager->UpdateRotaryTableInformation();    
}
void PickAndPlaceFacade::DeleteObjectMarkers()
{
    interaktiveMarkerServerManager->Remove(start_object_name); 
    interaktiveMarkerServerManager->Remove(finish_object_name);
    
    markerServer->remove(start_object_name);
    markerServer->remove(finish_object_name);    
    
    tfServerManager->Remove(start_object_name); 
    tfServerManager->Remove(finish_object_name);

    tfServer->remove("world", start_object_name + "_marker");
    tfServer->remove("world", finish_object_name + "_marker");
    
}
void PickAndPlaceFacade::SetObjectAdhesion(bool _flag)
{
    ObjectAdhesion = _flag;
    interaktiveMarkerServerManager->SetAdherence(ObjectAdhesion);
}
void PickAndPlaceFacade::SetStartPose(bool _flag)
{
    StartPose = _flag;
    if (StartPose)
    {
	tfServerManager->Remove(start_object_name); 
	interaktiveMarkerServerManager->Remove(start_object_name); 
    }
    else
    {
	tfServerManager->CreateObject(start_object_name); 
	interaktiveMarkerServerManager->CreateObject(start_object_name); 
    }
}
void PickAndPlaceFacade::SetFinishPose(bool _flag)
{
    FinishPose = _flag;
    if (FinishPose)
    {
	tfServerManager->Remove(finish_object_name); 
	interaktiveMarkerServerManager->Remove(finish_object_name); 
    }
    else
    {
	tfServerManager->CreateObject(finish_object_name); 
	interaktiveMarkerServerManager->CreateObject(finish_object_name); 
    }
}

void PickAndPlaceFacade::SetGraspChoser(bool _flag)
{
    GraspChooser = _flag;
    if (!GraspChooser)
    {
	tfServerManager->CreateGraspChooser(start_object_name);
	markerServerManager->CreateGraspChooser(start_object_name);	
	interaktiveMarkerServerManager->CreateGraspChooser(start_object_name);
	
    }
    else 
    {
	interaktiveMarkerServerManager->RemoveGraspChooser(start_object_name);
	markerServerManager->RemoveGraspChooser(start_object_name);
	tfServerManager->RemoveGraspChooser(start_object_name);	
    }
}

void PickAndPlaceFacade::Plan()
{
    addPlanThread();
}
void PickAndPlaceFacade::StopPlan()
{
    motion_planner->StopPlan();
}
void PickAndPlaceFacade::addPlanThread()
{
    background_process_.clear();
    addBackgroundJob(boost::bind(&PickAndPlaceFacade::PlanThread, this), "PlanThread");    
}
void PickAndPlaceFacade::PlanThread()
{
    if (StartPose && FinishPose)
    {
	ROS_INFO_STREAM("Start planning");
	motion_plan->Clear();
	pick_and_place_planner->SetPath(file_path);
	pick_and_place_planner->SetPlanningType(GraspChooser);
	pick_and_place_planner->Plan();
	
	ros::Duration(0.2).sleep();
	
	motion_planner->SetPlan(motion_plan);
	Planned =  motion_planner->Plan();
	
	if (Planned)
	{
	    executer->SetMotionPlan(motion_plan);
	    PublishTrajectory();
	}
	
	gui->OnPlanned();
    }
}
void PickAndPlaceFacade::PublishTrajectory()
{
    std::vector<moveit::planning_interface::MoveGroup::Plan> plans = motion_plan->GetPlans();
    
    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start = plans.at(0).start_state_;
    
    for (const auto& plan : plans)
    {
	display_trajectory.trajectory.push_back(plan.trajectory_);
    }
    trajectory_display_publisher.publish(display_trajectory);
}

void PickAndPlaceFacade::SetCorrectTrajectory(bool _flag)
{
    CorrectTrajectory = _flag;

}
void PickAndPlaceFacade::Execute()
{
    addExecutionThread();
}
void PickAndPlaceFacade::addExecutionThread()
{
    background_process_.clear();
    addBackgroundJob(boost::bind(&PickAndPlaceFacade::ExecutionThread, this), "ExecutionThread");    
}
void PickAndPlaceFacade::ExecutionThread()
{
    if (Planned)
    {
	executer->Execute();
	gui->OnExecuted();
	
	SetStartObjectMarker();
	SetFinishObjectMarker();
	StartInteractiveServers();	
    }
}

void PickAndPlaceFacade::OpenStaticGripper()
{
    egn_160_controller->Open();
}
void PickAndPlaceFacade::CloseStaticGripper()
{
    egn_160_controller->Close();
}

bool PickAndPlaceFacade::IsPlanned()
{
    return Planned;
}
void PickAndPlaceFacade::SetReadingPath(std::string _name)
{   
    file_path = ros::package::getPath("operation_panel");
    std::string wsg50_finger;   
    nh.getParam("wsg50_finger", wsg50_finger);
    file_path += "/grasps/" + _name + "_wsg50_" + wsg50_finger + ".txt";
}
void PickAndPlaceFacade::ReadGrasps()
{

}

void PickAndPlaceFacade::SetStartObjectMarker(visualization_msgs::Marker _object)
{
    start_object_marker = _object;
    
    geometry_msgs::Pose pose_object = start_object_marker.pose;
    geometry_msgs::Pose pose_world;
    
    start_object_name = "start_object";
    
    if (start_object_marker.header.frame_id != "world")
    {
	tfServer->remove("world", start_object_name + "_marker");   
	tfServer->set(start_object_marker.header.frame_id, start_object_name + "_marker_aux", pose_object);    
	tfServer->get("world", start_object_name + "_marker_aux", pose_world);
	tfServer->remove(start_object_marker.header.frame_id, start_object_name + "_marker_aux");
	tfServer->set("world", start_object_name + "_marker", pose_world);
    }
    else 
    {
	tfServer->set("world", start_object_name + "_marker", pose_object);
    }

    start_object_marker.pose.position.x = 0;
    start_object_marker.pose.position.y = 0;
    start_object_marker.pose.position.z = 0;
    
    start_object_marker.pose.orientation.x = 0;
    start_object_marker.pose.orientation.y = 0;
    start_object_marker.pose.orientation.z = 0;
    start_object_marker.pose.orientation.w = 1;
    
    start_object_marker.header.frame_id = start_object_name + "_marker";
    
    start_object_marker.color.r = 0.0;
    start_object_marker.color.g = 0.0;
    start_object_marker.color.b = 1.0;
    start_object_marker.color.a = 1.0;
    
    markerServer->set(start_object_name, start_object_marker);
}
void PickAndPlaceFacade::SetStartObjectMarker()
{
    geometry_msgs::Pose pose_world;
    tfServer->get("world", finish_object_name + "_marker", pose_world);
    tfServer->set("world", start_object_name + "_marker", pose_world);     
    
}
void PickAndPlaceFacade::SetFinishObjectMarker(visualization_msgs::Marker _object)
{
    // Add finish marker
    finish_object_marker = _object;
    geometry_msgs::Pose pose_world;
    
    geometry_msgs::Pose pose_object = finish_object_marker.pose;
//     pose_object.position.x -= 0.4;
	    
    finish_object_name = "finish_object";
    start_object_name = "start_object";

    tfServer->get("world", start_object_name + "_marker", pose_world);
    pose_world.position.x -= 0.4;
    tfServer->set("world", finish_object_name + "_marker", pose_world);	


    
    finish_object_marker.pose.position.x = 0;
    finish_object_marker.pose.position.y = 0;
    finish_object_marker.pose.position.z = 0;
    
    finish_object_marker.pose.orientation.x = 0;
    finish_object_marker.pose.orientation.y = 0;
    finish_object_marker.pose.orientation.z = 0;
    finish_object_marker.pose.orientation.w = 1;
    
    finish_object_marker.header.frame_id = finish_object_name + "_marker";
    
    finish_object_marker.color.r = 0.0;
    finish_object_marker.color.g = 1.0;
    finish_object_marker.color.b = 0.0;
    finish_object_marker.color.a = 1.0;
    
    markerServer->set(finish_object_name, finish_object_marker);
}
void PickAndPlaceFacade::SetFinishObjectMarker()
{
    geometry_msgs::Pose pose_world;
    tfServer->get("world", finish_object_name + "_marker", pose_world);    
    pose_world.position.y += 0.20;    
    tfServer->set("world", finish_object_name + "_marker", pose_world);   
}
void PickAndPlaceFacade::test()
{
    
}
void PickAndPlaceFacade::ClearOctomap()
{
    std_srvs::Empty msg;
    clear_octomap_service_client_.call(msg);
    ros::Publisher octomap_pub = nh.advertise<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 1);
    moveit_msgs::PlanningScene planning_scene;    
    moveit_msgs::AllowedCollisionMatrix acm;
    planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrix().getMessage(acm);
    planning_scene.allowed_collision_matrix = acm;   
    
    for (int i = 0; i < 10; i++)
    {
	octomap_pub.publish(planning_scene);
	ros::spinOnce();
	ros::Duration(0.02).sleep();
    }
    
}

void PickAndPlaceFacade::Clear()
{
    if (Object)
    {
	interaktiveMarkerServerManager->Remove(start_object_name);    
	interaktiveMarkerServerManager->Remove(finish_object_name);
	
	tfServerManager->Remove(start_object_name); 
	tfServerManager->Remove(finish_object_name);
	
	tfServer->remove("world", start_object_name + "_marker");
	tfServer->remove("world", finish_object_name + "_marker");
	
	markerServer->remove(start_object_name);
	markerServer->remove(finish_object_name);
	
	Object = false;
    }
    
    if (!GraspChooser)
    {
	interaktiveMarkerServerManager->RemoveGraspChooser(start_object_name);
	markerServerManager->RemoveGraspChooser(start_object_name);
	tfServerManager->RemoveGraspChooser(start_object_name);	
    }
}

void PickAndPlaceFacade::addBackgroundJob(const boost::function<void()> &job, const std::string &name)
{
    background_process_.addJob(job, name);
}

}
