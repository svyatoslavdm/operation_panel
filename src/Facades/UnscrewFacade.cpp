#include <operation_panel/Facades/UnscrewFacade.h>
#include <operation_panel/GUI/UnscrewGUI.h>
#include <operation_panel/robot_config.h>
namespace rviz
{

UnscrewFacade::UnscrewFacade(ros::NodeHandle _n)
{
    n = _n;
    
    tfServer = new TFServer(n);
    tfServer->start();    
    tfServerPtr.reset(tfServer);
    
    markerServer = new MarkerServer(n, "markers");
    markerServer->start();
    markerServerPtr.reset(markerServer);

    interaktiveMarkerServerPtr.reset( new interactive_markers::InteractiveMarkerServer("markers_control_unscrew","",false) );
    interaktiveMarkerServerPtr->applyChanges();
 
    scene = new Scene(n); 
    object_pose_name = "object_pose";
    object_axis_name = "unscrew_axis";
    
    msgsParser = new ObjectMsgsParser(n); 
        
    groupPtr = boost::shared_ptr< moveit::planning_interface::MoveGroup > (new moveit::planning_interface::MoveGroup (GROUPNAME));
    groupPtr->allowReplanning(true);
    groupPtr->setPlannerId("RRTkConfigDefault");
    
    waiter = new ExecutionWaiter(groupPtr.get());    
        
    setObjectMarker = false;
    setObjectIntMarker = false;
    setAxisMarker = false;
    setGraspMarker = false;
        
    markerServerManager = new MarkerServerManager(markerServerPtr, scene);
    tfServerManager = new TFServerManager(markerServerPtr, tfServerPtr, scene);
    interaktiveMarkerServerManager = new InteractiveMarkerServerManager(markerServerPtr, tfServerPtr, interaktiveMarkerServerPtr, scene); 
        
    isTerminated = false;
    hasGoal = false;
    actionThread = std::thread(&UnscrewFacade::SendActionGoal, this);    
    placeActionThread = std::thread(&UnscrewFacade::SendPlaceGoal, this);    
    	
    maxPullOutForce = 4;
    graspPosture = 109.0;
    desiredDistance = 0.1;
    
    SetViewpointJoints();
}        

void UnscrewFacade::SetGUI(GuiUnscrew* _gui)
{
    gui = _gui;
}

void UnscrewFacade::SetClient(ActionUnscrewClient* _client)
{
    actionClient = _client;
}

void UnscrewFacade::StartViewPointThread()
{
    background_process.clear();
    AddBackgroundJob(boost::bind(&UnscrewFacade::ViewPointMotion, this), "Go to viewpoint");  
}

void UnscrewFacade::ViewPointMotion()
{
    //Going to viewpoint
    robot_state::RobotState startState(*groupPtr->getCurrentState());
    jointModelGroup = startState.getJointModelGroup(groupPtr->getName());
//     groupPtr->setStartState(startState);
    bool success = groupPtr->plan(plan);
    ROS_INFO("Planning motion to viewpoint: %s",success?"SUCCESS":"FAILED"); 
    if (success)
    {
	groupPtr->asyncExecute(plan); 
	waiter->Wait();
    }    
    gui->InViewPoint();
}

void UnscrewFacade::SetViewpointJoints()
{ 
    double deg_to_rad = M_PI / 180;
    
    viewPointJointValues.resize(6);
/*    viewPointJointValues[0] = -65.56  * deg_to_rad;
    viewPointJointValues[1] = -101.73 * deg_to_rad;
    viewPointJointValues[2] =  78.13  * deg_to_rad;
    viewPointJointValues[3] =  48.27  * deg_to_rad;
    viewPointJointValues[4] =  103.46 * deg_to_rad;
    viewPointJointValues[5] = -87.73  * deg_to_rad;*/   

//     viewPointJointValues[0] = -1.4730578886832142;
//     viewPointJointValues[1] = -1.5725416560468908;
//     viewPointJointValues[2] =  1.99142067652553;
//     viewPointJointValues[3] =  1.1466813185602747;
//     viewPointJointValues[4] =  1.8291050560900572;
//     viewPointJointValues[5] = -2.150245638457014;
    
    //double jt[] = {-1.361356816555577, -1.5027284859671177, 1.6214108751027323, 0.7976154681614086, 1.9338248112097172, 1.160643952576229};
    double jt[] = {0.9494591130849153, -2.9426251188624395, 2.0001473227855016, -0.5602506898901798, 1.3473941825396225, 0.40491638646268446};
    viewPointJointValues = std::vector<double>(jt, jt+6);
    
    groupPtr->setJointValueTarget(viewPointJointValues);    
}

void UnscrewFacade::SetUnscrewAxis()
{
    Eigen::Vector3f unitZ;
    unitZ << 0.0, 0.0, 1.0;
    
    Eigen::Vector3f arrow_world_position;
    Eigen::Quaternionf arrow_world_rotation;
    
    if (tfServer->get("/world", "/egn_160/base_link", arrow_world_position, arrow_world_rotation))
    {
	Eigen::Vector3f axis;
	axis = arrow_world_rotation * unitZ;
	unscrewAxis.vector.x = axis.x();
	unscrewAxis.vector.y = axis.y();
	unscrewAxis.vector.z = axis.z();
    }
}

void UnscrewFacade::SetGraspMarker(bool _flag)
{
    if (_flag != setGraspMarker)
    {
	setGraspMarker = _flag;
	if (_flag)
	{
	    markerServerManager->CreateUnscrewChooser("unscrew");
	    tfServerManager->CreateUnscrewChooser("unscrew");
	    interaktiveMarkerServerManager->CreateUnscrewChooser("unscrew");
	    ros::Duration(0.1).sleep();
	    ROS_WARN("SetGraspMarker");
	}
	else 
	{
	    markerServerManager->RemoveUnscrewChooser("unscrew");
	    tfServerManager->RemoveUnscrewChooser("unscrew");
	    interaktiveMarkerServerManager->RemoveUnscrewChooser("unscrew");
	    ros::Duration(0.1).sleep();
	}
    }
}

void UnscrewFacade::SetUnscrewGrasp()
{
    geometry_msgs::Pose pose;
    tfServerPtr->get("world", "unscrew_marker", pose);    
    graspPose.header.frame_id = "/world";
    graspPose.header.stamp = ros::Time().now();
    graspPose.pose = pose;
    
    geometry_msgs::Pose arrow_pose;
    tfServerPtr->get("world", "unscrew_arrow", arrow_pose); 
    unscrewAxis.header.frame_id = "/world";
    unscrewAxis.header.stamp = ros::Time().now();    
    unscrewAxis.origin = arrow_pose.position;
}

void UnscrewFacade::SetManualTune(bool flag)
{
    manualTune = flag;
}

void UnscrewFacade::SetDirection(bool _flag)
{
    clockwise = _flag;
}

void UnscrewFacade::SetAxisTolerance(float _val)
{
    axisTolerance = _val;
}

void UnscrewFacade::SetMaxGraspForce(float _val)
{
    maxGraspForce = _val;
}

void UnscrewFacade::SetMaxPullForce(float _val)
{
    maxPullOutForce = _val;
}

void UnscrewFacade::SetMaxCycles(int _val)
{
    maxCycles = _val;
}

void UnscrewFacade::SetRotatePerCycle(int _val)
{
    rotatePerCycle = _val;
} 

void UnscrewFacade::Plan()
{      
    auto goal = new ActionUnscrewClient::Goal;
    goal->group = GROUPNAME;    
    goal->axis_tolerance = axisTolerance;
    goal->clockwise = clockwise;
    goal->desired_distance = desiredDistance;
    goal->end_effector_link = "/wsg_50/palm_link";    
    goal->grasp_pose = graspPose;
    goal->grasp_posture = graspPosture;
    goal->rotate_per_cycle = rotatePerCycle / 180 * 3.1415;
    goal->max_cycles = maxCycles;
    goal->max_grasp_force = maxGraspForce;
    goal->max_pull_out_force = maxPullOutForce;
    goal->unscrew_axis = unscrewAxis;
    goal->manual_tune = manualTune;    
    goal->plan_only = true;     
    
    std::lock_guard<std::mutex> lock(actionMutex);    
    goalPtr.reset(goal);
}

void UnscrewFacade::Execute()
{       
    auto goal = new ActionUnscrewClient::Goal;
    goal->group = GROUPNAME;    
    goal->axis_tolerance = axisTolerance;
    goal->clockwise = clockwise;
    goal->desired_distance = desiredDistance;
    goal->end_effector_link = "/wsg_50/palm_link";    
    goal->grasp_pose = graspPose;
    goal->grasp_posture = graspPosture;
    goal->rotate_per_cycle = rotatePerCycle / 180 * 3.1415;  
    goal->max_cycles = maxCycles;
    goal->max_grasp_force = maxGraspForce;
    goal->max_pull_out_force = maxPullOutForce;
    goal->unscrew_axis = unscrewAxis;
    goal->manual_tune = manualTune;    
    goal->plan_only = false;     
    
    std::lock_guard<std::mutex> lock(actionMutex);    
    goalPtr.reset(goal);
}

void UnscrewFacade::Abort()
{   
    std::lock_guard<std::mutex> lock(actionMutex);  
    actionClient->Abort();
}

void UnscrewFacade::Place()
{   
    auto goal = new ActionUnscrewClient::PlaceGoal;   
    
    std::lock_guard<std::mutex> lock(actionMutex);    
    placeGoalPtr.reset(goal);
}

void UnscrewFacade::SendActionGoal()
{   
    while (!isTerminated)
    {
	if (goalPtr)
	{
	   std::lock_guard<std::mutex> lock(actionMutex);
    	   actionClient->Send(*goalPtr);
	   goalPtr.reset();
	}
	ros::Duration(0.1).sleep();
    }
}

void UnscrewFacade::SendPlaceGoal()
{   
    while (!isTerminated)
    {
	if (placeGoalPtr)
	{
	   std::lock_guard<std::mutex> lock(actionMutex);
    	   actionClient->Send(*placeGoalPtr);
	   placeGoalPtr.reset();
	}
	ros::Duration(0.1).sleep();
    }
}


void UnscrewFacade::AbortActionGoal()
{
    actionClient->Abort();
}

void UnscrewFacade::AddBackgroundJob(const boost::function<void()> &job, const std::string &name)
{
    background_process.addJob(job, name);
}

}
