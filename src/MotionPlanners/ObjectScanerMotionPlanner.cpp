#include "operation_panel/MotionPlanners/ObjectScanerMotionPlanner.h"
#include <operation_panel/robot_config.h>

ObjectScanerMotionPlanner::ObjectScanerMotionPlanner()
{
#ifdef KUKA
    kuka_group_ptr = boost::shared_ptr< moveit::planning_interface::MoveGroup > (new moveit::planning_interface::MoveGroup ("kuka"));
#endif
#ifdef KAWASAKI
    kuka_group_ptr = boost::shared_ptr< moveit::planning_interface::MoveGroup > (new moveit::planning_interface::MoveGroup ("right_arm"));
#endif
    robot_state::RobotState kuka_start_state(*(kuka_group_ptr->getCurrentState()));
    kuka_joint_model_group = kuka_start_state.getJointModelGroup(kuka_group_ptr->getName());
    
    kuka_group_ptr->setStartState(kuka_start_state);
    kuka_group_ptr->allowReplanning(false);
    kuka_group_ptr->setPlannerId("RRTConnectkConfigDefault");
#ifdef KUKA
    kawasaki_group_ptr = boost::shared_ptr< moveit::planning_interface::MoveGroup > (new moveit::planning_interface::MoveGroup ("kawasaki"));
#endif
#ifdef KAWASAKI
    kawasaki_group_ptr = boost::shared_ptr< moveit::planning_interface::MoveGroup > (new moveit::planning_interface::MoveGroup ("left_arm"));
#endif
    robot_state::RobotState kawasaki_start_state(*(kawasaki_group_ptr->getCurrentState()));
    kawasaki_joint_model_group = kawasaki_start_state.getJointModelGroup(kawasaki_group_ptr->getName());
    
    kawasaki_group_ptr->setStartState(kawasaki_start_state);
    kawasaki_group_ptr->allowReplanning(false);
    kawasaki_group_ptr->setPlannerId("RRTConnectkConfigDefault");
}
void ObjectScanerMotionPlanner::ArmToViewpoint()
{
    double deg_to_rad = M_PI / 180;
    std::vector<double> angles;  
    angles.resize(6);  
//     angles[0] = -48.6  * deg_to_rad;       
//     angles[1] = -98.89 * deg_to_rad;   
//     angles[2] =  97.98 * deg_to_rad;     
//     angles[3] = -57.00 * deg_to_rad;   
//     angles[4] = 104.96 * deg_to_rad;   
//     angles[5] = 96.40  * deg_to_rad;   
#ifdef KUKA  
    angles[0] = -28.00  * deg_to_rad;       
    angles[1] = -78.99 * deg_to_rad;   
    angles[2] = 123.77 * deg_to_rad;     
    angles[3] = -58.32 * deg_to_rad;   
    angles[4] = 110.51 * deg_to_rad;   
    angles[5] = 140.40  * deg_to_rad;  
#endif    
#ifdef KAWASAKI  
    angles[0] = -0.068  * deg_to_rad;       
    angles[1] = 7.397 * deg_to_rad;   
    angles[2] = -108.914 * deg_to_rad;     
    angles[3] = -69.871 * deg_to_rad;   
    angles[4] = -63.173 * deg_to_rad;   
    angles[5] = 102.539  * deg_to_rad;  
#endif    
    robot_state::RobotState start_state(*kuka_group_ptr->getCurrentState());
    kuka_group_ptr->setStartState(start_state);
    kuka_group_ptr->setJointValueTarget(angles);       
    bool success = kuka_group_ptr->plan(kuka_plan);
    ROS_INFO("Viewpoint kuka %s",success?"SUCCESS":"FAILED"); 
    KukaExecution();
}
void ObjectScanerMotionPlanner::RotateTableToStartPosition()
{
    double deg_to_rad = M_PI / 180;
    std::vector<double> angles;  
    angles.resize(6);
#ifdef KUKA  
    angles[0] =-132.33  * deg_to_rad;       
    angles[1] = 126.42  * deg_to_rad;  
    angles[2] =  71.98  * deg_to_rad;    
    angles[3] =  -0.18  * deg_to_rad;   
    angles[4] =  54.25  * deg_to_rad;   
    angles[5] = 180.0   * deg_to_rad;                                          
#endif
#ifdef KAWASAKI  
    angles[0] = -32.296 * deg_to_rad;       
    angles[1] = 131.096 * deg_to_rad;  
    angles[2] = 26.752 * deg_to_rad;    
    angles[3] = 1.448 * deg_to_rad;   
    angles[4] = 101.565 * deg_to_rad;   
    angles[5] = 180.0 * deg_to_rad;                                          
#endif
    robot_state::RobotState start_state(*kawasaki_group_ptr->getCurrentState());
    kawasaki_group_ptr->setStartState(start_state);
    kawasaki_group_ptr->setJointValueTarget(angles);       
    bool success = kawasaki_group_ptr->plan(kawasaki_plan);
    ROS_INFO("Start position kawasaki %s",success?"SUCCESS":"FAILED"); 
    KawasakiExecution();
}
void ObjectScanerMotionPlanner::RotateTable(int cloud_count, float angle)
{
    double deg_to_rad = M_PI / 180;
    std::vector<double> angles;  
    angles.resize(6);
    #ifdef KUKA  
    angles[0] =-132.33  * deg_to_rad;       
    angles[1] = 126.42  * deg_to_rad;  
    angles[2] =  71.98  * deg_to_rad;    
    angles[3] =  -0.18  * deg_to_rad;   
    angles[4] =  54.25  * deg_to_rad;  
#endif
#ifdef KAWASAKI  
    angles[0] = -32.296 * deg_to_rad;       
    angles[1] = 131.096 * deg_to_rad;  
    angles[2] = 26.752 * deg_to_rad;    
    angles[3] = 1.448 * deg_to_rad;   
    angles[4] = 101.565 * deg_to_rad;     
#endif
  
    angles[5] = 180.0   * deg_to_rad - cloud_count * angle * deg_to_rad; 
    
    geometry_msgs::Pose kawasaki_target_pose;
    robot_state::RobotState start_state(*kawasaki_group_ptr->getCurrentState());
    start_state.setFromIK(kawasaki_joint_model_group, kawasaki_target_pose);
    kawasaki_group_ptr->setStartState(start_state);
    kawasaki_group_ptr->setJointValueTarget(angles);       
    bool success = kawasaki_group_ptr->plan(kawasaki_plan);
    ROS_INFO("Survey %s",success?"SUCCESS":"FAILED"); 
    KawasakiExecution();
}
void ObjectScanerMotionPlanner::KukaExecution()
{
    kuka_group_ptr->asyncExecute(kuka_plan); 
    WaitForKukaExecution();
}
void ObjectScanerMotionPlanner::KawasakiExecution()
{
    kawasaki_group_ptr->asyncExecute(kawasaki_plan); 
    WaitForKawasakiExecution();
}
void ObjectScanerMotionPlanner::WaitForKukaExecution()
{
    robot_state::RobotState prev_state(*kuka_group_ptr->getCurrentState());
    robot_state::RobotState curr_state(*kuka_group_ptr->getCurrentState());
    bool moving = true;
    ros::Duration(2.0).sleep();
    ROS_INFO_STREAM("START WAITING");

    while (moving)
    {
	ros::Duration(1.0).sleep();
	curr_state = *kuka_group_ptr->getCurrentState();
	std::vector<double> diffs;

	diffs.push_back(*(curr_state.getJointPositions(JOINT1NAME)) - *(prev_state.getJointPositions(JOINT1NAME)));
	diffs.push_back(*(curr_state.getJointPositions(JOINT2NAME)) - *(prev_state.getJointPositions(JOINT2NAME)));
	diffs.push_back(*(curr_state.getJointPositions(JOINT3NAME)) - *(prev_state.getJointPositions(JOINT3NAME)));
	diffs.push_back(*(curr_state.getJointPositions(JOINT4NAME)) - *(prev_state.getJointPositions(JOINT4NAME)));
	diffs.push_back(*(curr_state.getJointPositions(JOINT5NAME)) - *(prev_state.getJointPositions(JOINT5NAME)));
	diffs.push_back(*(curr_state.getJointPositions(JOINT6NAME)) - *(prev_state.getJointPositions(JOINT6NAME)));
	for (std::vector<double>::iterator diff_it = diffs.begin(); diff_it != diffs.end(); ++diff_it)
	{
	    if (fabs(*diff_it) > 0.0005)
	    {
		moving = true;
		break;
	    }
	    else 
	    {
		moving = false;
	    }
	}

	prev_state = curr_state;
    }

    ROS_INFO_STREAM("STOP WAITING");
}
void ObjectScanerMotionPlanner::WaitForKawasakiExecution()
{
    robot_state::RobotState prev_state(*kawasaki_group_ptr->getCurrentState());
    robot_state::RobotState curr_state(*kawasaki_group_ptr->getCurrentState());
    bool moving = true;
    ros::Duration(2.0).sleep();
    ROS_INFO_STREAM("START WAITING");

    while (moving)
    {
	ros::Duration(1.0).sleep();
	curr_state = *kuka_group_ptr->getCurrentState();
	std::vector<double> diffs;
	diffs.push_back(*(curr_state.getJointPositions(SUPJOINT1NAME)) - *(prev_state.getJointPositions(SUPJOINT1NAME)));
	diffs.push_back(*(curr_state.getJointPositions(SUPJOINT2NAME)) - *(prev_state.getJointPositions(SUPJOINT2NAME)));
	diffs.push_back(*(curr_state.getJointPositions(SUPJOINT3NAME)) - *(prev_state.getJointPositions(SUPJOINT3NAME)));
	diffs.push_back(*(curr_state.getJointPositions(SUPJOINT4NAME)) - *(prev_state.getJointPositions(SUPJOINT4NAME)));
	diffs.push_back(*(curr_state.getJointPositions(SUPJOINT5NAME)) - *(prev_state.getJointPositions(SUPJOINT5NAME)));
	diffs.push_back(*(curr_state.getJointPositions(SUPJOINT6NAME)) - *(prev_state.getJointPositions(SUPJOINT6NAME)));
	for (std::vector<double>::iterator diff_it = diffs.begin(); diff_it != diffs.end(); ++diff_it)
	{
	    if (fabs(*diff_it) > 0.0005)
	    {
		moving = true;
		break;
	    }
	    else 
	    {
		moving = false;
	    }
	}

	prev_state = curr_state;
    }

    ROS_INFO_STREAM("STOP WAITING");
}