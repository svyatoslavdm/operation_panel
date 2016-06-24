#include "operation_panel/MotionPlanners/OpenBagMotionPlanner.h"
#include <operation_panel/robot_config.h>
OpenBagMotionPlanner::OpenBagMotionPlanner(boost::shared_ptr<TFServer> tfServer_) //, ros::NodeHandle& nh_)
{
    groupPtr = boost::shared_ptr< moveit::planning_interface::MoveGroup > (new moveit::planning_interface::MoveGroup (GROUPNAME));
    robot_state::RobotState start_state(*groupPtr->getCurrentState());
    joint_model_group = start_state.getJointModelGroup(groupPtr->getName());

    groupPtr->setStartState(start_state);
    groupPtr->allowReplanning(false);
    groupPtr->setPlannerId("RRTConnectkConfigDefault");

    tfServer = tfServer_;
}

void OpenBagMotionPlanner::Execution()
{
    groupPtr->asyncExecute(plan); 
    WaitForExecution();
}

void OpenBagMotionPlanner::Viewpoint()
{
    double deg_to_rad = M_PI / 180;
    std::vector<double> angles;  
    angles.resize(6);  
    angles[0] = -65.56 * deg_to_rad;       
    angles[1] = -101.73 * deg_to_rad;   
    angles[2] =  78.13 * deg_to_rad;     
    angles[3] =  48.27 * deg_to_rad;   
    angles[4] =  103.46 * deg_to_rad;   
    angles[5] = -87.73 * deg_to_rad;                                 

    robot_state::RobotState start_state(*groupPtr->getCurrentState());
    groupPtr->setStartState(start_state);
    groupPtr->setJointValueTarget(angles);       
    bool success = groupPtr->plan(plan);
    ROS_INFO("Survey %s",success?"SUCCESS":"FAILED"); 
    Execution();
}

void OpenBagMotionPlanner::PlanBagOpeningMovement ()
{
    tf::Transform point0Transf, point1Transf, arrowTransf;
    
    tfServer->get("world", "point0_control", point0Transf);
    tfServer->get("world", "point1_control", point1Transf);
    tfServer->get("world", "arrow_control", arrowTransf);
    
    moveit_msgs::RobotTrajectory trajectory;
    std::vector<geometry_msgs::Pose> waypoints;
    
    robot_state::RobotState start_state(*groupPtr->getCurrentState());
    groupPtr->setStartState(start_state);
	
    waypoints = CalcOpeningTrajectory(point0Transf, point1Transf, arrowTransf);  
    
    double fraction = groupPtr->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    
//     ros::Time begin = ros::Time::now();
//     while(fraction != 1 && (ros::Time::now() - begin).toSec() < 5.0)
//     {
// 	fraction = groupPtr->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
//     }

    plan.trajectory_ = trajectory;
}

std::vector<geometry_msgs::Pose> OpenBagMotionPlanner::CalcOpeningTrajectory (tf::Transform point0Transf, tf::Transform point1Transf, tf::Transform arrowTransf)
{
    std::vector<geometry_msgs::Pose> waypoints;
    tf::Transform tr;
    geometry_msgs::Pose pose;
    
    Eigen::Vector3f point0_pos = Eigen::Vector3f(point0Transf.getOrigin().getX(), point0Transf.getOrigin().getY(), point0Transf.getOrigin().getZ());
    Eigen::Vector3f point1_pos = Eigen::Vector3f(point1Transf.getOrigin().getX(), point1Transf.getOrigin().getY(), point1Transf.getOrigin().getZ());
    Eigen::Vector3f arrow_pos = Eigen::Vector3f(arrowTransf.getOrigin().getX(), arrowTransf.getOrigin().getY(), arrowTransf.getOrigin().getZ());
    Eigen::Quaternionf arrow_quat = Eigen::Quaternionf(arrowTransf.getRotation().getW(), arrowTransf.getRotation().getX(), arrowTransf.getRotation().getY(), arrowTransf.getRotation().getZ());

    Eigen::Vector3f l_finger_toolFrame = Eigen::Vector3f(- 0.114, 0.0, - 0.240);
    float length = ((point1_pos - arrow_pos).cross(point0_pos - arrow_pos)).norm() / (point0_pos - point1_pos).norm();
    
    Eigen::Vector3f xGripper = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitZ(), arrow_quat.toRotationMatrix() * (-Eigen::Vector3f::UnitX())).toRotationMatrix() * Eigen::Vector3f::UnitX();
    Eigen::Quaternionf q_gripper = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitZ(), arrow_quat.toRotationMatrix() * (-Eigen::Vector3f::UnitX())) * 
				   Eigen::AngleAxisf(acos(xGripper.dot(-Eigen::Vector3f::UnitZ())) , -Eigen::Vector3f::UnitZ());

    tr.setRotation(tf::Quaternion(q_gripper.x(), q_gripper.y(), q_gripper.z(), q_gripper.w()));
    tr.setOrigin(tf::Vector3((arrow_pos + q_gripper.toRotationMatrix() * l_finger_toolFrame).x(), 
			     (arrow_pos + q_gripper.toRotationMatrix() * l_finger_toolFrame).y(), 
			     (arrow_pos + q_gripper.toRotationMatrix() * l_finger_toolFrame).z()));
    tfServer->set("world", "gripperTransform0", tr);
    
    tr.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    tr.setOrigin(tf::Vector3(0.0, 0.0, -0.75) * length);
    tfServer->set("gripperTransform0", "gripperTransform1", tr);
    tfServer->get("world", "gripperTransform1", pose);
    waypoints.push_back(pose);
    
    tr.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    tr.setOrigin(tf::Vector3(0.0, 0.0, 1.0) * length);
    tfServer->set("gripperTransform1", "gripperTransform2", tr);
    tfServer->get("world", "gripperTransform2", pose);
    waypoints.push_back(pose);
    
    tr.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    tr.setOrigin(tf::Vector3(- 0.5, 0.0, 1.0) * length);
    tfServer->set("gripperTransform2", "gripperTransform3", tr);
    tfServer->get("world", "gripperTransform3", pose);
    waypoints.push_back(pose);
    
    tr.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    tr.setOrigin(tf::Vector3(0.0, 0.0, 1.25) * length);
    tfServer->set("gripperTransform3", "gripperTransform4", tr);
    tfServer->get("world", "gripperTransform4", pose);
    waypoints.push_back(pose);
    
    return waypoints;
}

void OpenBagMotionPlanner::DeleteTrajectory()
{
    tfServer->remove("gripperTransform3", "gripperTransform4");
    tfServer->remove("gripperTransform2", "gripperTransform3");
    tfServer->remove("gripperTransform1", "gripperTransform2");
    tfServer->remove("gripperTransform0", "gripperTransform1");
    tfServer->remove("world", "gripperTransform0");
}

void OpenBagMotionPlanner::WaitForExecution()
{    
    ROS_INFO_STREAM("START WAITING");
    robot_state::RobotState prev_state(*groupPtr->getCurrentState());
    robot_state::RobotState curr_state(*groupPtr->getCurrentState());
    bool moving = true;
    ros::Duration(2.0).sleep();

    while (moving)
    {
	ros::Duration(1.0).sleep();
	curr_state = *groupPtr->getCurrentState();
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