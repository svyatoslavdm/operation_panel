#include "operation_panel/InteractiveMarkers/InteractiveMarkerGraspChooser.h"

InteractiveMarkerGraspChooser::InteractiveMarkerGraspChooser()
{
    tfReaded = false;
}
	
void InteractiveMarkerGraspChooser::MakeInteractiveMarker(std::string intMarkerName, tf::Quaternion qx_control, tf::Quaternion qy_control, tf::Quaternion qz_control)
{	
    MakeRotationMarker(intMarkerName, qx_control, qy_control, qz_control);
    MakePositionMarker(intMarkerName, qx_control, qy_control, qz_control);
    
    intMarkerSrv->applyChanges();
}

void InteractiveMarkerGraspChooser::MakeRotationMarker(std::string intMarkerName, tf::Quaternion qx_control, tf::Quaternion qy_control, tf::Quaternion qz_control)
{
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = intMarkerName + "_control";
    int_marker.scale = 0.4;
    int_marker.name = intMarkerName + "_gripper_rotation";
    
    int_marker.pose.position.x = 0.0;
    int_marker.pose.position.y = 0.0;
    int_marker.pose.position.z = 0.0;
    int_marker.pose.orientation.x = 0.0;
    int_marker.pose.orientation.y = 0.0;
    int_marker.pose.orientation.z = 0.0;
    int_marker.pose.orientation.w = 1.0;
    
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

    control.orientation.w = qx_control.getW();
    control.orientation.x = qx_control.getX();
    control.orientation.y = qx_control.getY();
    control.orientation.z = qx_control.getZ();
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = qz_control.getW();
    control.orientation.x = qz_control.getX();
    control.orientation.y = qz_control.getY();
    control.orientation.z = qz_control.getZ();
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = qy_control.getW();
    control.orientation.x = qy_control.getX();
    control.orientation.y = qy_control.getY();
    control.orientation.z = qy_control.getZ();
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    intMarkerSrv->insert(int_marker);      
    intMarkerSrv->setCallback(int_marker.name, _processFeedBackTemp(boost::bind(&InteractiveMarkerGraspChooser::ProcessRotateFeedback, this, _1)));
    intMarkerSrv->applyChanges();
}
    
void InteractiveMarkerGraspChooser::MakePositionMarker(std::string intMarkerName, tf::Quaternion qx_control, tf::Quaternion qy_control, tf::Quaternion qz_control)
{
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = intMarkerName + "_gripper_position";
    int_marker.scale = 0.15;
    int_marker.name = intMarkerName + "_gripper_position";
    
    int_marker.pose.position.x = 0.0;
    int_marker.pose.position.y = 0.0;
    int_marker.pose.position.z = 0.0;
    int_marker.pose.orientation.x = 0.0;
    int_marker.pose.orientation.y = 0.0;
    int_marker.pose.orientation.z = 0.0;
    int_marker.pose.orientation.w = 1.0;
    
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

    control.orientation.w = qx_control.getW();
    control.orientation.x = qx_control.getX();
    control.orientation.y = qx_control.getY();
    control.orientation.z = qx_control.getZ();
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = qz_control.getW();
    control.orientation.x = qz_control.getX();
    control.orientation.y = qz_control.getY();
    control.orientation.z = qz_control.getZ();
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = qy_control.getW();
    control.orientation.x = qy_control.getX();
    control.orientation.y = qy_control.getY();
    control.orientation.z = qy_control.getZ();
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    intMarkerSrv->insert(int_marker);      
    intMarkerSrv->setCallback(int_marker.name, _processFeedBackTemp(boost::bind(&InteractiveMarkerGraspChooser::ProcessPositionFeedback, this, _1)));
    intMarkerSrv->applyChanges();
}
    
void InteractiveMarkerGraspChooser::ProcessRotateFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{    
    if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    {
	std::string objectName = feedback->marker_name.substr(0, feedback->marker_name.length() - 17);		
	tf::Transform tr;
	tr.setOrigin(tf::Vector3(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z));
	tr.setRotation(tf::Quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w));    		
	tfSrv->set(objectName + "_control", objectName + "_gripper_rotation", tr);		
    }
    intMarkerSrv->applyChanges();
}
    
void InteractiveMarkerGraspChooser::ProcessPositionFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{    
    if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    {
	std::string objectName = feedback->marker_name.substr(0, feedback->marker_name.length() - 17);
	tf::Transform tr;
	tr.setOrigin(tf::Vector3(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z));
	tr.setRotation(tf::Quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w));    		
	tfSrv->set(objectName + "_gripper_position", "grasp_marker", tr);
    }
    intMarkerSrv->applyChanges();
}