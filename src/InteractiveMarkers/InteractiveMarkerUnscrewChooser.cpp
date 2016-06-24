#include "operation_panel/InteractiveMarkers/InteractiveMarkerUnscrewChooser.h"

InteractiveMarkerUnscrewChooser::InteractiveMarkerUnscrewChooser()
{
    tfReaded = false;
}
	
void InteractiveMarkerUnscrewChooser::MakeInteractiveMarker(std::string intMarkerName, tf::Quaternion qx_control, tf::Quaternion qy_control, tf::Quaternion qz_control)
{	
    MakeRotationMarker(intMarkerName, qx_control, qy_control, qz_control);
    MakePositionMarker(intMarkerName, qx_control, qy_control, qz_control);
    
    intMarkerSrv->applyChanges();
}

void InteractiveMarkerUnscrewChooser::MakeRotationMarker(std::string intMarkerName, tf::Quaternion qx_control, tf::Quaternion qy_control, tf::Quaternion qz_control)
{
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "unscrew_control";
    int_marker.scale = 0.3;
    int_marker.name = "unscrew_rotation";
    
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
    
    control.orientation.w = qz_control.getW();
    control.orientation.x = qz_control.getX();
    control.orientation.y = qz_control.getY();
    control.orientation.z = qz_control.getZ();
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    intMarkerSrv->insert(int_marker);      
    intMarkerSrv->setCallback(int_marker.name, _processFeedBackTemp(boost::bind(&InteractiveMarkerUnscrewChooser::ProcessRotateFeedback, this, _1)));
    intMarkerSrv->applyChanges();
}
    
void InteractiveMarkerUnscrewChooser::MakePositionMarker(std::string intMarkerName, tf::Quaternion qx_control, tf::Quaternion qy_control, tf::Quaternion qz_control)
{
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "unscrew_position";
    int_marker.scale = 0.15;
    int_marker.name = "unscrew_position";
    
    int_marker.pose.position.x = 0.0;
    int_marker.pose.position.y = 0.0;
    int_marker.pose.position.z = 0.0;
    int_marker.pose.orientation.x = 0.0;
    int_marker.pose.orientation.y = 0.0;
    int_marker.pose.orientation.z = 0.0;
    int_marker.pose.orientation.w = 1.0;
    
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

    control.orientation.w = qz_control.getW();
    control.orientation.x = qz_control.getX();
    control.orientation.y = qz_control.getY();
    control.orientation.z = qz_control.getZ();
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    intMarkerSrv->insert(int_marker);      
    intMarkerSrv->setCallback(int_marker.name, _processFeedBackTemp(boost::bind(&InteractiveMarkerUnscrewChooser::ProcessPositionFeedback, this, _1)));
    intMarkerSrv->applyChanges();
}
    
void InteractiveMarkerUnscrewChooser::ProcessRotateFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{    
    if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    {	
	tf::Transform tr;
	tr.setOrigin(tf::Vector3(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z));
	tr.setRotation(tf::Quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w));    		
	tfSrv->set("unscrew_control", "unscrew_rotation", tr);
	tr.setRotation(tf::Quaternion(0.0, -0.707, 0.0, 0.707));
	tfSrv->set("unscrew_control", "unscrew_arrow", tr);
    }
    intMarkerSrv->applyChanges();
}
    
void InteractiveMarkerUnscrewChooser::ProcessPositionFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{    
    if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    {
	tf::Transform tr;
	tr.setOrigin(tf::Vector3(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z));
	tr.setRotation(tf::Quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w));    		
	tfSrv->set("unscrew_position", "unscrew_marker", tr);
    }
    intMarkerSrv->applyChanges();
}