#include "operation_panel/InteractiveMarkers/InteractiveMarkerPoint.h"

InteractiveMarkerPoint::InteractiveMarkerPoint()
{
    tfReaded = false;
}
   
void InteractiveMarkerPoint::MakeInteractiveMarker(std::string intMarkerName, tf::Quaternion qx_control, tf::Quaternion qy_control, tf::Quaternion qz_control)
{	
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "world";
    int_marker.scale = 0.1;
    int_marker.name = intMarkerName;
    
    geometry_msgs::Pose pose;    
    tfSrv->get("world", intMarkerName + "_control", pose);    
    int_marker.pose = pose;

    InteractiveMarkerPoint::MakeControl(int_marker);
    
    int_marker.controls[0].interaction_mode = 7;

    visualization_msgs::InteractiveMarkerControl control;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    
    tf::Transform tr;
    tfSrv->get("world", int_marker.name + "_control", tr);
    
    qx_control = tr.getRotation() * qx_control;
    qy_control = tr.getRotation() * qy_control;
    qz_control = tr.getRotation() * qz_control;

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
    intMarkerSrv->setCallback(int_marker.name, _processFeedBackTemp(boost::bind(&InteractiveMarkerPoint::ProcessFeedback, this, _1)));
    intMarkerSrv->applyChanges();
}

void InteractiveMarkerPoint::ProcessFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    if (!tfReaded)
    {
	WaitTF(feedback->marker_name);
    }
    
    if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    {
	tf::Transform tr;
	tr.setOrigin(tf::Vector3(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z));
	tr.setRotation(tf::Quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w));    		
	tfSrv->set("world", feedback->marker_name + "_control", tr);					
    }
    intMarkerSrv->applyChanges();	
}
