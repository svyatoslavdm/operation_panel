#include "operation_panel/InteractiveMarkers/InteractiveMarkerBase.h"

void InteractiveMarkerBase::SetTFServer(boost::shared_ptr<TFServer> tfSrv_)
{
    tfSrv = tfSrv_;
}

void InteractiveMarkerBase::SetInteractiveMarkerServer(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> intMarkerSrv_)
{
    intMarkerSrv = intMarkerSrv_;
}

void InteractiveMarkerBase::Add(std::string intMarkerName)
{
    tf::Quaternion qx_control = tf::Quaternion (1.0, 0.0, 0.0, 1.0);
    tf::Quaternion qy_control = tf::Quaternion (0.0, 0.0, 1.0, 1.0); 
    tf::Quaternion qz_control = tf::Quaternion (0.0, 1.0, 0.0, 1.0);
    
    MakeInteractiveMarker(intMarkerName, qx_control, qy_control, qz_control);
    intMarkerSrv->applyChanges();
}

void InteractiveMarkerBase::Remove(std::string intMarkerName)
{
    intMarkerSrv->erase(intMarkerName);
    intMarkerSrv->applyChanges();
} 

void InteractiveMarkerBase::Delete()
{
    intMarkerSrv->clear();
    intMarkerSrv->applyChanges();
}   

visualization_msgs::Marker InteractiveMarkerBase::MakeFakeMarker(std::string intMarkerName)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = intMarkerName + "_control";
    marker.ns = intMarkerName + "_fakeMarker";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.0;
    return marker;
}

visualization_msgs::InteractiveMarkerControl& InteractiveMarkerBase::MakeControl(visualization_msgs::InteractiveMarker &msg)
{
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( MakeFakeMarker(msg.name) );
    msg.controls.push_back( control );
    return msg.controls.back();
}

void InteractiveMarkerBase::WaitTF(std::string intMarkerName)
{
    geometry_msgs::Pose pose;    
    visualization_msgs::InteractiveMarker int_marker;
    
    if (tfSrv->get("world", intMarkerName + "_control", pose))
    {
	tfReaded = true;
	intMarkerSrv->get(intMarkerName, int_marker);	
	int_marker.pose = pose;	
	intMarkerSrv->insert(int_marker);
    }
}

void InteractiveMarkerBase::MakeInteractiveMarker(std::string intMarkerName, tf::Quaternion qx_control, tf::Quaternion qy_control, tf::Quaternion qz_control)
{}

