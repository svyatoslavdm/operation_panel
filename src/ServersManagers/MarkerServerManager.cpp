#include "operation_panel/ServersManagers/MarkerServerManager.h"

MarkerServerManager::MarkerServerManager(boost::shared_ptr<MarkerServer> markerServer_, Scene* scene)
{
     markerServer = markerServer_;
     gripperMarkers = scene->GetGripperMarkers();
}

void MarkerServerManager::CreateArrow(std::string name)
{
    visualization_msgs::Marker marker; 
    marker.header.frame_id = name + "_marker";
    marker.ns = name;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.16;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;    
    markerServer->set(name, marker);
}

void MarkerServerManager::CreatePoint(std::string name)
{
    visualization_msgs::Marker marker; 
    marker.header.frame_id = name + "_marker";
    marker.ns = name;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;    
    markerServer->set(name, marker);
}

void MarkerServerManager::CreateObject(std::string name)
{
}

void MarkerServerManager::CreateGraspChooser(std::string name)
{
    std::vector<visualization_msgs::Marker> gripperMarkers_ = gripperMarkers;
    
    for(int i = 0; i < gripperMarkers_.size(); i++)
    {
	gripperMarkers_[i].header.frame_id = "grasp_marker";
	markerServer->set(gripperMarkers_[i].ns, gripperMarkers_[i]);
    }
}

void MarkerServerManager::CreateUnscrewChooser(std::string name)
{
    std::vector<visualization_msgs::Marker> unscrewMarkers_ = gripperMarkers;
    
    for(int i = 0; i < unscrewMarkers_.size(); i++)
    {
	unscrewMarkers_[i].header.frame_id = "unscrew_marker";
	markerServer->set(unscrewMarkers_[i].ns, unscrewMarkers_[i]);
    }
    
    visualization_msgs::Marker marker; 
    marker.header.frame_id = "unscrew_arrow";
    marker.ns = name;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.16;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;    
    markerServer->set("unscrew_arrow", marker);
}

void MarkerServerManager::Remove(std::string name)
{
    markerServer->remove(name);
}

void MarkerServerManager::RemoveGraspChooser(std::string name)
{
    for(int i = 0; i < gripperMarkers.size(); i++)
    {
	markerServer->remove(gripperMarkers[i].ns);
    }
}

void MarkerServerManager::RemoveUnscrewChooser(std::string name)
{
    for(int i = 0; i < gripperMarkers.size(); i++)
    {
	markerServer->remove(gripperMarkers[i].ns);
    }
    
    markerServer->remove("unscrew_arrow");
}
