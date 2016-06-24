#include <operation_panel/Servers/MarkerServer.h>

MarkerServer::MarkerServer(ros::NodeHandle _nh, std::string _topic_name)
{
    
   nh = _nh;
   topic_name = _topic_name;
   frame_timer = nh.createTimer(ros::Duration(0.02), &MarkerServer::Publish, this);
   stop();
   publisher = nh.advertise<visualization_msgs::Marker>(topic_name, 0);
}
void MarkerServer::Publish(const ros::TimerEvent&)
{
    for (std::map<std::string, visualization_msgs::Marker>::iterator marker_it = markers.begin(); marker_it != markers.end(); ++marker_it)
    {
// 	ROS_INFO_STREAM("Publishing " << marker_it->first);
	publisher.publish(marker_it->second);
    }
//     ROS_INFO_STREAM("Publishing " << marker_it->first);
}
void MarkerServer::set(std::string name,visualization_msgs::Marker marker)
{
    markers[name] = marker;
    markers[name].ns = name;
    ros::TimerEvent e;
    Publish(e);
}
bool MarkerServer::get(std::string name, visualization_msgs::Marker& marker)
{
    if (markers.find(name) != markers.end())
    { 
	marker = markers[name];
	return true;
    }
    else
    {
	return false;
    }
}
void MarkerServer::remove(std::string name)
{
    if (markers.find(name) != markers.end())
    { 
	delete_marker(name);
	markers.erase(name);
	
	ros::TimerEvent e;
	Publish(e);
    }
}
void MarkerServer::delete_marker(std::string name)
{
    markers[name].action = visualization_msgs::Marker::DELETE;
    publisher.publish(markers[name]);
}
void MarkerServer::start()
{
    frame_timer.start();
}
void MarkerServer::stop()
{
    frame_timer.stop();
}
std::map< std::string, visualization_msgs::Marker > MarkerServer::getMap()
{
    return markers;
}
