#ifndef MARKERSERVER_H
#define MARKERSERVER_H

#include <operation_panel/Servers/IServer.h>

#include <visualization_msgs/Marker.h>
#include <string.h>
#include <map>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>

#include <Eigen/Eigen>

class MarkerServer : public IServer
{

public:
    
    MarkerServer(ros::NodeHandle, std::string);
    
    void set(std::string ,visualization_msgs::Marker);

    bool get(std::string, visualization_msgs::Marker&);

    void remove(std::string);

    void start();

    void stop();
    
    std::map<std::string, visualization_msgs::Marker> getMap();
     
private:
    ros::NodeHandle nh;
    ros::Publisher publisher;
    ros::Timer frame_timer;
    
    std::string topic_name;
    std::map<std::string, visualization_msgs::Marker> markers;
    
    void Publish(const ros::TimerEvent&);
    
    void delete_marker(std::string);

};

#endif // MARKERSERVER_H
