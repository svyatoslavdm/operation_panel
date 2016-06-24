#ifndef TFSERVER_H
#define TFSERVER_H

#include <operation_panel/Servers/IServer.h>

#include <string.h>
#include <map>
#include <vector>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <geometry_msgs/Pose.h>

#include <Eigen/Eigen>

class  TFServer : public IServer
{

private:

    ros::NodeHandle nh;
    ros::Timer frame_timer;
        
    tf::TransformBroadcaster tf_publisher;
    tf::TransformListener tf_listener;
    
    std::map<std::pair<std::string, std::string>, tf::Transform> tfs;
    
    bool pub_lock;

    void Publish(const ros::TimerEvent&);
    
    void PubThread(const ros::TimerEvent&);

public:

    TFServer(ros::NodeHandle _nh);
    ~TFServer();
    
    void set(){};
    
    void set(std::string, std::string, tf::Transform);
    
    void set(std::string, std::string, geometry_msgs::Pose);
    
    void set(std::string, std::string, Eigen::Vector3f, Eigen::Quaternionf);
    
    bool get(std::string, std::string, tf::Transform&);
    
    bool get(std::string, std::string, geometry_msgs::Pose&);
    
    bool get(std::string, std::string, Eigen::Vector3f&, Eigen::Quaternionf&);
    
    bool getFromMap(std::string, std::string, tf::Transform&);
    
    bool getFromMap(std::string, std::string, geometry_msgs::Pose&);
    
    bool getFromMap(std::string, std::string, Eigen::Vector3f&, Eigen::Quaternionf&);
    
    std::map<std::pair<std::string, std::string>, tf::Transform> getMap();
    
    void remove(std::string, std::string);
    
    void start();
    
    void stop();
    
    void PublishTFs();

};

#endif //  TFSERVER_H
