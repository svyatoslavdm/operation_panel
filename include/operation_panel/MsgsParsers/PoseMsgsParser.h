#ifndef POSEMSGSPARSER_H
#define POSEMSGSPARSER_H

#include <operation_panel/MsgsParsers/IMsgsParser.h>
#include <geometry_msgs/PoseStamped.h>

class PoseMsgsParser : public IMsgsParser
{
    
public:
    PoseMsgsParser(ros::NodeHandle);
    
    bool GetObject(visualization_msgs::Marker&);
    
private:
    
    std::string object_name;
    bool pose_parsed;
    
    visualization_msgs::Marker object;
    
    ros::NodeHandle nh;
    
    ros::Subscriber selected_object_id_sub;
    ros::Subscriber pose_sub;
    
    object_recognition_ros::ObjectInfoDiskCache info_cache_;
    
    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void IDCallback(const std_msgs::String::ConstPtr& msg);
    
    void CreateObject();

};

#endif // ORKMSGSPARSER_H
