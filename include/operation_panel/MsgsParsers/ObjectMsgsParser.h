#ifndef OBJECTMSGSPARSER_H
#define OBJECTMSGSPARSER_H

#include <operation_panel/MsgsParsers/IMsgsParser.h>

class ObjectMsgsParser : public IMsgsParser
{
    
public:
    ObjectMsgsParser(ros::NodeHandle);
    
    bool GetObject(visualization_msgs::Marker&);
    
private:
    
    std::string object_name;
    
    visualization_msgs::Marker object;
    
    ros::NodeHandle nh;

    ros::Subscriber selected_object_id_sub;
    
    object_recognition_ros::ObjectInfoDiskCache info_cache_;
    
    void IDCallback(const std_msgs::String::ConstPtr& msg);
    
    void CreateObject();

};

#endif // OBJECTMSGSPARSER_H
