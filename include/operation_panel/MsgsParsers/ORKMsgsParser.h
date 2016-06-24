#ifndef ORKMSGSPARSER_H
#define ORKMSGSPARSER_H

#include <operation_panel/MsgsParsers/IMsgsParser.h>


class ORKMsgsParser : public IMsgsParser
{
    
public:
    ORKMsgsParser(ros::NodeHandle);
    
    bool GetObject(visualization_msgs::Marker&);
    
private:
    
    std::string object_name;
    
    visualization_msgs::Marker object;
    
    ros::NodeHandle nh;

    ros::Subscriber rec_obj_array_sub;
    
    object_recognition_ros::ObjectInfoDiskCache info_cache_;
    
    void ORKCallback(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg);
    
    void CreateObject();

};

#endif // ORKMSGSPARSER_H
