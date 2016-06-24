#ifndef IMSGSPARSER_H
#define IMSGSPARSER_H

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_ros/object_info_cache.h>
#include <object_recognition_msgs/ObjectType.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <shape_msgs/Mesh.h>
#include <visualization_msgs/Marker.h>

class IMsgsParser
{

public:
    
    IMsgsParser(ros::NodeHandle);

    virtual bool GetObject(visualization_msgs::Marker&) = 0;

};

#endif // IMSGSPARSER_H
