#ifndef SCENE_H_
#define SCENE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/ros/conversions.h>
#include <urdf/model.h>
#include <visualization_msgs/Marker.h>

class Scene
{
public:    
    Scene(ros::NodeHandle& nh);
    std::vector< std::vector< tf::Vector3 > > GetOperationTablePolygons();
    std::vector< tf::Vector3 > GetOperationTableNormals();
    std::vector< std::vector< tf::Vector3 > > GetRotaryTablePolygons();
    std::vector< tf::Vector3 > GetRotaryTableNormals();
    std::vector< std::vector< tf::Vector3 > > GetViceGripPolygons();
    std::vector< tf::Vector3 > GetViceGripNormals();
    tf::Vector3 GetViceGripCenter();
    tf::Quaternion GetViceGripRotation();
    std::vector<visualization_msgs::Marker> GetGripperMarkers();
    
private:
    urdf::Model robotModel;
    std::vector< std::vector< tf::Vector3 > > operationTablePolygons;    
    std::vector< tf::Vector3 > operationTableNormals;
    std::vector< std::vector< tf::Vector3 > > rotaryTablePolygons;    
    std::vector< tf::Vector3 > rotaryTableNormals;
    std::vector< std::vector< tf::Vector3 > > viceGripPolygons;
    std::vector< tf::Vector3 > viceGripNormals;
    tf::Vector3 viceGripCenter;
    std::vector<visualization_msgs::Marker> gripperMarkers;
    
    bool GetRobotModel(std::string);    
    pcl::PolygonMesh GetTableMesh(std::string);  
    tf::Transform GetTableTransform(std::string); 
    void GetTableProperty(std::string, std::vector< std::vector< tf::Vector3 > >&, std::vector< tf::Vector3 >&); 
    void GetViceGripProperty();
    tf::Transform GetViceGripTransform();
    void GetViceGripArea(tf::Transform);
    void CreateGripperMarkers();
};

#endif //SCENE_H_
