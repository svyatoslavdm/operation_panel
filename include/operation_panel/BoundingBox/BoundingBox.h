#ifndef BOUNDING_BOX_H_
#define BOUNDING_BOX_H_

#include <ros/ros.h>
#include <geometric_shapes/shape_operations.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/ros/conversions.h>

class BoundingBox 
{
public:
    BoundingBox();    
    int SetObject(visualization_msgs::Marker);    
    visualization_msgs::Marker Get_bb_marker();    
    tf::Vector3 Get_L_bb_ob_ob();
    tf::Quaternion Get_Q_bb_ob();
    tf::Vector3 GetDimensions();
    
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; 
    shape_msgs::Mesh object_mesh;
    visualization_msgs::Marker bb_marker;    
    Eigen::Vector3f l_bb_obj_obj;
    Eigen::Matrix3f tau_bb_obj; 
    Eigen::Quaternionf q_bb_obj;
    Eigen::Vector3f dimensions;       
    
    void GetBBPosition();
    void GetBBOrientation();
    void CorrBBOrientation();
    void CorrBBPosition();
    void GetBBDimensions(float, float, float);
    void CreateBBMarker();
    void CreateBoundingBox();
};

#endif //BOUNDING_BOX_H_
