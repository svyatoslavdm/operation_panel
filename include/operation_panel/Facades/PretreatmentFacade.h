#ifndef PRETREATMENT_FACADE_H_
#define PRETREATMENT_FACADE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/parse.h>

#include <tf/tf.h>

#include <moveit/background_processing/background_processing.h>

#include "operation_panel/PointCloudProcessor/PointCloudProcessor.h"
#include "operation_panel/MotionPlanners/ObjectScanerMotionPlanner.h"
#include "operation_panel/BoundingBox/BoundingBox.h"

namespace rviz
{  
    
class PretreatmentGUI;
    
class PretreatmentFacade
{
public:
    PretreatmentFacade(ros::NodeHandle&);
    void SetGui(PretreatmentGUI*);    
    std::vector<std::string> EnterFileNameButtonClicked(std::string);
    void ViewpointButtonClicked();
    void SetBiasButtonClicked();
    void ScanButtonClicked();
    
private:   
    ros::Publisher cloudPub;
    ros::Subscriber cloudSub;
    ros::Subscriber ftsSub;
    
    PretreatmentGUI* pretreatmentGUI;
    PointCloudProcessor* pointCloudProcessor;
    ObjectScanerMotionPlanner* objectScanerMotionPlanner;

    std::string name;
    std::string cloud_file_name;
    std::string mesh_ply_file_name;
    std::string mesh_stl_file_name;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input;    
    float angle;
    int cloud_count;
    bool cloud_received;
    std::vector<float> masses;
    std::vector<float> forces;
    std::vector<float> forces_raw;
    std::vector<float> bias;
    float force_input;
    bool force_received;
    float massBias;
    
    bool ParsePath(std::string); 
    void CloudCallback(const sensor_msgs::PointCloud2ConstPtr&);
    void FTSCallback(const geometry_msgs::WrenchStamped::ConstPtr&);
    void PublishCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    void CalcMass();
    void CalcDimensions();
    
    void GoToViewpoint_f();
    void GoToStartPosition_f();
    void GoToHomePosition_f();
    void RotateTable();
    void RotateTable_f();
    
    void AddBackgroundJob(const boost::function<void()> &, const std::string &);
    moveit::tools::BackgroundProcessing background_process_;
};

}//end namespace rviz

#endif // PRETREATMENT_FACADE_H_