

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>

#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/ply_io.h>

#include <tf/transform_listener.h>

//#include <pcl/features/fpfh.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/registration/ia_ransac.h>
// //#include <pcl/registration/icp.h>

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> ColorHandlerTXYZRGB;

struct SensorCoords
{
    float x;
    float y;
    float z;
    float a;
    float b;
    float g;
};

class PointCloudProcessor
{
public:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudOutput;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudFinal;
    pcl::PolygonMesh mesh;

    PointCloudProcessor();  
    void Clear();
    void SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float angle, int num);
    void CreateFinalCloud();
    
private:
    float _angle;
    int _num;
    bool first_time;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudInput;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudInputRotated;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudCutted;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudWOPlane;  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudPrev;  
    SensorCoords mSensorCoords;
    
    void ProcessCloud();
    void CutCloud();
    void DeletePlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudOut);
    void VoxelGridFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudOut);
    void RemoveOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudOut);
    void SmoothCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudOut);
    void AlignClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _Source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _Target, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudOut);
    void FirstTransformation();
    void ithTransformation(float _angle, int _num);
    void TransformCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudIn,pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudOut, Eigen::Matrix4f transformationMatrix,Eigen::Matrix4f rotationMatrix);//     
    void ReconstructSurface();
};
