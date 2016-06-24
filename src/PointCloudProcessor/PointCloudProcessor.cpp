#include "operation_panel/PointCloudProcessor/PointCloudProcessor.h"
//------------------------------------------------------------------------------------------------------
PointCloudProcessor::PointCloudProcessor()
{
    CloudInput = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    CloudInputRotated = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    CloudCutted = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    CloudWOPlane = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    CloudOutput = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    CloudFinal = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    CloudPrev = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    //get sensor coord
}
void PointCloudProcessor::Clear()
{
    CloudOutput->clear();
    CloudFinal->clear();
}
void PointCloudProcessor::CreateFinalCloud()
{
    
//     RemoveOutliers(CloudOutput, CloudOutput);
    SmoothCloud(CloudOutput, CloudFinal);
    *CloudOutput = *CloudFinal;
    DeletePlanes(CloudOutput, CloudFinal);
    pcl::visualization::PCLVisualizer visualizer("Visualiser");
    visualizer.addPointCloud(CloudFinal,
			    ColorHandlerTXYZRGB(CloudFinal, 0.0, 255.0, 0.0), "filtered");

    visualizer.addCoordinateSystem(0.2);
    visualizer.setBackgroundColor(0.0,0.0,0.0);
    while(!visualizer.wasStopped())
    {
	visualizer.spinOnce();
    }   
    ReconstructSurface();
}
void PointCloudProcessor::SetInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float angle, int num)
{
    *CloudInput = *cloud;
    _angle = angle;
    _num = num;
    ProcessCloud();
}

void PointCloudProcessor::ProcessCloud()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudTemp(new pcl::PointCloud<pcl::PointXYZRGB>);
    FirstTransformation();  
    CutCloud();    
    *CloudTemp = *CloudCutted;
    RemoveOutliers(CloudTemp, CloudCutted); 
    *CloudTemp = *CloudCutted;
    VoxelGridFilter(CloudTemp, CloudCutted);
    *CloudTemp = *CloudCutted;
    SmoothCloud(CloudTemp, CloudCutted);
//     pcl::visualization::PCLVisualizer visualizer("Visualiser");
//     visualizer.addPointCloud(CloudTemp,
// 			    ColorHandlerTXYZRGB(CloudTemp, 255.0, 0.0, 0.0), "filtered");
//     visualizer.addPointCloud(CloudCutted,
// 			    ColorHandlerTXYZRGB(CloudCutted, 0.0, 255.0, 0.0), "input");
// 
//     visualizer.addCoordinateSystem(0.2);
//     visualizer.setBackgroundColor(0.0,0.0,0.0);
//     while(!visualizer.wasStopped())
//     {
// 	visualizer.spinOnce();
//     }   
    
    
    if (_num > 0)
    {
	*CloudTemp = *CloudCutted;
	AlignClouds(CloudTemp, CloudPrev, CloudCutted);
	
	pcl::visualization::PCLVisualizer icp_visualizer("Visualiser");
	icp_visualizer.addPointCloud(CloudCutted,
				ColorHandlerTXYZRGB(CloudCutted, 255.0, 0.0, 0.0), "input");
	icp_visualizer.addPointCloud(CloudTemp,
				ColorHandlerTXYZRGB(CloudTemp, 0.0, 255.0, 0.0), "filtered");
	icp_visualizer.addPointCloud(CloudOutput,
				ColorHandlerTXYZRGB(CloudOutput, 0.0, 0.0, 255.0), "CloudOutput");
	icp_visualizer.addCoordinateSystem(0.2);
	icp_visualizer.setBackgroundColor(0.0,0.0,0.0);
	while(!icp_visualizer.wasStopped())
	{
	    icp_visualizer.spinOnce();
	}  
    }

    *CloudOutput += *CloudCutted;   
    
    *CloudPrev = *CloudCutted;
   
//     DeletePlanes();
//     ithTransformation(_angle, _num);   
//     *CloudOutput += *CloudWOPlane;
    //fCloudsAggregator.Clouds.push_back(CloudOutput);
}
void PointCloudProcessor::CutCloud()
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;

    pass.setInputCloud (CloudInputRotated);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 0.4);
    pass.filter (*CloudCutted);

    pass.setInputCloud (CloudCutted);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-0.2, 0.2);
    pass.filter (*CloudCutted);

    pass.setInputCloud (CloudCutted);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-0.2, 0.2);
    pass.filter (*CloudCutted);


}
void PointCloudProcessor::AlignClouds(pcl::PointCloud< pcl::PointXYZRGB >::Ptr _Source, pcl::PointCloud< pcl::PointXYZRGB >::Ptr _Target, pcl::PointCloud< pcl::PointXYZRGB >::Ptr _CloudOut)
{
    ROS_WARN_STREAM("AlignClouds started");
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> ICPAlighner;
    ICPAlighner.setInputSource(_Source); //model
    ICPAlighner.setInputTarget (_Target); //scene
    ICPAlighner.setMaximumIterations (500);
    ICPAlighner.setMaxCorrespondenceDistance (0.01);
    ICPAlighner.setTransformationEpsilon (1e-6);
    ICPAlighner.align (*_CloudOut);
    ROS_WARN_STREAM("AlignClouds ended");
}
void PointCloudProcessor::VoxelGridFilter(pcl::PointCloud< pcl::PointXYZRGB >::Ptr _CloudIn, pcl::PointCloud< pcl::PointXYZRGB >::Ptr _CloudOut)
{
    float VOXEL_SIZE = 0.005;
    pcl::VoxelGrid<pcl::PointXYZRGB> VoxelGridFilterICP;
    VoxelGridFilterICP.setInputCloud (_CloudIn);
    VoxelGridFilterICP.setLeafSize (VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    VoxelGridFilterICP.filter (*_CloudOut);
}

void PointCloudProcessor::DeletePlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudOut)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.008);

    seg.setInputCloud (_CloudIn);

    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
    }
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (_CloudIn);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*_CloudOut);
}
void PointCloudProcessor::RemoveOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudOut)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (_CloudIn);
    sor.setMeanK (15);
    sor.setStddevMulThresh (1.0);
    sor.filter (*_CloudOut);

    ROS_INFO_STREAM("Outliers removed, putliers num: " << sor.getRemovedIndices()->size() << " of: " << _CloudIn->points.size());
}
void PointCloudProcessor::SmoothCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudOut)
{
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr Tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    mls.setInputCloud (_CloudIn);
    mls.setComputeNormals (false);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (Tree);
    mls.setSearchRadius (0.01);
    mls.process(*_CloudOut);
//    ROS_INFO("Planes smoothed");
}
void PointCloudProcessor::ReconstructSurface()
{
    pcl::PointCloud<pcl::PointNormal>::Ptr CloudNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> ne;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal> ());
    ne.setInputCloud (CloudFinal);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.01);
    ne.compute (*CloudNormals);

    for (size_t i = 0; i < CloudNormals->points.size(); i++)
    {
        CloudNormals->points[i].x = CloudFinal->points[i].x;
        CloudNormals->points[i].y = CloudFinal->points[i].y;
        CloudNormals->points[i].z = CloudFinal->points[i].z;
    }

    pcl::Poisson<pcl::PointNormal>poi;
    poi.setInputCloud(CloudNormals);
    poi.setSearchMethod(tree2);
    poi.setConfidence(false);
    poi.setManifold(false);
    poi.setOutputPolygons(false);
    poi.setDepth(7);
    poi.setSolverDivide(7);
    poi.setIsoDivide(7);
    poi.setSamplesPerNode(1);

    poi.performReconstruction(mesh);

}
void PointCloudProcessor::FirstTransformation()
{
    Eigen::Matrix4f translationMatrix, rotationMatrix;
    Eigen::Quaternionf quat;
    
    tf::TransformListener tf_listener;
    tf::StampedTransform stamped_transform;
    tf::Transform transform;
    
    std::string parent = "camera_rgb_optical_frame";
    std::string child = "rotary_table";
    
    try
    {
	tf_listener.waitForTransform(parent, child, ros::Time(0), ros::Duration(5));
	tf_listener.lookupTransform(parent, child, ros::Time(0), stamped_transform);
	
	transform.setOrigin(stamped_transform.getOrigin());
	transform.setRotation(stamped_transform.getRotation());
	quat.x() = transform.getRotation().x();
	quat.y() = transform.getRotation().y();
	quat.z() = transform.getRotation().z();
	quat.w() = transform.getRotation().w();
	
// 	return true;
    }
    catch (tf::TransformException ex)
    {
	ROS_INFO_STREAM("Can't get transform " << parent << "->" << child << " cause: " << ex.what());
// 	return false;
    }
    
    
    float x = -transform.getOrigin().x(), // magic numbers
          y = -transform.getOrigin().y(),
          z = -transform.getOrigin().z();
	  
	  
    ROS_INFO_STREAM("transform:" << " x: " << transform.getOrigin().x() << " y: " << transform.getOrigin().y() << " z: " << transform.getOrigin().z() );

    translationMatrix << 1, 0, 0, x,
                            0, 1, 0, y,
                            0, 0, 1, z,
                            0, 0, 0, 1;
    Eigen::Matrix3f mat = quat.toRotationMatrix().transpose();
    
    rotationMatrix <<       mat(0,0), mat(0,1), mat(0,2), 0,
                            mat(1,0), mat(1,1), mat(1,2), 0,
                            mat(2,0), mat(2,1), mat(2,2), 0,
                            0,           0,           0, 1;

    TransformCloud(CloudInput, CloudInputRotated, translationMatrix, rotationMatrix);

}
void PointCloudProcessor::ithTransformation(float _angle, int _num)
{

    Eigen::Matrix4f translationMatrix, rotationMatrix;
    float x = 0.0, // magic numbers
          y = 0.0,
          z = 0.0;
    float alpha = _angle * _num * 3.1415926 / 180;

    translationMatrix << 1, 0, 0, x,
                            0, 1, 0, y,
                            0, 0, 1, z,
                            0, 0, 0, 1;
    rotationMatrix << cos (alpha),-sin (alpha), 0, 0,
                      sin (alpha), cos (alpha), 0, 0,
                                0,           0, 1, 0,
                                0,           0, 0, 1;
    TransformCloud(CloudWOPlane, CloudWOPlane, translationMatrix, rotationMatrix);

}
void PointCloudProcessor::TransformCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudIn,pcl::PointCloud<pcl::PointXYZRGB>::Ptr _CloudOut,
                                      Eigen::Matrix4f translationMatrix,Eigen::Matrix4f rotationMatrix)
{

    pcl::transformPointCloud(*_CloudIn,*_CloudOut, translationMatrix);
    pcl::transformPointCloud(*_CloudOut,*_CloudOut, rotationMatrix);
}