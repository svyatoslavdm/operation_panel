#include <operation_panel/Facades/PretreatmentFacade.h>
#include <operation_panel/GUI/PretreatmentGUI.h>

namespace rviz
{
    
PretreatmentFacade::PretreatmentFacade(ros::NodeHandle& nh)
{
    cloud_received = false;
    force_received = false;
    
    bias.resize(3, 0);
    forces.resize(3, 0);
    forces_raw.resize(3, 0);
    
    cloud_input = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloudPub = nh.advertise<sensor_msgs::PointCloud2> ("/scanned_cloud", 1);    
    cloudSub = nh.subscribe ("/camera/depth_registered/points", 1, &PretreatmentFacade::CloudCallback, this);
    ftsSub = nh.subscribe("/ft_sensor/raw_249", 1, &PretreatmentFacade::FTSCallback, this);
    pointCloudProcessor = new PointCloudProcessor();   
    objectScanerMotionPlanner = new ObjectScanerMotionPlanner();

}
void PretreatmentFacade::SetGui(PretreatmentGUI* pretreatmentGUI_)
{
    pretreatmentGUI = pretreatmentGUI_;
}
std::vector<std::string> PretreatmentFacade::EnterFileNameButtonClicked(std::string name)
{
    std::vector<std::string> pathes;
    
    if(ParsePath(name))
    {
	pathes.push_back(mesh_ply_file_name);
	pathes.push_back(mesh_stl_file_name);
    }
    
    return pathes;
}
bool PretreatmentFacade::ParsePath(std::string name_)
{
    name = name_;
    std::string file_path = ros::package::getPath("object_recognition_msd_pcl") + "/models/";
    std::string cloud_file_type = "_cloud.ply";
    std::string mesh_ply_file_type = ".ply";
    std::string mesh_stl_file_type = ".stl";
    
    if(name.length() == 0)	// другие варианты некорректного имени
    {
	return false;
    }
    else
    {
	cloud_file_name = file_path + name + cloud_file_type;
	mesh_ply_file_name = file_path + name + mesh_ply_file_type;
	mesh_stl_file_name = file_path + name + mesh_stl_file_type;
	
	return true;
    }
}
void PretreatmentFacade::CloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    if(!cloud_received)
    {
	pcl::fromROSMsg(*input, *cloud_input);
	cloud_received = true;
    }
}
void PretreatmentFacade::FTSCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    if(!force_received)
    {
	forces_raw.clear();	
	forces_raw.push_back(msg->wrench.force.x);
	forces_raw.push_back(msg->wrench.force.y);
	forces_raw.push_back(msg->wrench.force.z);
	
	forces.clear();	
	forces.push_back(forces_raw[0] - bias[0]);
	forces.push_back(forces_raw[1] - bias[1]);
	forces.push_back(forces_raw[2] - bias[2]);
	
// 	force_input = sqrt(msg->wrench.force.x * msg->wrench.force.x + msg->wrench.force.y * msg->wrench.force.y + msg->wrench.force.z * msg->wrench.force.z);
	force_received = true;
    }
}
void PretreatmentFacade::PublishCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "rotary_table";
    msg.header.stamp = ros::Time::now();
    cloudPub.publish(msg);
    
}
void PretreatmentFacade::ViewpointButtonClicked()
{
    AddBackgroundJob(boost::bind(&PretreatmentFacade::GoToViewpoint_f, this), "Go to viewpoint"); 
    AddBackgroundJob(boost::bind(&PretreatmentFacade::GoToStartPosition_f, this), "Go to start position");
}
void PretreatmentFacade::SetBiasButtonClicked()
{
    force_received = false;   
//     while (!force_received)
//     {
// 	ros::spinOnce();
// 	ros::Duration(0.01).sleep();
// 	 ROS_INFO_STREAM("Waiting for force");
// 	if (force_received)
// 	{    
// 	    bias.clear();
// 	    bias.push_back(forces_raw[0]);
// 	    bias.push_back(forces_raw[1]);
// 	    bias.push_back(forces_raw[2]);	    
// 	}
//     }  
//     massBias = 0.0;
//     for(i = 0; i < f.size(); i++)
//     {
// 	massBias = massBias + f[i] / (f.size() * 9.80665);
//     }
    
    
    pretreatmentGUI->SetBias_done();
}
void PretreatmentFacade::ScanButtonClicked()
{      
    cloud_input.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pointCloudProcessor->Clear();
    masses.clear();
    
    angle = 45;
    cloud_count = 0;
    
    AddBackgroundJob(boost::bind(&PretreatmentFacade::RotateTable_f, this), "Rotate table");
}
void PretreatmentFacade::GoToViewpoint_f()
{
    objectScanerMotionPlanner->ArmToViewpoint();
}
void PretreatmentFacade::GoToStartPosition_f()
{    
    objectScanerMotionPlanner->RotateTableToStartPosition();
    pretreatmentGUI->Viewpoint_done();
}
void PretreatmentFacade::RotateTable()
{
    cloud_received = false;
    force_received = false;
    
    while (!cloud_received && !force_received)
    {
	ros::spinOnce();
	ros::Duration(0.01).sleep();
    }     
//     if (cloud_received && force_received)
//     {
	pointCloudProcessor->SetInputCloud(cloud_input, angle, cloud_count);
	masses.push_back(sqrt(forces[0] * forces[0] + forces[1] * forces[1] + forces[2] * forces[2]));
//     }
    cloud_count++;

    PublishCloud(pointCloudProcessor->CloudOutput);
    AddBackgroundJob(boost::bind(&PretreatmentFacade::RotateTable_f, this), "Rotate table");

}
void PretreatmentFacade::RotateTable_f()
{ 

    objectScanerMotionPlanner->RotateTable(cloud_count, angle);
    
    if(cloud_count < (360 / angle))
    {
	RotateTable();
    }
    else
    {
	AddBackgroundJob(boost::bind(&PretreatmentFacade::GoToHomePosition_f, this), "Go to home position");
    }
    
}
void PretreatmentFacade::GoToHomePosition_f()
{
    objectScanerMotionPlanner->RotateTable(0, angle);
    pointCloudProcessor->CreateFinalCloud();
    PublishCloud(pointCloudProcessor->CloudFinal);
    
    pcl::io::savePLYFileBinary(cloud_file_name, *pointCloudProcessor->CloudFinal);
    pcl::io::savePolygonFilePLY(mesh_ply_file_name, pointCloudProcessor->mesh);
    pcl::io::savePolygonFileSTL(mesh_stl_file_name, pointCloudProcessor->mesh);
    
    CalcMass();
    CalcDimensions();
    
    pretreatmentGUI->Scan_done();
}
void PretreatmentFacade::CalcMass()
{
    float mass = 0.0;
    for(int i = 0; i < masses.size(); i++)
    {
	mass = mass + masses[i] / (masses.size() * 9.80665);
    }
    mass = mass;// - massBias;
    pretreatmentGUI->SetMass(mass);
}
void PretreatmentFacade::CalcDimensions()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://object_recognition_msd_pcl/models/" + name + ".stl";
    
    BoundingBox* boundingBox = new BoundingBox();
    boundingBox->SetObject(marker);
    
    pretreatmentGUI->SetDimensions(boundingBox->GetDimensions());
}
void PretreatmentFacade::AddBackgroundJob(const boost::function<void()> &job, const std::string &name)
{
    background_process_.addJob(job, name);
}

}//end namespace rviz