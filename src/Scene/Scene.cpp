#include "operation_panel/Scene/Scene.h"
#include <operation_panel/robot_config.h>
Scene::Scene(ros::NodeHandle& nh)
{    
    std::string robotDescription;
    nh.getParam("robot_description", robotDescription);
    GetRobotModel(robotDescription);
    GetTableProperty("operation_table", operationTablePolygons, operationTableNormals);
    GetTableProperty("rotary_table", rotaryTablePolygons, rotaryTableNormals);
    GetViceGripProperty();
    CreateGripperMarkers();
}

std::vector< std::vector< tf::Vector3 > > Scene::GetOperationTablePolygons()
{
    return operationTablePolygons;
}
    
std::vector< tf::Vector3 > Scene::GetOperationTableNormals()
{
    return operationTableNormals;
}

std::vector< std::vector< tf::Vector3 > > Scene::GetRotaryTablePolygons()
{
    return rotaryTablePolygons;
}
    
std::vector< tf::Vector3 > Scene::GetRotaryTableNormals()
{
    return rotaryTableNormals;
}

std::vector< std::vector< tf::Vector3 > > Scene::GetViceGripPolygons()
{
    return viceGripPolygons;
}

std::vector< tf::Vector3 > Scene::GetViceGripNormals()
{
    return viceGripNormals;
}

tf::Vector3 Scene::GetViceGripCenter()
{
    return viceGripCenter;
}

std::vector<visualization_msgs::Marker> Scene::GetGripperMarkers()
{
    return gripperMarkers;
}

bool Scene::GetRobotModel(std::string robotDescription)
{
    TiXmlDocument xml;
    xml.Parse( robotDescription.c_str() );
    if( !xml.RootElement() )
    {
	ROS_ERROR_STREAM("No urdf found.");
	return false;
    }

    if( !robotModel.initXml( xml.RootElement() ))
    {
	ROS_ERROR_STREAM("No parse urdf.");
	return false;
    }
    return true;
}

void Scene::GetTableProperty(std::string name, std::vector< std::vector< tf::Vector3 > >& polygons, std::vector< tf::Vector3 >& normals)
{    
    pcl::PolygonMesh mesh = GetTableMesh(name);
    tf::Transform meshTransform = GetTableTransform(name);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(mesh.cloud, *cloud); 
    tf::Vector3 L_mesh_table_table = meshTransform.getOrigin();
    tf::Quaternion Q_mesh_table_table = meshTransform.getRotation();
    tf::Vector3 vec_a, vec_b, vec_c, vec_ob, vec_a_b, vec_a_c, normal;
    std::vector< tf::Vector3 > polygon;
    float sABC, sAOB, sBOC, sCOA, h;
    for(int i = 0; i < mesh.polygons.size(); i++)
    {
	vec_a.setX(cloud->points[mesh.polygons[i].vertices[0]].x);
	vec_a.setY(cloud->points[mesh.polygons[i].vertices[0]].y);
	vec_a.setZ(cloud->points[mesh.polygons[i].vertices[0]].z);
	
	vec_b.setX(cloud->points[mesh.polygons[i].vertices[1]].x);
	vec_b.setY(cloud->points[mesh.polygons[i].vertices[1]].y);
	vec_b.setZ(cloud->points[mesh.polygons[i].vertices[1]].z);
	
	vec_c.setX(cloud->points[mesh.polygons[i].vertices[2]].x);
	vec_c.setY(cloud->points[mesh.polygons[i].vertices[2]].y);
	vec_c.setZ(cloud->points[mesh.polygons[i].vertices[2]].z);
	
	vec_a = L_mesh_table_table + tf::quatRotate(Q_mesh_table_table, vec_a);
	vec_b = L_mesh_table_table + tf::quatRotate(Q_mesh_table_table, vec_b);
	vec_c = L_mesh_table_table + tf::quatRotate(Q_mesh_table_table, vec_c);	   
	
	vec_a_b = vec_b - vec_a;
	vec_a_c = vec_c - vec_a;
	normal = vec_a_b.cross(vec_a_c) / (vec_a_b.cross(vec_a_c)).length();

	polygon.clear();

	polygon.push_back(vec_a);
	polygon.push_back(vec_b);
	polygon.push_back(vec_c);

	polygons.push_back(polygon);
	normals.push_back(normal);
    }
}

pcl::PolygonMesh Scene::GetTableMesh(std::string name)
{
    pcl::PolygonMesh mesh;
    std::string mesh_path, pkg_path;
    mesh_path = boost::dynamic_pointer_cast<const urdf::Mesh>(robotModel.getLink(name)->visual->geometry)->filename;
    mesh_path = mesh_path.substr(mesh_path.find("/") + 2, mesh_path.length() - mesh_path.find("/") - 2);
    pkg_path = ros::package::getPath(mesh_path.substr(0, mesh_path.find("/")));	
    mesh_path = pkg_path + mesh_path.substr(mesh_path.find("/"), mesh_path.length() - mesh_path.find("/"));

    if(mesh_path.substr(mesh_path.length() - 3, 3) == "ply")
    {
	pcl::io::loadPolygonFilePLY (mesh_path, mesh);
    }
    else if(mesh_path.substr(mesh_path.length() - 3, 3) == "stl")
    {
	pcl::io::loadPolygonFileSTL (mesh_path, mesh);
    }
    else
    {
	ROS_ERROR("Unknown type of polygon file");
    }
    return mesh;
} 

tf::Transform Scene::GetTableTransform(std::string name)
{
    tf::Transform transform;    
    tf::Vector3 L_mesh_table_table;
    tf::Quaternion Q_mesh_table;

    L_mesh_table_table.setX(robotModel.getLink(name)->visual->origin.position.x);
    L_mesh_table_table.setY(robotModel.getLink(name)->visual->origin.position.y);
    L_mesh_table_table.setZ(robotModel.getLink(name)->visual->origin.position.z);

    Q_mesh_table.setX(robotModel.getLink(name)->visual->origin.rotation.x);
    Q_mesh_table.setY(robotModel.getLink(name)->visual->origin.rotation.y);
    Q_mesh_table.setZ(robotModel.getLink(name)->visual->origin.rotation.z);
    Q_mesh_table.setW(robotModel.getLink(name)->visual->origin.rotation.w);
    
    transform.setOrigin(L_mesh_table_table);
    transform.setRotation(Q_mesh_table);
    
    return transform;
}

void Scene::GetViceGripProperty()
{
    GetViceGripArea(GetViceGripTransform());
};

tf::Transform Scene::GetViceGripTransform()
{
    tf::Transform viceGripMeshTransform;
    tf::Vector3 L_viceGrip_w_w;
    tf::Quaternion Q_viceGrip_w;

    L_viceGrip_w_w.setX(robotModel.getJoint("egn_160/base_joint")->parent_to_joint_origin_transform.position.x);
    L_viceGrip_w_w.setY(robotModel.getJoint("egn_160/base_joint")->parent_to_joint_origin_transform.position.y);
    L_viceGrip_w_w.setZ(robotModel.getJoint("egn_160/base_joint")->parent_to_joint_origin_transform.position.z);

    Q_viceGrip_w.setX(robotModel.getJoint("egn_160/base_joint")->parent_to_joint_origin_transform.rotation.x);
    Q_viceGrip_w.setY(robotModel.getJoint("egn_160/base_joint")->parent_to_joint_origin_transform.rotation.y);
    Q_viceGrip_w.setZ(robotModel.getJoint("egn_160/base_joint")->parent_to_joint_origin_transform.rotation.z);
    Q_viceGrip_w.setW(robotModel.getJoint("egn_160/base_joint")->parent_to_joint_origin_transform.rotation.w);

    viceGripMeshTransform.setOrigin(L_viceGrip_w_w);
    viceGripMeshTransform.setRotation(Q_viceGrip_w);

    return viceGripMeshTransform;
}

void Scene::GetViceGripArea(tf::Transform viceGripMeshTransform)
{	
    std::vector< tf::Vector3 > polygon;	
    tf::Vector3 viceGripGuide;
    tf::Vector3 vertex_0 (0.0, 0.0, 0.343);
    tf::Vector3 vertex_1 (-0.075, 0.15, 0.343);
    tf::Vector3 vertex_2 (0.075, 0.15, 0.343);
    tf::Vector3 vertex_3 (0.075, -0.15, 0.343);
    tf::Vector3 vertex_4 (-0.075, -0.15, 0.343);

    viceGripCenter = viceGripMeshTransform.getOrigin() + tf::quatRotate(viceGripMeshTransform.getRotation(), vertex_0);
    viceGripGuide = tf::quatRotate(viceGripMeshTransform.getRotation(), tf::Vector3 (0.0, 0.0, 1.0));

    polygon.clear();
    polygon.push_back(viceGripMeshTransform.getOrigin() + tf::quatRotate(viceGripMeshTransform.getRotation(), vertex_1));
    polygon.push_back(viceGripMeshTransform.getOrigin() + tf::quatRotate(viceGripMeshTransform.getRotation(), vertex_2));
    polygon.push_back(viceGripMeshTransform.getOrigin() + tf::quatRotate(viceGripMeshTransform.getRotation(), vertex_3));	
    viceGripPolygons.push_back(polygon);
    viceGripNormals.push_back(viceGripGuide);

    polygon.clear();	
    polygon.push_back(viceGripMeshTransform.getOrigin() + tf::quatRotate(viceGripMeshTransform.getRotation(), vertex_1));
    polygon.push_back(viceGripMeshTransform.getOrigin() + tf::quatRotate(viceGripMeshTransform.getRotation(), vertex_3));
    polygon.push_back(viceGripMeshTransform.getOrigin() + tf::quatRotate(viceGripMeshTransform.getRotation(), vertex_4));	
    viceGripPolygons.push_back(polygon);
    viceGripNormals.push_back(viceGripGuide);
};

tf::Quaternion Scene::GetViceGripRotation()
{
    return GetViceGripTransform().getRotation();
}

void Scene::CreateGripperMarkers()
{
    visualization_msgs::Marker marker;
    std::string  mesh_path;
    
    gripperMarkers.clear();
    
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.header.frame_id = "_gripper_marker";	
    marker.action = visualization_msgs::Marker::ADD;	
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.5;    

    mesh_path = boost::dynamic_pointer_cast<const urdf::Mesh>(robotModel.getLink("gripper_adapter_link")->visual->geometry)->filename;
    marker.mesh_resource = mesh_path;
    marker.ns = "gripper_adapter_link";
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = -0.03;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    gripperMarkers.push_back(marker);
#ifdef KUKA
    mesh_path = boost::dynamic_pointer_cast<const urdf::Mesh>(robotModel.getLink("xtion/link")->visual->geometry)->filename;
#endif
#ifdef KAWASAKI
    mesh_path = boost::dynamic_pointer_cast<const urdf::Mesh>(robotModel.getLink("kinect/link")->visual->geometry)->filename;
#endif 
    marker.mesh_resource = mesh_path;
    marker.ns = "xtion";
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = -0.03;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 1.0;
    marker.pose.orientation.w = 0.0;
    gripperMarkers.push_back(marker);
    
    mesh_path = boost::dynamic_pointer_cast<const urdf::Mesh>(robotModel.getLink("wsg_50/palm_link")->visual->geometry)->filename;
    marker.mesh_resource = mesh_path;
    marker.ns = "palm_link";
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    gripperMarkers.push_back(marker);
    
    mesh_path = boost::dynamic_pointer_cast<const urdf::Mesh>(robotModel.getLink("wsg_50/finger_right")->visual->geometry)->filename;
    marker.mesh_resource = mesh_path;
    marker.ns = "finger_right";
    marker.pose.position.x = 0.0545;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.023;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 1;
    marker.pose.orientation.w = 0;
    gripperMarkers.push_back(marker);
    
    mesh_path = boost::dynamic_pointer_cast<const urdf::Mesh>(robotModel.getLink("wsg_50/finger_left")->visual->geometry)->filename;
    marker.mesh_resource = mesh_path;
    marker.ns = "finger_left";
    marker.pose.position.x = -0.0545;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.023;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    gripperMarkers.push_back(marker);
    
    mesh_path = boost::dynamic_pointer_cast<const urdf::Mesh>(robotModel.getLink("wsg_50/gripper_right")->visual->geometry)->filename;
    marker.mesh_resource = mesh_path;
    marker.ns = "gripper_right";
    marker.pose.position.x = 0.0545;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 1;
    marker.pose.orientation.w = 0;
    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;
    gripperMarkers.push_back(marker);
    
    mesh_path = boost::dynamic_pointer_cast<const urdf::Mesh>(robotModel.getLink("wsg_50/gripper_left")->visual->geometry)->filename;
    marker.mesh_resource = mesh_path;
    marker.ns = "gripper_left";
    marker.pose.position.x = -0.0545;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    gripperMarkers.push_back(marker);
}