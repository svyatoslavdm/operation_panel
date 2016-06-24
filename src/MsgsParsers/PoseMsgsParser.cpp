#include <operation_panel/MsgsParsers/PoseMsgsParser.h>
PoseMsgsParser::PoseMsgsParser(ros::NodeHandle _nh) : IMsgsParser(_nh)
{
    nh = _nh;
    
    pose_sub = nh.subscribe("/recognized_object_pose", 1000, &PoseMsgsParser::PoseCallback, this);
    selected_object_id_sub = nh.subscribe("selected_object_id", 1000, &PoseMsgsParser::IDCallback, this);
    
    pose_parsed = false;
}
bool PoseMsgsParser::GetObject(visualization_msgs::Marker& _object)
{
    if (object.ns != "" && pose_parsed)
    {
	visualization_msgs::Marker return_object = object;	
	_object = return_object;
	return true;
    }
    else return false;
}
void PoseMsgsParser::IDCallback(const std_msgs::String_< std::allocator< void > >::ConstPtr& msg)
{
    if (msg->data != "")
    {
	object.ns = msg->data;
	CreateObject();
    }
    else
    {
	ROS_INFO_STREAM("MSG is NOT parsed cause NO ID");
    }
}
void PoseMsgsParser::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    object.pose = msg->pose;
    object.header.frame_id = msg->header.frame_id;
    
    pose_parsed = true;
    ROS_INFO_STREAM("PoseMsgsParser: Pose parsed!");
}
void PoseMsgsParser::CreateObject()
{
    ROS_INFO_STREAM("CreateObject started");
    object_recognition_msgs::ObjectType _type;
    std::string _db = "{\"collection\":\"object_recognition\",\"root\":\"http://localhost:5984\",\"type\":\"CouchDB\"}";
    _type.db = _db;
    _type.key = object.ns;
    
    object_recognition_core::prototypes::ObjectInfo object_info;
    info_cache_.getInfo(_type, object_info);
	
    object_name = object_info.get_field<std::string>("name");
    object.ns = object_name;
    object.type = visualization_msgs::Marker::MESH_RESOURCE;
    object.mesh_resource = object_info.get_field<std::string>("mesh_uri");
    object.scale.x = 1.0;
    object.scale.y = 1.0;
    object.scale.z = 1.0;
    
    object.color.r = 0.5;
    object.color.g = 0.5;
    object.color.b = 0.5;
    object.color.a = 0.5;
    
    ROS_INFO_STREAM("PoseMsgsParser: MSG parsed! Object name: " << object_name);    
}