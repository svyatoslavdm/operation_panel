#include <operation_panel/MsgsParsers/ORKMsgsParser.h>
ORKMsgsParser::ORKMsgsParser(ros::NodeHandle _nh) : IMsgsParser(_nh)
{
    nh = _nh;
    
    rec_obj_array_sub = nh.subscribe("/recognized_object_array_", 1000, &ORKMsgsParser::ORKCallback, this);
}
bool ORKMsgsParser::GetObject(visualization_msgs::Marker& _object)
{
    if (object.ns != "")
    {
	_object = object;
	return true;
    }
    else return false;
}
void ORKMsgsParser::ORKCallback(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg)
{
    ROS_INFO_STREAM("ORKCallback started, msg->objects.size(): " << msg->objects.size());
    if (msg->objects.size() > 0)
    {
	object.ns = msg->objects[0].type.key;
	object.pose = msg->objects[0].pose.pose.pose;
	object.header.frame_id = msg->objects[0].header.frame_id;
	CreateObject();
    }
}
void ORKMsgsParser::CreateObject()
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
    
    ROS_INFO_STREAM("ORKMsgsParser: MSG parsed! Object name: " << object_name);
}