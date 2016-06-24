#include <operation_panel/MsgsParsers/ObjectMsgsParser.h>

ObjectMsgsParser::ObjectMsgsParser(ros::NodeHandle _nh) : IMsgsParser(_nh)
{
    nh = _nh;
    
    selected_object_id_sub = nh.subscribe("selected_object_id", 1000, &ObjectMsgsParser::IDCallback, this);
}
bool ObjectMsgsParser::GetObject(visualization_msgs::Marker& _object)
{
    if (object.ns != "")
    {
	visualization_msgs::Marker return_object = object;	
	_object = return_object;
	return true;
    }
    else return false;
}
void ObjectMsgsParser::IDCallback(const std_msgs::String_< std::allocator< void > >::ConstPtr& msg)
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
void ObjectMsgsParser::CreateObject()
{
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
    
    object.pose.position.x = 0.25;
    object.pose.position.y = 0.5;
    object.pose.position.z = 0.4;
    
    object.pose.orientation.x = 0;
    object.pose.orientation.y = 0;
    object.pose.orientation.z = 0;
    object.pose.orientation.w = 1;
    
    object.color.r = 0.5;
    object.color.g = 0.5;
    object.color.b = 0.5;
    object.color.a = 0.5;
    object.header.frame_id = "world";
    
    ROS_INFO_STREAM("ObjectMsgsParser: MSG parsed! Object name: " << object_name);
}
