#include <operation_panel/CollisionObjects/CollisionObjectUtilities.h>
#include <typeindex>
#include <operation_panel/robot_config.h>
CollisionObjectUtilities::CollisionObjectUtilities(moveit::planning_interface::MoveGroup* _move_group, TFServer* _tfServer, MarkerServer* _markerServer) 
						    //: ICollisionObjectController(_move_group, _tfServer, _markerServer)
{
    move_group = _move_group;
    tfServer = _tfServer;
    markerServer = _markerServer;
}

void CollisionObjectUtilities::AddCollisionObject(visualization_msgs::Marker _object)
{
    object = _object;
    
    RemoveCollisionObjects();
    collision_objects.clear();
    
    moveit_msgs::CollisionObject collision_object;
    collision_object.meshes.clear();
    collision_object.mesh_poses.clear();

    collision_object.header.frame_id = object.header.frame_id;
    collision_object.id = object.ns;

    shapes::Mesh* shape_mesh = shapes::createMeshFromResource(object.mesh_resource);
    shape_msgs::Mesh collision_mesh;
    shapes::ShapeMsg collision_mesh_msg;
    shapes::constructMsgFromShape(shape_mesh,collision_mesh_msg);
    collision_mesh = boost::get<shape_msgs::Mesh>(collision_mesh_msg);

    geometry_msgs::Pose collision_object_pose = object.pose;

    collision_object.meshes.push_back(collision_mesh);
    collision_object.mesh_poses.push_back(collision_object_pose);
    collision_object.operation = collision_object.ADD; 
    
    collision_objects.push_back(collision_object);

    planning_scene_interface.addCollisionObjects(collision_objects);

    for (std::vector<moveit_msgs::CollisionObject>::iterator it = collision_objects.begin(); it != collision_objects.end(); it++)
    {
	ROS_INFO_STREAM("Adding collision object with id: " << it->id);
    }
    ros::Duration(0.1).sleep();
    ROS_INFO_STREAM("Added " << collision_objects.size() << " collision objects into the world");
}
void CollisionObjectUtilities::AddMovingCollisionObject(visualization_msgs::Marker _object)
{
    geometry_msgs::Pose collision_object_pose;
    
    std::string parent_frame =  "current_grasp_tool";
    std::string child_frame =  "start_object_marker";
    
    if (tfServer->get(parent_frame,child_frame, collision_object_pose))
    {
// 	_object.header.frame_id = "r/link_6";	
// 	_object.header.frame_id = "kuka/tool";	
	_object.header.frame_id = TOOLNAME;	
	_object.pose = collision_object_pose;	
	_object.ns = "attached_object";	
	AddCollisionObject(_object);
    }
}
void CollisionObjectUtilities::AddFinishStaticCollisionObject(visualization_msgs::Marker _object)
{
    geometry_msgs::Pose collision_object_pose;    
    std::string parent_frame =  "world";
    std::string child_frame =  "finish_object_marker";
    
    if (tfServer->get(parent_frame,child_frame, collision_object_pose))
    {
	_object.header.frame_id = parent_frame;	
	_object.pose = collision_object_pose;	
	_object.ns = "finish_object";	
	AddCollisionObject(_object);
    }
}
void CollisionObjectUtilities::AddStartStaticCollisionObject()
{
    visualization_msgs::Marker _object;    
    markerServer->get("start_object", _object);    
    geometry_msgs::Pose collision_object_pose;    
    std::string parent_frame =  "world";
    std::string child_frame =  "start_object_marker";
    
    if (tfServer->get(parent_frame,child_frame, collision_object_pose))
    {
	_object.header.frame_id = parent_frame;	
	_object.pose = collision_object_pose;	
	_object.ns = "start_object";	
	AddCollisionObject(_object);
    } 
}
void CollisionObjectUtilities::AttachCollisionObject()
{
    std::vector<std::string> _ids;
    std::vector<std::string> touch_links;
    touch_links.push_back("wsg_50/finger_left");
    touch_links.push_back("wsg_50/finger_right");
    
    AddMovingCollisionObject(object);
    
    for (std::vector<moveit_msgs::CollisionObject>::iterator it = collision_objects.begin(); it != collision_objects.end(); it++)
    {
// 	move_group->attachObject(it->id, "r/link_6", touch_links);
	move_group->attachObject(it->id, TOOLNAME, touch_links);
	ROS_INFO_STREAM("Attached collision object " << it->id);
    }
    ros::Duration(0.1).sleep();
}
void CollisionObjectUtilities::DetachCollisionObject()
{
    std::vector<std::string> _ids;
    for (std::vector<moveit_msgs::CollisionObject>::iterator it = collision_objects.begin(); it != collision_objects.end(); it++)
    {
	move_group->detachObject();
	ROS_INFO_STREAM("Detached collision object " << it->id);
    }
    for (std::vector<moveit_msgs::CollisionObject>::iterator it = collision_objects.begin(); it != collision_objects.end(); it++)
    {
	move_group->detachObject(it->id);
	ROS_INFO_STREAM("Detached collision object " << it->id);
    }
    ros::Duration(0.1).sleep(); 
    AddFinishStaticCollisionObject(object);    
    ros::Duration(0.1).sleep();
}
void CollisionObjectUtilities::RemoveCollisionObjects()
{
    std::vector<std::string> _ids;
    int count = 0;
    for (std::vector<moveit_msgs::CollisionObject>::iterator it = collision_objects.begin(); it != collision_objects.end(); it++)
    {
	_ids.push_back(it->id);
	ROS_INFO_STREAM("Removing collision object " << _ids.at(count));
	count++;
    }
    if (_ids.size() > 0)
    {
	planning_scene_interface.removeCollisionObjects(_ids);
	collision_objects.clear();
	ros::Duration(0.1).sleep();
	ROS_INFO_STREAM("Removed collision objects");
    }      
}
void CollisionObjectUtilities::addBackgroundJob(const boost::function<void()> &job, const std::string &name)
{
    background_process_.addJob(job, name);
}
