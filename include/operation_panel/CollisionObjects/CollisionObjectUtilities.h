#ifndef COLLISIONOBJECTUTILITIES_H_
#define COLLISIONOBJECTUTILITIES_H_

#include <operation_panel/CollisionObjects/ICollisionObjectController.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/background_processing/background_processing.h>

#include <visualization_msgs/Marker.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

#include <operation_panel/Servers/TFServer.h>
#include <operation_panel/Servers/MarkerServer.h>

class CollisionObjectUtilities : public ICollisionObjectController
{
public:
    
    CollisionObjectUtilities(moveit::planning_interface::MoveGroup*, TFServer*, MarkerServer*);
    
    void AddStartStaticCollisionObject();
    
    void RemoveCollisionObjects();
    
    void AttachCollisionObject();
    
    void DetachCollisionObject();
    
//     void SetObject();
    
    
private:
    
    MarkerServer* markerServer;
    
    TFServer* tfServer;
    
    moveit::planning_interface::MoveGroup* move_group;
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    visualization_msgs::Marker object;
    
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    
    moveit::tools::BackgroundProcessing background_process_;
    
    void AddCollisionObject(visualization_msgs::Marker);
    
    void AddMovingCollisionObject(visualization_msgs::Marker);
    
    void AddFinishStaticCollisionObject(visualization_msgs::Marker);
    
    void addBackgroundJob(const boost::function<void()> &job, const std::string &name);
    
    void AddStartStaticCollisionObjectThread();
    
    void RemoveCollisionObjectsThread();
    
    void AttachCollisionObjectThread();
    
    void DetachCollisionObjectThread();
};

#endif