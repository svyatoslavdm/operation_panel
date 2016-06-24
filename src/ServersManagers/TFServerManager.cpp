#include "operation_panel/ServersManagers/TFServerManager.h"

TFServerManager::TFServerManager(boost::shared_ptr<MarkerServer> markerServer_, boost::shared_ptr<TFServer> tfServer_, Scene* scene)
{
    markerServer = markerServer_;
    tfServer = tfServer_;
    viceGripPolygons = scene->GetViceGripPolygons();
    viceGripRotation = scene->GetViceGripRotation();
    bb = new BoundingBox();
    
    object_pose = false;
    gripper_pose = false;
}

void TFServerManager::CreateArrow(std::string name)
{
    tf::Transform tr;
    
    tfServer->remove("world", name + "_marker");
    
    tr.setOrigin(tf::Vector3((viceGripPolygons[0][1] + viceGripPolygons[0][2]).x() / 2, (viceGripPolygons[0][1] + viceGripPolygons[0][2]).y() / 2, (viceGripPolygons[0][1] + viceGripPolygons[0][2]).z() / 2));
    tr.setRotation(viceGripRotation);
    tfServer->set("world", name + "_control", tr); 

    tr.setOrigin(tf::Vector3(0.16, 0.0, 0.0));
    tr.setRotation(tf::Quaternion(0.0, 0.0, 1.0, 0.0));
    tfServer->set(name + "_control", name + "_marker", tr); 
    
    tr.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tr.setRotation(viceGripRotation);
    tfServer->set("world", "viceGripRotation", tr);
}
void TFServerManager::CreateArrow(std::string parent, std::string name)
{
    tf::Transform tr;
    
    tfServer->remove("world", name + "_marker");
    
    tr.setOrigin(tf::Vector3(0.0, 0.0, 0.19));
    tr.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    tfServer->set(parent, name + "_control", tr); 
    
    tfServer->get("world", name + "_control", tr); 
    tfServer->remove(parent, name + "_control"); 
    tfServer->set("world", name + "_control", tr); 

    tr.setOrigin(tf::Vector3(0.0, 0.0, 0.06));
    tr.setRotation(tf::Quaternion(0.0, 0.707, 0.0, 0.707));
    tfServer->set(name + "_control", name + "_marker", tr); 
}
void TFServerManager::CreatePoint(std::string name)
{
    tf::Transform tr;
    
    tfServer->remove("world", name + "_marker");
    
    if(name.substr(name.length() - 1, 1) == "0")
    {
	tr.setOrigin(tf::Vector3(viceGripPolygons[0][0].x(), viceGripPolygons[0][0].y(), viceGripPolygons[0][0].z()));
    }
    if(name.substr(name.length() - 1, 1) == "1")
    {
	tr.setOrigin(tf::Vector3(viceGripPolygons[1][2].x(), viceGripPolygons[1][2].y(), viceGripPolygons[1][2].z()));
    }
    tr.setRotation(viceGripRotation);
    tfServer->set("world", name + "_control", tr);

    tr.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tr.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    tfServer->set(name + "_control", name + "_marker", tr);
}

void TFServerManager::CreateObject(std::string name)
{
    tf::Transform tr;
	
    tr.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tr.setRotation(viceGripRotation);
    tfServer->set("world", "viceGripRotation", tr);
    
    if(!gripper_pose)
    {
	visualization_msgs::Marker marker;  
	markerServer->get(name, marker);   
	bb->SetObject(marker);
	
	tr.setOrigin(bb->Get_L_bb_ob_ob()); 
	tr.setRotation(bb->Get_Q_bb_ob());

	tfServer->set(name + "_marker", name + "_control", tr);
	ros::Duration(0.1).sleep();
	tfServer->get("world", name + "_control", tr);
	
	tfServer->remove(name + "_marker", name + "_control");
	tfServer->set("world", name + "_control", tr);
	ros::Duration(0.1).sleep();
	
	tfServer->get(name + "_control", name + "_marker", tr);
	tfServer->remove("world", name + "_marker");
	tfServer->set(name + "_control", name + "_marker", tr);
	ros::Duration(0.1).sleep();

	object_pose = true;
    }
}


void TFServerManager::CreateGraspChooser(std::string name)
{     
    if(!object_pose)
    {
	CreateObject(name);
	object_pose = false;
    }
    
    tf::Transform tr;
    tfServer->remove("world", "grasp_candidate_0");
    
    tr.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tr.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    tfServer->set(name + "_control", name + "_gripper_rotation", tr);
    
    tr.setOrigin(tf::Vector3(0.0, 0.0, 0.4));
    tr.setRotation(tf::Quaternion(0.0, 1.0, 0.0, 0.0));
    tfServer->set(name + "_gripper_rotation", name + "_gripper_position", tr);
    
    tr.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tr.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    tfServer->set(name + "_gripper_position", "grasp_marker", tr);
    
    tr.setOrigin(tf::Vector3(0.0, 0.0, -0.062));
    tr.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    tfServer->set("grasp_marker", "grasp_candidate_0", tr);
    
    gripper_pose = true;
}

void TFServerManager::CreateUnscrewChooser(std::string name)
{
    tf::Transform tr;
    tfServer->remove("world", "unscrew_marker");
    
    tr.setOrigin(tf::Vector3(0.0, 0.0, 0.320));
    tr.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    tfServer->set("egn_160/base_link", "unscrew_control", tr);
    
    tr.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tr.setRotation(tf::Quaternion(0.0, -0.707, 0.0, 0.707));
    tfServer->set("unscrew_control", "unscrew_arrow", tr);
    
    tr.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tr.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    tfServer->set("unscrew_control", "unscrew_rotation", tr);
    
    tr.setOrigin(tf::Vector3(0.0, 0.0, 0.4));
    tr.setRotation(tf::Quaternion(0.0, 1.0, 0.0, 0.0) * tf::Quaternion(0.0, 0.0, -0.707, 0.707));
    tfServer->set("unscrew_rotation", "unscrew_position", tr);
    
    tr.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tr.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    tfServer->set("unscrew_position", "unscrew_marker", tr);
}

void TFServerManager::Remove(std::string name)
{
    if(!gripper_pose)
    {
	tf::Transform tr;
	
	tfServer->get("world", name + "_marker", tr); 
		
	tfServer->remove("world", name + "_control");
	tfServer->remove(name + "_control", name + "_marker");
	tfServer->remove(name + "_control", "world");
	tfServer->remove(name + "_marker", name + "_control");
	
	tfServer->set("world", name + "_marker", tr);
		
	tfServer->PublishTFs();
	
	object_pose = false;
    }
}
void TFServerManager::Remove(std::string parent, std::string name)
{
    if(!gripper_pose)
    {
	tf::Transform tr;
	
	tfServer->get(parent, name + "_marker", tr); 
		
	tfServer->remove(parent, name + "_control");
	tfServer->remove(name + "_control", name + "_marker");
	tfServer->remove(name + "_control", parent);
	tfServer->remove(name + "_marker", name + "_control");
	
	tfServer->set(parent, name + "_marker", tr);
	
	object_pose = false;
    }
}
void TFServerManager::RemoveGraspChooser(std::string name)
{
    tf::Transform tr; 
    tfServer->get("world", "grasp_candidate_0", tr);   
    tfServer->set("world", "grasp_candidate_0", tr);
    
    tfServer->remove(name + "_gripper_position", "grasp_candidate_0");
    tfServer->remove(name + "_gripper_rotation", name + "_gripper_position");
    tfServer->remove(name + "_gripper_position", "grasp_marker");
    tfServer->remove(name + "_control", name + "_gripper_rotation");  
    
    if(!object_pose)
    {
	Remove(name); 
    }
    
    gripper_pose = false;
}
void TFServerManager::RemoveUnscrewChooser(std::string name)
{
    tf::Transform tr; 
    tfServer->get("world", "unscrew_marker", tr);   
    tfServer->set("world", "unscrew_marker", tr);
    
    tfServer->remove("unscrew_position", "unscrew_marker");
    tfServer->remove("unscrew_rotation", "unscrew_position");
    tfServer->remove("unscrew_control", "unscrew_rotation");
    tfServer->remove("unscrew_control", "unscrew_arrow");
    tfServer->remove("egn_160/base_link", "unscrew_control");
}

