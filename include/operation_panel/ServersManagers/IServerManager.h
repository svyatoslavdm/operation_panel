#ifndef I_SERVER_MANAGER_H
#define I_SERVER_MANAGER_H

#include <ros/ros.h>

#include <operation_panel/Servers/MarkerServer.h>
#include <operation_panel/Servers/TFServer.h>
#include <interactive_markers/interactive_marker_server.h>

#include "operation_panel/Scene/Scene.h"
#include "operation_panel/BoundingBox/BoundingBox.h"

class IServerManager
{
public:   
    virtual void CreateArrow(std::string) = 0; 
    virtual void CreatePoint(std::string) = 0; 
    virtual void CreateObject(std::string) = 0; 
    virtual void CreateGraspChooser(std::string) = 0; 
    virtual void Remove(std::string) = 0; 
    virtual void RemoveGraspChooser(std::string) = 0;     
};

#endif // I_SERVER_MANAGER_H