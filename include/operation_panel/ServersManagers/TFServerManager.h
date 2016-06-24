#ifndef TF_SERVER_MANAGER_H
#define TF_SERVER_MANAGER_H

#include "operation_panel/ServersManagers/IServerManager.h"

class TFServerManager : public IServerManager
{
public:
    TFServerManager(boost::shared_ptr<MarkerServer>, boost::shared_ptr<TFServer>, Scene*);
    void CreateArrow(std::string); 
    void CreateArrow(std::string, std::string); 
    void CreatePoint(std::string);
    void CreateObject(std::string);
    void CreateGraspChooser(std::string);
    void CreateUnscrewChooser(std::string);
    void Remove(std::string);
    void Remove(std::string, std::string); 
    void RemoveGraspChooser(std::string);
    void RemoveUnscrewChooser(std::string);

private:
    boost::shared_ptr<MarkerServer> markerServer;
    boost::shared_ptr<TFServer> tfServer;
    std::vector< std::vector< tf::Vector3 > > viceGripPolygons;
    tf::Quaternion viceGripRotation;
    bool tfRemove;
    bool object_pose;
    bool gripper_pose;
    BoundingBox* bb;
    
    void WaitTF(std::string);
};

#endif // TF_SERVER_MANAGER_H