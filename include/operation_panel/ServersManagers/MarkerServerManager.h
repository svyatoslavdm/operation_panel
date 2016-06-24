#ifndef MARKER_SERVER_MANAGER_H
#define MARKER_SERVER_MANAGER_H

#include "operation_panel/ServersManagers/IServerManager.h"

class MarkerServerManager : public IServerManager
{
public:
    MarkerServerManager(boost::shared_ptr<MarkerServer>, Scene*);
    void CreateArrow(std::string); 
    void CreatePoint(std::string);
    void CreateObject(std::string);
    void CreateGraspChooser(std::string);
    void CreateUnscrewChooser(std::string);
    void Remove(std::string);
    void RemoveGraspChooser(std::string);
    void RemoveUnscrewChooser(std::string);

private:
    boost::shared_ptr<MarkerServer> markerServer;
    std::vector<visualization_msgs::Marker> gripperMarkers;
};

#endif // MARKER_SERVER_MANAGER_H