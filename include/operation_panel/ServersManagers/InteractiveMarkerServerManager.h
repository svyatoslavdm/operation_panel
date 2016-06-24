#ifndef INTERACTIVE_MARKER_SERVER_MANAGER_H
#define INTERACTIVE_MARKER_SERVER_MANAGER_H

#include "operation_panel/ServersManagers/IServerManager.h"

#include <operation_panel/InteractiveMarkers/InteractiveMarkerArrow.h>
#include <operation_panel/InteractiveMarkers/InteractiveMarkerPoint.h>
#include <operation_panel/InteractiveMarkers/InteractiveMarkerObject.h>
#include <operation_panel/InteractiveMarkers/InteractiveMarkerGraspChooser.h>
#include <operation_panel/InteractiveMarkers/InteractiveMarkerUnscrewChooser.h>

class InteractiveMarkerServerManager : public IServerManager
{
public:
    InteractiveMarkerServerManager(boost::shared_ptr<MarkerServer>, boost::shared_ptr<TFServer>, boost::shared_ptr<interactive_markers::InteractiveMarkerServer>, Scene*);
    void CreateArrow(std::string); 
    void CreatePoint(std::string);
    void CreateObject(std::string);
    void CreateGraspChooser(std::string);
    void CreateUnscrewChooser(std::string);
    void UpdateRotaryTableInformation();
    void SetAdherence(bool);
    void Remove(std::string);
    void RemoveGraspChooser(std::string);
    void RemoveUnscrewChooser(std::string);
    
private:
    InteractiveMarkerArrow* interactiveMarkerArrow;
    InteractiveMarkerPoint* interactiveMarkerPoint;
    InteractiveMarkerObject* interactiveMarkerObject;
    InteractiveMarkerGraspChooser* interactiveMarkerGraspChooser;
    InteractiveMarkerUnscrewChooser* interactiveMarkerUnscrewChooser;
};

#endif // INTERACTIVE_MARKER_SERVER_MANAGER_H