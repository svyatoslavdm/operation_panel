#include "operation_panel/ServersManagers/InteractiveMarkerServerManager.h"

InteractiveMarkerServerManager::InteractiveMarkerServerManager(boost::shared_ptr<MarkerServer> markerServer, boost::shared_ptr<TFServer> tfServer, 
							       boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interaktiveMarkerServer, Scene* scene)
{
    interactiveMarkerArrow = new InteractiveMarkerArrow();
    interactiveMarkerArrow->SetTFServer(tfServer);
    interactiveMarkerArrow->SetInteractiveMarkerServer(interaktiveMarkerServer);
    
    interactiveMarkerPoint = new InteractiveMarkerPoint();
    interactiveMarkerPoint->SetTFServer(tfServer);
    interactiveMarkerPoint->SetInteractiveMarkerServer(interaktiveMarkerServer);
    
    interactiveMarkerObject = new InteractiveMarkerObject();
    interactiveMarkerObject->SetMarkerServer(markerServer);
    interactiveMarkerObject->SetTFServer(tfServer);
    interactiveMarkerObject->SetInteractiveMarkerServer(interaktiveMarkerServer);
    interactiveMarkerObject->SetSceneInformation(scene); 
    interactiveMarkerObject->UpdateTablesInformation();
    
    interactiveMarkerGraspChooser = new InteractiveMarkerGraspChooser();
    interactiveMarkerGraspChooser->SetTFServer(tfServer);
    interactiveMarkerGraspChooser->SetInteractiveMarkerServer(interaktiveMarkerServer);
    
    interactiveMarkerUnscrewChooser = new InteractiveMarkerUnscrewChooser();
    interactiveMarkerUnscrewChooser->SetTFServer(tfServer);
    interactiveMarkerUnscrewChooser->SetInteractiveMarkerServer(interaktiveMarkerServer);
}

void InteractiveMarkerServerManager::CreateArrow(std::string name)
{
    interactiveMarkerArrow->Add(name);
} 

void InteractiveMarkerServerManager::CreatePoint(std::string name)
{
    interactiveMarkerPoint->Add(name);
}

void InteractiveMarkerServerManager::UpdateRotaryTableInformation()
{    
    interactiveMarkerObject->UpdateRotaryTableInformation();
}

void InteractiveMarkerServerManager::CreateObject(std::string name)
{    
    interactiveMarkerObject->UpdateObjectInformation(name);
    interactiveMarkerObject->Add(name);
}

void InteractiveMarkerServerManager::CreateGraspChooser(std::string name)
{
    interactiveMarkerGraspChooser->Add(name);
}

void InteractiveMarkerServerManager::CreateUnscrewChooser(std::string name)
{
    interactiveMarkerUnscrewChooser->Add(name);
}

void InteractiveMarkerServerManager::SetAdherence(bool adherence)
{
    interactiveMarkerObject->SetAdherence(adherence);
}

void InteractiveMarkerServerManager::Remove(std::string name)
{
    interactiveMarkerArrow->Remove(name);
}

void InteractiveMarkerServerManager::RemoveGraspChooser(std::string name)
{
    interactiveMarkerArrow->Remove(name + "_gripper_rotation");
    interactiveMarkerArrow->Remove(name + "_gripper_position");
}

void InteractiveMarkerServerManager::RemoveUnscrewChooser(std::string name)
{
    interactiveMarkerArrow->Remove("unscrew_rotation");
    interactiveMarkerArrow->Remove("unscrew_position");
}