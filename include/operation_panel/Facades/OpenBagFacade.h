#ifndef OPEN_BAG_FACADE_H_
#define OPEN_BAG_FACADE_H_

#include "std_srvs/Empty.h"

#include <moveit/background_processing/background_processing.h>

#include <operation_panel/ServersManagers/MarkerServerManager.h>
#include <operation_panel/ServersManagers/TFServerManager.h>
#include <operation_panel/ServersManagers/InteractiveMarkerServerManager.h>

#include "operation_panel/MotionPlanners/OpenBagMotionPlanner.h"

namespace rviz
{
    
class OpenBagGUI;

class OpenBagFacade
{
public:
    OpenBagFacade(ros::NodeHandle&);
    void SetGui(OpenBagGUI*);
    
    void CreateMarkers();
    void DeleteMarkers();
    void CreateTF();
    void DeleteTF();
    void CreateIntMarkers();
    void DeleteIntMarkers();    
    
    void DeleteTrajectory();
    void GoToViewpoint();
    void PlanTrajectory();
    void ExecutTrajectory();
    
private:    
    OpenBagGUI* openBagGUI;
    OpenBagMotionPlanner* openBagMotionPlanner;
    Scene* scene;
    
    ros::ServiceClient HomeGripperClient;
    
    MarkerServerManager* markerServerManager;
    TFServerManager* tfServerManager;
    InteractiveMarkerServerManager* interaktiveMarkerServerManager;     
    
    void GoToViewpoint_f();
    void PlanTrajectory_f();
    void ExecutTrajectory_f();
    
    void AddBackgroundJob(const boost::function<void()> &, const std::string &);
    moveit::tools::BackgroundProcessing background_process_;
};

}//end namespace rviz

#endif // OPEN_BAG_FACADE_H_