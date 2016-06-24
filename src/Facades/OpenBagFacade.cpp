#include <operation_panel/Facades/OpenBagFacade.h>
#include <operation_panel/GUI/OpenBagGUI.h>

namespace rviz
{
    
OpenBagFacade::OpenBagFacade(ros::NodeHandle& nh)
{     
    boost::shared_ptr<TFServer> tfServer;
    tfServer.reset( new TFServer(nh) );
    tfServer->start();
    
    boost::shared_ptr<MarkerServer> markerServer;
    markerServer.reset( new MarkerServer(nh, "markers") );
    markerServer->start();
    
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interaktiveMarkerServer;
    interaktiveMarkerServer.reset( new interactive_markers::InteractiveMarkerServer("markers_control_open_bag","",false) );
    interaktiveMarkerServer->applyChanges();
    
    scene = new Scene(nh);

    HomeGripperClient = nh.serviceClient<std_srvs::Empty>("/wsg_50_driver/homing");     
        
    markerServerManager = new MarkerServerManager(markerServer, scene);
    tfServerManager = new TFServerManager(markerServer, tfServer, scene);
    interaktiveMarkerServerManager = new InteractiveMarkerServerManager(markerServer, tfServer, interaktiveMarkerServer, scene); 
    
    openBagMotionPlanner = new OpenBagMotionPlanner(tfServer);
}

void OpenBagFacade::SetGui(OpenBagGUI* openBagGUI_)
{
    openBagGUI = openBagGUI_;
}

void OpenBagFacade::CreateMarkers()
{
    markerServerManager->CreateArrow("arrow");
    markerServerManager->CreatePoint("point0");
    markerServerManager->CreatePoint("point1");
}

void OpenBagFacade::DeleteMarkers()
{
    markerServerManager->Remove("arrow");
    markerServerManager->Remove("point0");
    markerServerManager->Remove("point1");
}

void OpenBagFacade::CreateTF()
{
    tfServerManager->CreateArrow("arrow");
    tfServerManager->CreatePoint("point0");
    tfServerManager->CreatePoint("point1");
}

void OpenBagFacade::DeleteTF()
{
    tfServerManager->Remove("arrow");
    tfServerManager->Remove("point0");
    tfServerManager->Remove("point1");
}

void OpenBagFacade::CreateIntMarkers()
{
    interaktiveMarkerServerManager->CreateArrow("arrow");
    interaktiveMarkerServerManager->CreatePoint("point0");
    interaktiveMarkerServerManager->CreatePoint("point1");
}

void OpenBagFacade::DeleteIntMarkers()
{
    interaktiveMarkerServerManager->Remove("arrow");
    interaktiveMarkerServerManager->Remove("point0");
    interaktiveMarkerServerManager->Remove("point1");
}

void OpenBagFacade::DeleteTrajectory()
{
    openBagMotionPlanner->DeleteTrajectory();
}

void OpenBagFacade::GoToViewpoint()
{
    AddBackgroundJob(boost::bind(&OpenBagFacade::GoToViewpoint_f, this), "GoToViewpoint");   
}

void OpenBagFacade::GoToViewpoint_f()
{
    std_srvs::Empty empty;
    HomeGripperClient.call(empty);
    openBagMotionPlanner->Viewpoint();
    openBagGUI->GoToViewpoint_done();
}

void OpenBagFacade::PlanTrajectory()
{
    AddBackgroundJob(boost::bind(&OpenBagFacade::PlanTrajectory_f, this), "Execute trajectory");  
}

void OpenBagFacade::PlanTrajectory_f()
{
    openBagMotionPlanner->PlanBagOpeningMovement();
    ros::Duration(12.0).sleep();
    
    openBagGUI->PlanTrajectory_done();
}

void OpenBagFacade::ExecutTrajectory()
{
    AddBackgroundJob(boost::bind(&OpenBagFacade::ExecutTrajectory_f, this), "Execute trajectory");   
}

void OpenBagFacade::ExecutTrajectory_f()
{
    openBagMotionPlanner->Execution();
    AddBackgroundJob(boost::bind(&OpenBagFacade::GoToViewpoint_f, this), "GoToViewpoint");
}

void OpenBagFacade::AddBackgroundJob(const boost::function<void()> &job, const std::string &name)
{
    background_process_.addJob(job, name);
}

}//end namespace rviz