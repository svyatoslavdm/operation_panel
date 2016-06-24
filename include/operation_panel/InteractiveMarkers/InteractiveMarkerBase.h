#ifndef INTERACTIVE_MARKER_BASE_H_
#define INTERACTIVE_MARKER_BASE_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <operation_panel/Servers/MarkerServer.h>
#include <operation_panel/Servers/TFServer.h>

class InteractiveMarkerBase
{
public: 
    void SetTFServer(boost::shared_ptr<TFServer>);    
    void SetInteractiveMarkerServer(boost::shared_ptr<interactive_markers::InteractiveMarkerServer>);
    void Add(std::string);
    void Remove(std::string);
    void Delete();
    
protected:
    boost::shared_ptr<TFServer> tfSrv;
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> intMarkerSrv;
    
    bool tfReaded;
    
    typedef boost::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)> _processFeedBackTemp;
         
    visualization_msgs::Marker MakeFakeMarker(std::string);    
    visualization_msgs::InteractiveMarkerControl& MakeControl( visualization_msgs::InteractiveMarker &msg ); 
    void WaitTF(std::string);
    void virtual MakeInteractiveMarker(std::string, tf::Quaternion, tf::Quaternion, tf::Quaternion) = 0;
};

#endif //INTERACTIVE_MARKER_BASE_H_
