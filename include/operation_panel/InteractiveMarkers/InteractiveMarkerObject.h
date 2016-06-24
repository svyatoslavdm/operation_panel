#ifndef INTERACTIVE_MARKER_OBJECT_H_
#define INTERACTIVE_MARKER_OBJECT_H_

#include <operation_panel/Scene/Scene.h>
#include <operation_panel/BoundingBox/BoundingBox.h>
#include "InteractiveMarkerBase.h"

class InteractiveMarkerObject : public InteractiveMarkerBase
{
public:    
    InteractiveMarkerObject();    
    void SetMarkerServer(boost::shared_ptr<MarkerServer>);
    void SetSceneInformation(Scene*);
    void SetAdherence(bool);
    void UpdateObjectInformation(std::string);
    void UpdateTablesInformation();
    void UpdateRotaryTableInformation();
    
private:    
    boost::shared_ptr<MarkerServer> markerSrv;
    std::vector< std::vector< tf::Vector3 > > allOperationTablePolygons;    
    std::vector< tf::Vector3 > allOperationTableNormals;
    std::vector< std::vector< tf::Vector3 > > allRotaryTablePolygons;    
    std::vector< tf::Vector3 > allRotaryTableNormals;    
    std::vector< std::vector< tf::Vector3 > > operationTablePolygons;    
    std::vector< tf::Vector3 > operationTableNormals;
    std::vector< std::vector< tf::Vector3 > > rotaryTablePolygons;    
    std::vector< tf::Vector3 > rotaryTableNormals;
    std::vector< std::vector< tf::Vector3 > > viceGripPolygons;
    std::vector< tf::Vector3 > viceGripNormals;
    tf::Vector3 viceGripCenter;
    tf::Vector3 dimensions;
    bool adherence;
    BoundingBox* bb;
    visualization_msgs::Marker marker;

    void UpdateSceneInformation(std::string, std::vector< std::vector< tf::Vector3 > >, std::vector< tf::Vector3 >, std::vector< std::vector< tf::Vector3 > >&, std::vector< tf::Vector3 >&);
    visualization_msgs::InteractiveMarkerControl& MakeControl(visualization_msgs::InteractiveMarker&);
    void MakeInteractiveMarker(std::string, tf::Quaternion, tf::Quaternion, tf::Quaternion);    
    void ProcessFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);    
    int GetQuasiZ(tf::Matrix3x3);
    geometry_msgs::Pose ViceGripAdherence(tf::Vector3, tf::Quaternion);
    geometry_msgs::Pose TableAdherence(std::vector< tf::Vector3 >, tf::Vector3, tf::Quaternion, tf::Vector3);	
    int CheckAbove(std::vector< std::vector< tf::Vector3 > >, std::vector< tf::Vector3 >, tf::Vector3);
    tf::Vector3 GetPointOfIntersectionStraightWithPlane(tf::Vector3, tf::Vector3, tf::Vector3, tf::Vector3);    
    tf::Quaternion CalcRotation(char, bool, tf::Vector3, tf::Vector3);
    tf::Vector3 CalcPositionInViceGrip(tf::Vector3, tf::Vector3, float);
    tf::Vector3 CalcPositionOnTable(tf::Vector3, tf::Vector3, tf::Vector3, float);
};

#endif //INTERACTIVE_MARKER_OBJECT_H_

