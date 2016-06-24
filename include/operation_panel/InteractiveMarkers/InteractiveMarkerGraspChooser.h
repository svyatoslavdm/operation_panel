#ifndef INTERACTIVE_MARKER_GRASP_CHOOSER_H_
#define INTERACTIVE_MARKER_GRASP_CHOOSER_H_

#include "InteractiveMarkerBase.h"

class InteractiveMarkerGraspChooser : public InteractiveMarkerBase
{
public:    
    InteractiveMarkerGraspChooser();
    
private:  
    void MakeInteractiveMarker(std::string, tf::Quaternion, tf::Quaternion, tf::Quaternion);  
    void MakeRotationMarker(std::string, tf::Quaternion, tf::Quaternion, tf::Quaternion);  
    void MakePositionMarker(std::string, tf::Quaternion, tf::Quaternion, tf::Quaternion);  
    void ProcessRotateFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
    void ProcessPositionFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
}; 
   
#endif //INTERACTIVE_MARKER_GRASP_CHOOSER_H_

