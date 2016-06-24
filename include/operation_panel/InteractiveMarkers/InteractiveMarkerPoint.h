#ifndef INTERACTIVE_MARKER_POINT_H_
#define INTERACTIVE_MARKER_POINT_H_

#include "InteractiveMarkerBase.h"

class InteractiveMarkerPoint : public InteractiveMarkerBase
{
public:    
    InteractiveMarkerPoint();
    
private:    
    void MakeInteractiveMarker(std::string, tf::Quaternion, tf::Quaternion, tf::Quaternion);
    void ProcessFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);
};

#endif //INTERACTIVE_MARKER_POINT_H_

