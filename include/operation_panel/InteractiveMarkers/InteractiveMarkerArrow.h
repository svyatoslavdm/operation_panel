#ifndef INTERACTIVE_MARKER_ARROW_H_
#define INTERACTIVE_MARKER_ARROW_H_

#include "InteractiveMarkerBase.h"

class InteractiveMarkerArrow : public InteractiveMarkerBase
{
public:    
    InteractiveMarkerArrow();
    
private:     
    void MakeInteractiveMarker(std::string, tf::Quaternion, tf::Quaternion, tf::Quaternion);
    void ProcessFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);
};

#endif //INTERACTIVE_MARKER_ARROW_H_

