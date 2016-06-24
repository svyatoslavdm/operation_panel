#ifndef COORDSSERVER_H
#define COORDSSERVER_H

#include <string.h>
#include <map>
#include <vector>

#include <ros/ros.h>


#include <geometry_msgs/Pose.h>

#include <Eigen/Eigen>

struct frame
{
    std::string parent;
    std::string name;
    Eigen::Vector3f L_p_n_p;
    Eigen::Quaternionf Q_p_n; 
};

class  CoordsServer 
{
public:
    void set(std::string, std::string, geometry_msgs::Pose);
    
    bool get(std::string, std::string, geometry_msgs::Pose&);
    
private:
    std::map<std::string, frame> frames;
    
    bool isConnected(std::string, std::string);

};

#endif //  COORDSSERVER_H
