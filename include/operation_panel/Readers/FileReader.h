#ifndef FILEREADER_H
#define FILEREADER_H

#include <operation_panel/Readers/IGraspReader.h>

#include <Eigen/Geometry>
#include <fstream>

using namespace std;

class FileReader : public IGraspReader
{
    
public:
  
    FileReader(boost::shared_ptr<TFServer>, std::string, std::string, std::string, std::string);
    
    void set_grasp_candidates();
    
    std::vector<std::pair<std::string, std::string>> tfnames;
  
private:
  
    std::string filepath;
    std::string object_frame, tool_frame, grasp_frame;
    boost::shared_ptr<TFServer> server;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> points;
    
    void read_grasps(); 
    std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> get_in_tool();

};
#endif // FILEREADER_H
