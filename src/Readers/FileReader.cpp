#include <operation_panel/Readers/FileReader.h>

FileReader::FileReader(boost::shared_ptr<TFServer> server_, std::string filepath_, std::string object_frame_, std::string tool_frame_, std::string grasp_frame_)
{
    server = server_;  
    filepath = filepath_;
    object_frame = object_frame_;
    tool_frame = tool_frame_;		// r/link_6
    grasp_frame = grasp_frame_;		// wsg_50/palm_link
}

void FileReader::set_grasp_candidates()
{
    std::string name;
    read_grasps();
    tfnames.clear();
    
    std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> points_in_tool;
    points_in_tool = get_in_tool();
    
    for (int i = 0; i < (int)points_in_tool.size(); i++)
    {
	name = "grasp_candidate_" + std::to_string(i);
	tfnames.push_back(std::make_pair(object_frame, name));
	server->set(object_frame, name, points_in_tool.at(i).first, points_in_tool.at(i).second);
    }
}

std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> FileReader::get_in_tool()
{
    std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> points_in_tool;

    Eigen::Vector3f object_to_grasp_in_object_translation, object_to_tool_in_object_translation, grasp_to_tool_in_grasp_translation;
    Eigen::Quaternionf object_to_grasp_rotation, object_to_tool_rotation, grasp_to_tool_rotation;
    
    server->get(grasp_frame, tool_frame, grasp_to_tool_in_grasp_translation, grasp_to_tool_rotation);
    for (const auto& point : points)
    {	
	object_to_grasp_in_object_translation = point.first;
	object_to_grasp_rotation = point.second;	
	object_to_tool_in_object_translation = object_to_grasp_in_object_translation + object_to_grasp_rotation * grasp_to_tool_in_grasp_translation;	
	object_to_tool_rotation = object_to_grasp_rotation * grasp_to_tool_rotation;	
	points_in_tool.push_back(std::make_pair(object_to_tool_in_object_translation, object_to_tool_rotation));
    }   
    return points_in_tool;
}

void FileReader::read_grasps()
{
    ifstream fin(filepath);
    points.clear();
    
    double matrix[9];
    double coordinates[3];
	
    std::string file_string;
    bool stopped = false;
    
    while (!stopped)
    {
	for(int i = 0; i < 9; i++)
	{
	    if(std::getline(fin, file_string))
	    {
		std::istringstream in(file_string);
		in >> matrix[i];
	    }
	    else
	    {
		stopped = true;
		break;
	    }
	}
	for(int i = 0; i < 3; i++)
	{
	    if(std::getline(fin, file_string))
	    {
		std::istringstream in(file_string);
		in >> coordinates[i];
	    }
	    else
	    {
		stopped = true;
		break;
	    }
	}
	if (!stopped)
	{
	    Eigen::Vector3f grasp_position;
	    grasp_position << coordinates[0], coordinates[1], coordinates[2];
	    
	    Eigen::Matrix3f grasp_tau;
	    Eigen::Quaternionf grasp_quaternion;

	    grasp_tau << matrix[0], matrix[1], matrix[2], matrix[3], matrix[4], matrix[5], matrix[6], matrix[7], matrix[8];
	    grasp_quaternion = grasp_tau;
	    
	    points.push_back(std::make_pair(grasp_position, grasp_quaternion));
	}
    }  
    fin.close();
}
