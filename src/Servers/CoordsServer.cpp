#include <operation_panel/Servers/CoordsServer.h>

void CoordsServer::set(std::string parent, std::string name, geometry_msgs::Pose pose)
{
    frame _frame;
    _frame.parent = parent;
    _frame.name = name;
    _frame.L_p_n_p.x() = pose.position.x;
    _frame.L_p_n_p.y() = pose.position.y;
    _frame.L_p_n_p.z() = pose.position.z;
    
    _frame.Q_p_n.x() = pose.orientation.x;
    _frame.Q_p_n.y() = pose.orientation.y;
    _frame.Q_p_n.z() = pose.orientation.z;
    _frame.Q_p_n.w() = pose.orientation.w;
    
    frames[name] = _frame;
    
}
bool CoordsServer::get(std::string source, std::string dest, geometry_msgs::Pose&)
{
    isConnected(source, dest);
}
bool CoordsServer::isConnected(std::string source, std::string dest)
{
    frame source_frame, dest_frame;
    if ((frames.find(source) != frames.end()) && (frames.find(dest) != frames.end()))
    {
	source_frame = frames.at(source);
	dest_frame = frames.at(dest);
    }
    else return false;
    
    // get chain to world
    std::string parent;
    
    parent = source_frame.parent;
    std::vector<std::string> source_chain;
    source_chain.push_back(source_frame.name);
    do
    {
	frame _tmp_frame = frames.at(parent);
	source_chain.push_back(_tmp_frame.name);
	parent = _tmp_frame.parent;
    }
    while (parent != "world");
	
    parent = dest_frame.parent;
    std::vector<std::string> dest_chain;
    dest_chain.push_back(dest_frame.name);
    do
    {
	frame _tmp_frame = frames.at(parent);
	dest_chain.push_back(_tmp_frame.name);
	parent = _tmp_frame.parent;
    }
    while (parent != "world");
    
    ROS_INFO_STREAM("source_chain");
    for (auto chain : source_chain)
	ROS_INFO_STREAM(chain);
    ROS_INFO_STREAM("dest_chain");
    for (auto chain : dest_chain)
	ROS_INFO_STREAM(chain);
    
    Eigen::Vector3f L_w_d_w;
    Eigen::Quaternionf Q_w_d;    
    
    Eigen::Vector3f L_temp;
    Eigen::Quaternionf Q_temp; 
    
    bool first_element = true;
    
    for (auto chain_element : dest_chain)
    {
	if (first_element)
	{
	    L_temp = frames[chain_element].L_p_n_p;
	    Q_temp = frames[chain_element].Q_p_n; 
	    first_element = false;
	}	
	std::string parent_name = frames[chain_element].parent;
	if (parent_name != "world")
	{
	    Eigen::Vector3f L_parent = frames[parent_name].L_p_n_p;
	    Eigen::Quaternionf Q_parent = frames[parent_name].Q_p_n;

// 	    Q_parent.w() *= -1;
	    L_temp = L_parent + Q_temp * L_temp;
	    
	    Q_temp = Q_parent * Q_temp;
	    
	}
    }
    ROS_INFO_STREAM("Vec: " << " x: " << L_temp.x()  << " y: " << L_temp.y()  << " z: " << L_temp.z() );
    ROS_INFO_STREAM("Q_temp: " << " x: " << Q_temp.x()  << " y: " << Q_temp.y()  << " z: " << Q_temp.z()  << " w: " << Q_temp.w() );
}










