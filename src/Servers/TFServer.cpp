#include <operation_panel/Servers/TFServer.h>
#include <thread>


TFServer::TFServer(ros::NodeHandle _nh)
{
    nh = _nh;
    ros::TimerEvent e;
    std::thread thr(&TFServer::PubThread, this, e);
    thr.detach();
    
    pub_lock = false;
}
TFServer::~TFServer()
{
    
}
void TFServer::PubThread(const ros::TimerEvent& e)
{
    while(ros::ok)
    {
	if (!pub_lock)
	    Publish(e);
	ros::Duration(0.01).sleep();
    }
}

void TFServer::Publish(const ros::TimerEvent&)
{
    pub_lock = true;
    std::map<std::pair<std::string, std::string>, tf::Transform> tfs_copy;
    tfs_copy.clear();
    
    try
    {
	tfs_copy = tfs;
	
	for (auto tf : tfs_copy)
	{
	    tf::StampedTransform stamped_transform;
	    stamped_transform.frame_id_ = tf.first.first;
	    stamped_transform.child_frame_id_ = tf.first.second;
	    stamped_transform.setOrigin(tf.second.getOrigin());
	    stamped_transform.setRotation(tf.second.getRotation());
	    stamped_transform.stamp_ = ros::Time().now();
	    tf_publisher.sendTransform(stamped_transform);
	    ros::spinOnce();
	}   
    }
    catch(...)
    {
	ROS_ERROR_STREAM("TFSERVER: CATCHED ON PUBLISH");
	pub_lock = false;
	return;
    }
    pub_lock = false;
}
void TFServer::PublishTFs()
{
    ros::TimerEvent e;
    if (pub_lock)
    {
	while (pub_lock)
	{
// 	    ROS_INFO_STREAM("TFSERVER: Waiting for !pub_lock");
	}
    }    
    Publish(e);
}

void TFServer::set(std::string parent, std::string child, Eigen::Vector3f vec, Eigen::Quaternionf quat)
{
    tf::Transform transform;
    tf::Vector3 _vec(vec.x(), vec.y(), vec.z());
    tf::Quaternion _quat(quat.x(), quat.y(), quat.z(), quat.w());
    transform.setOrigin(_vec);
    transform.setRotation(_quat);
    set(parent, child, transform);
}
void TFServer::set(std::string parent, std::string child, geometry_msgs::Pose pose)
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
    set(parent, child, transform);
}
void TFServer::set(std::string parent, std::string child, tf::Transform transform)
{
    std::pair<std::string, std::string> frames;
    frames.first = parent;
    frames.second = child;
    tfs[frames] = transform;
    ros::TimerEvent e;
    if (!pub_lock)
	Publish(e);
}
bool TFServer::get(std::string parent, std::string child, tf::Transform& transform)
{
    tf::StampedTransform stamped_transform;
    try
    {
	tf_listener.waitForTransform(parent, child, ros::Time(0), ros::Duration(5));
	tf_listener.lookupTransform(parent, child, ros::Time(0), stamped_transform);
	
	transform.setOrigin(stamped_transform.getOrigin());
	transform.setRotation(stamped_transform.getRotation());
	
	return true;
    }
    catch (tf::TransformException ex)
    {
	ROS_INFO_STREAM("Can't get transform " << parent << "->" << child << " cause: " << ex.what());
	return false;
    }
}
bool TFServer::get(std::string parent, std::string child, Eigen::Vector3f& vec, Eigen::Quaternionf& quat)
{
    tf::Transform transform;
    if (get(parent, child, transform))
    {
	vec.x() = transform.getOrigin().x();
	vec.y() = transform.getOrigin().y();
	vec.z() = transform.getOrigin().z();
	
	quat.x() = transform.getRotation().x();
	quat.y() = transform.getRotation().y();
	quat.z() = transform.getRotation().z();
	quat.w() = transform.getRotation().w();
	return true;
    }
    else 
    {
	return false;
    }
}
bool TFServer::get(std::string parent, std::string child, geometry_msgs::Pose& pose)
{
    tf::Transform transform;
    if (get(parent, child, transform))
    {
	pose.position.x = transform.getOrigin().x();
	pose.position.y = transform.getOrigin().y();
	pose.position.z = transform.getOrigin().z();
	
	pose.orientation.x = transform.getRotation().x();
	pose.orientation.y = transform.getRotation().y();
	pose.orientation.z = transform.getRotation().z();
	pose.orientation.w = transform.getRotation().w();
	return true;
    }
    else 
    {
	return false;
    }
}
bool TFServer::getFromMap(std::string parent, std::string child, tf::Transform& pose)
{
    std::pair<std::string, std::string> frames;
    frames.first = parent;
    frames.second = child;
    if (tfs.find(frames) != tfs.end())
    {
	try
	{
	    pose = tfs.at(frames);
	}
	catch (...)
	{
	    ROS_ERROR_STREAM("TFSERVER: Execption was cought while getting from map " << parent << "->" << child);
	    return false;
	}
	return true;
    }
    else
    {
	return false;
    }
}
bool TFServer::getFromMap(std::string parent, std::string child, geometry_msgs::Pose& pose)
{
    tf::Transform transform;
    if (getFromMap(parent, child, transform))
    {
	pose.position.x = transform.getOrigin().x();
	pose.position.y = transform.getOrigin().y();
	pose.position.z = transform.getOrigin().z();
	
	pose.orientation.x = transform.getRotation().x();
	pose.orientation.y = transform.getRotation().y();
	pose.orientation.z = transform.getRotation().z();
	pose.orientation.w = transform.getRotation().w();
	return true;
    }
    else
    {
	return false;
    }
    
}
bool TFServer::getFromMap(std::string parent, std::string child, Eigen::Vector3f& vec, Eigen::Quaternionf& quat)
{
    tf::Transform transform;
    if (getFromMap(parent, child, transform))
    {
	vec.x() = transform.getOrigin().x();
	vec.y() = transform.getOrigin().y();
	vec.z() = transform.getOrigin().z();
	
	quat.x() = transform.getRotation().x();
	quat.y() = transform.getRotation().y();
	quat.z() = transform.getRotation().z();
	quat.w() = transform.getRotation().w();
	return true;
    }
    else 
    {
	return false;
    }
}
std::map< std::pair< std::string, std::string >, tf::Transform > TFServer::getMap()
{
    return tfs;
}
void TFServer::remove(std::string parent, std::string child)
{
    std::pair<std::string, std::string> frames;
    frames.first = parent;
    frames.second = child;
    if (tfs.find(frames) != tfs.end())
    {
	try
	{
	    tfs.erase(frames);
	}
	catch (...)
	{
	    ROS_ERROR_STREAM("TFSERVER: Execption was cought while removing " << parent << "->" << child);
	    return;
	}
	ros::TimerEvent e;
	if (!pub_lock)
	    Publish(e);
    }
}
void TFServer::start()
{
//     frame_timer.start();
}
void TFServer::stop()
{
//     frame_timer.stop();
}









