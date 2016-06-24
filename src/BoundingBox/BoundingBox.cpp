#include "operation_panel/BoundingBox/BoundingBox.h"

BoundingBox::BoundingBox()
{};

int BoundingBox::SetObject(visualization_msgs::Marker object_marker)
{
	shapes::Mesh* object_marker_mesh = shapes::createMeshFromResource(object_marker.mesh_resource);
	shapes::ShapeMsg object_mesh_msg;
	shapes::constructMsgFromShape(object_marker_mesh, object_mesh_msg);
	object_mesh = boost::get<shape_msgs::Mesh>(object_mesh_msg);

	CreateBoundingBox();

	return 1;
};

visualization_msgs::Marker BoundingBox::Get_bb_marker()
{
	return bb_marker;
};

tf::Vector3 BoundingBox::Get_L_bb_ob_ob()
{
	tf::Vector3 L_bb_ob_ob;

	L_bb_ob_ob.setX(l_bb_obj_obj.x());
	L_bb_ob_ob.setY(l_bb_obj_obj.y());
	L_bb_ob_ob.setZ(l_bb_obj_obj.z());

	return L_bb_ob_ob;
}

tf::Quaternion BoundingBox::Get_Q_bb_ob()
{
	tf::Quaternion Q_bb_ob;

	Q_bb_ob.setX(q_bb_obj.x());
	Q_bb_ob.setY(q_bb_obj.y());
	Q_bb_ob.setZ(q_bb_obj.z());
	Q_bb_ob.setW(q_bb_obj.w());

	return Q_bb_ob;
};

tf::Vector3 BoundingBox::GetDimensions()
{
	tf::Vector3 _dimensions;

	_dimensions.setX(dimensions.x());
	_dimensions.setY(dimensions.y());
	_dimensions.setZ(dimensions.z());
	
	return _dimensions;
};

void BoundingBox::GetBBPosition()
{
	Eigen::Vector3f buff;

	for (size_t i = 0; i < object_mesh.vertices.size(); i++)
	{
		buff.x() = object_mesh.vertices[i].x;
		buff.y() = object_mesh.vertices[i].y;
		buff.z() = object_mesh.vertices[i].z;

		l_bb_obj_obj = l_bb_obj_obj + buff;
	}

	l_bb_obj_obj = l_bb_obj_obj / object_mesh.vertices.size();
};

void BoundingBox::GetBBOrientation()
{
	Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
	Eigen::Vector3f buff, x, y, z;

	for (size_t i = 0; i < object_mesh.vertices.size(); i++)
	{
		for(int j = 0; j < 3; j++)
		{
			for(int k = 0; k < 3; k++)
			{
				buff.x() = object_mesh.vertices[i].x;
				buff.y() = object_mesh.vertices[i].y;
				buff.z() = object_mesh.vertices[i].z;

				covariance(j,k) = covariance(j,k) + (buff - l_bb_obj_obj)(j) * (buff - l_bb_obj_obj)(k) / 
																			object_mesh.vertices.size();
			}
    	}
	}

	x << covariance(0,0), covariance(0,1), covariance(0,2);
	y << covariance(1,0), covariance(1,1), covariance(1,2);
	z << covariance(2,0), covariance(2,1), covariance(2,2);

	if(x.dot(y) < 0.001 && y.dot(z) < 0.001 && z.dot(x) < 0.001)
	{
		x = x / x.norm();
		y = y / y.norm();
		z = x.cross(y);
		
		tau_bb_obj(0,0) = x(0);	tau_bb_obj(0,1) = x(1);		tau_bb_obj(0,2) = x(2);
		tau_bb_obj(1,0) = y(0);	tau_bb_obj(1,1) = y(1);		tau_bb_obj(1,2) = y(2);
		tau_bb_obj(2,0) = z(0);	tau_bb_obj(2,1) = z(1);		tau_bb_obj(2,2) = z(2);
	}
	else
	{
		Eigen::EigenSolver<Eigen::MatrixXf> es;
		es.compute(covariance);	
		
		for(int i = 0; i < 3; i++)
		{
			for(int j = 0; j < 3; j++)
			{
				tau_bb_obj(i,j) = es.eigenvectors().col(i)(j).real();
			}
		}
 
		x << tau_bb_obj(0,0), tau_bb_obj(0,1), tau_bb_obj(0,2);
		y << tau_bb_obj(1,0), tau_bb_obj(1,1), tau_bb_obj(1,2);
		z = x.cross(y);        
		tau_bb_obj(2,0) = z(0); tau_bb_obj(2,1) = z(1); tau_bb_obj(2,2) = z(2);
	}
};

void BoundingBox::CorrBBOrientation()
{
	Eigen::Vector3f oZ = Eigen::Vector3f::UnitZ();
	Eigen::Vector3f oz;
	oz.x() = tau_bb_obj(2,0);
	oz.y() = tau_bb_obj(2,1);
	oz.z() = tau_bb_obj(2,2);

	tau_bb_obj = Eigen::Quaternionf().setFromTwoVectors(oZ,oz).toRotationMatrix() * tau_bb_obj;

	q_bb_obj = tau_bb_obj;
};

void BoundingBox::CorrBBPosition()
{
	Eigen::Vector3f buff, l_corr_bb_bb;
	float xSize, ySize, zSize, _xSize, _ySize, _zSize;  

	xSize = ySize = zSize = _xSize = _ySize = _zSize = 0.0;

	for (size_t i = 0; i < object_mesh.vertices.size(); i++)
	{
		buff.x() = object_mesh.vertices[i].x;
		buff.y() = object_mesh.vertices[i].y;
		buff.z() = object_mesh.vertices[i].z;
		
		buff = tau_bb_obj * (buff - l_bb_obj_obj);
		
		if(buff(0) > xSize)		xSize = buff(0);
		if(buff(1) > ySize)		ySize = buff(1);
		if(buff(2) > zSize)		zSize = buff(2);
		
		if(buff(0) < _xSize)	_xSize = buff(0);
		if(buff(1) < _ySize)	_ySize = buff(1);
		if(buff(2) < _zSize)	_zSize = buff(2);
	}

	l_corr_bb_bb(0) = (xSize + _xSize) / 2.0;
	l_corr_bb_bb(1) = (ySize + _ySize) / 2.0;
	l_corr_bb_bb(2) = (zSize + _zSize) / 2.0;

	xSize = xSize - (xSize + _xSize) / 2.0;
	ySize = ySize - (ySize + _ySize) / 2.0;
	zSize = zSize - (zSize + _zSize) / 2.0;

	GetBBDimensions(xSize, ySize, zSize);
		
	l_bb_obj_obj = l_bb_obj_obj + tau_bb_obj.transpose() * l_corr_bb_bb;
};

void BoundingBox::GetBBDimensions(float xSize, float ySize, float zSize)
{
	dimensions(0) = 2 * xSize;
	dimensions(1) = 2 * ySize;
	dimensions(2) = 2 * zSize;
};

void BoundingBox::CreateBBMarker()
{	
	bb_marker.action = visualization_msgs::Marker::ADD;	
	bb_marker.type = visualization_msgs::Marker::CUBE;
	bb_marker.color.r = 1.0;
	bb_marker.color.g = 1.0;
	bb_marker.color.b = 1.0;
	bb_marker.color.a = 0.5;
	bb_marker.pose.position.x = 0;
	bb_marker.pose.position.y = 0;
	bb_marker.pose.position.z = 0;
	bb_marker.pose.orientation.x = 0;
	bb_marker.pose.orientation.y = 0;
	bb_marker.pose.orientation.z = 0;
	bb_marker.pose.orientation.w = 1;
	bb_marker.scale.x = dimensions(0);
	bb_marker.scale.y = dimensions(1);
	bb_marker.scale.z = dimensions(2);
}

void BoundingBox::CreateBoundingBox()
{ 
	tau_bb_obj = Eigen::Matrix3f::Zero();
	l_bb_obj_obj = Eigen::Vector3f::Zero();

	GetBBPosition();
	GetBBOrientation();	
	CorrBBOrientation();
	CorrBBPosition();
	CreateBBMarker();
}
