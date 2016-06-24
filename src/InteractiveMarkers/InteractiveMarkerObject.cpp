#include "operation_panel/InteractiveMarkers/InteractiveMarkerObject.h"

InteractiveMarkerObject::InteractiveMarkerObject()
{
    bb = new BoundingBox();
    adherence = false;
}

void InteractiveMarkerObject::SetSceneInformation(Scene* scene)
{
    allOperationTablePolygons = scene->GetOperationTablePolygons();    
    allOperationTableNormals = scene->GetOperationTableNormals();
    allRotaryTablePolygons = scene->GetRotaryTablePolygons();
    allRotaryTableNormals = scene->GetRotaryTableNormals();
    viceGripPolygons = scene->GetViceGripPolygons();
    viceGripNormals = scene->GetViceGripNormals();
    viceGripCenter = scene->GetViceGripCenter();
}

void InteractiveMarkerObject::SetMarkerServer(boost::shared_ptr<MarkerServer> markerSrv_)
{
    markerSrv = markerSrv_;
}

void InteractiveMarkerObject::SetAdherence(bool adherence_)
{
    adherence = adherence_;
}

void InteractiveMarkerObject::UpdateObjectInformation(std::string name)
{
    markerSrv->get(name, marker);
    bb->SetObject(marker);
    dimensions = bb->GetDimensions();
    marker = bb->Get_bb_marker();
    marker.color.a = 0.0;
    
    tfReaded = false;
}

void InteractiveMarkerObject::UpdateTablesInformation()
{
    UpdateSceneInformation("operation_table", allOperationTablePolygons, allOperationTableNormals, operationTablePolygons, operationTableNormals);
    UpdateSceneInformation("rotary_table", allRotaryTablePolygons, allRotaryTableNormals, rotaryTablePolygons, rotaryTableNormals);
}

void InteractiveMarkerObject::UpdateRotaryTableInformation()
{
    UpdateSceneInformation("rotary_table", allRotaryTablePolygons, allRotaryTableNormals, rotaryTablePolygons, rotaryTableNormals);
}

void InteractiveMarkerObject::UpdateSceneInformation(std::string name, std::vector< std::vector< tf::Vector3 > > allTablePolygons, std::vector< tf::Vector3 > allTableNormals, 
						     std::vector< std::vector< tf::Vector3 > >& tablePolygons, std::vector< tf::Vector3 >& tableNormals)
{
    tablePolygons.clear();
    tableNormals.clear();
  
    tf::Transform tr;    
    tfSrv->get("world", name, tr);
    
    tf::Vector3 L_table_w_w = tr.getOrigin();
    tf::Quaternion Q_table_w_w = tr.getRotation();
    
    std::vector<tf::Vector3> polygon;
    tf::Vector3 vec_a, vec_b, vec_c, normal;  
    
    for(int i = 0; i < allTablePolygons.size(); i++)
    {
	vec_a = allTablePolygons[i][0];
	vec_b = allTablePolygons[i][1];
	vec_c = allTablePolygons[i][2];
	normal = allTableNormals[i];

	vec_a = L_table_w_w + tf::quatRotate(Q_table_w_w, vec_a);
	vec_b = L_table_w_w + tf::quatRotate(Q_table_w_w, vec_b);
	vec_c = L_table_w_w + tf::quatRotate(Q_table_w_w, vec_c);
	normal = tf::quatRotate(Q_table_w_w, normal);
	
	if(vec_a.getZ() > 0.1 && normal.getZ() > 0.7)
	{
	    polygon.clear();
    
	    polygon.push_back(vec_a);
	    polygon.push_back(vec_b);
	    polygon.push_back(vec_c);
	    
	    tablePolygons.push_back(polygon);
	    tableNormals.push_back(normal);
	}
    }
}

visualization_msgs::InteractiveMarkerControl& InteractiveMarkerObject::MakeControl(visualization_msgs::InteractiveMarker &msg)
{
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    marker.header.frame_id = msg.name + "_control";
    marker.ns = msg.name + "_fakeMarker";
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    control.markers.push_back( marker );
    msg.controls.push_back( control );
    return msg.controls.back();
}

void InteractiveMarkerObject::MakeInteractiveMarker(std::string intMarkerName, tf::Quaternion qx_control, tf::Quaternion qy_control, tf::Quaternion qz_control)
{
    if (!tfReaded)
    {
	UpdateObjectInformation(intMarkerName);
    }
    
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "world";
    int_marker.scale = 0.2;
    int_marker.name = intMarkerName;
    
    geometry_msgs::Pose pose;    
    tfSrv->get("world", intMarkerName + "_control", pose);    
    int_marker.pose = pose;

    InteractiveMarkerObject::MakeControl(int_marker);
    
    int_marker.controls[0].interaction_mode = 7;

    visualization_msgs::InteractiveMarkerControl control;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

    control.orientation.w = qx_control.getW();
    control.orientation.x = qx_control.getX();
    control.orientation.y = qx_control.getY();
    control.orientation.z = qx_control.getZ();
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = qz_control.getW();
    control.orientation.x = qz_control.getX();
    control.orientation.y = qz_control.getY();
    control.orientation.z = qz_control.getZ();
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = qy_control.getW();
    control.orientation.x = qy_control.getX();
    control.orientation.y = qy_control.getY();
    control.orientation.z = qy_control.getZ();
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
	    
    intMarkerSrv->insert(int_marker);      
    intMarkerSrv->setCallback(int_marker.name, _processFeedBackTemp(boost::bind(&InteractiveMarkerObject::ProcessFeedback, this, _1)));
    intMarkerSrv->applyChanges();
}
    
void InteractiveMarkerObject::ProcessFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    if (!tfReaded)
    {
	WaitTF(feedback->marker_name);
    }
    
    tf::Transform tr;
    tf::Quaternion q_bb_w, q_corr;
    tf::Vector3 l_bb_w_w;
    geometry_msgs::Pose pose;
    int nofOperationTablePolygon = -1;
    int nofRotaryTablePolygon = -1;
    int nofViceGripPolygon = -1;
	    
    tf::Quaternion qx_control = tf::Quaternion (1.0, 0.0, 0.0, 1.0);
    tf::Quaternion qy_control = tf::Quaternion (0.0, 0.0, 1.0, 1.0); 
    tf::Quaternion qz_control = tf::Quaternion (0.0, 1.0, 0.0, 1.0);

    if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    {	
	pose = feedback->pose;
	
	l_bb_w_w = tf::Vector3(pose.position.x, pose.position.y, pose.position.z);
	q_bb_w = tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

	nofViceGripPolygon = CheckAbove(viceGripPolygons, viceGripNormals, l_bb_w_w); 
	nofOperationTablePolygon = CheckAbove(operationTablePolygons, operationTableNormals, l_bb_w_w);
	nofRotaryTablePolygon = CheckAbove(rotaryTablePolygons, rotaryTableNormals, l_bb_w_w);

	if(nofViceGripPolygon > -1 && adherence)
	{
	    tfSrv->get("world", "viceGripRotation", tr);
	    q_corr = CalcRotation('z', true, tf::Vector3 (1.0, 0.0, 0.0), viceGripNormals[nofViceGripPolygon]);

	    qx_control = q_corr * (tr.getRotation() * qx_control);
	    qy_control = q_corr * (tr.getRotation() * qy_control);
	    qz_control = q_corr * (tr.getRotation() * qz_control);

	    pose = ViceGripAdherence(viceGripNormals[nofViceGripPolygon], q_bb_w);
	    intMarkerSrv->setPose( feedback->marker_name, pose );		
	}

	else if(nofOperationTablePolygon > -1 && adherence)
	{
	    q_corr = CalcRotation('z', true, tf::Vector3 (1.0, 0.0, 0.0), operationTableNormals[nofOperationTablePolygon]);

	    qx_control = q_corr * qx_control;
	    qy_control = q_corr * qy_control;
	    qz_control = q_corr * qz_control;
	    
	    pose = TableAdherence(operationTablePolygons[nofOperationTablePolygon], operationTableNormals[nofOperationTablePolygon], q_bb_w, l_bb_w_w);
	    intMarkerSrv->setPose( feedback->marker_name, pose );	
	}
	
	else if(nofRotaryTablePolygon > -1 && adherence)
	{
	    q_corr = CalcRotation('z', true, tf::Vector3 (1.0, 0.0, 0.0), rotaryTableNormals[nofRotaryTablePolygon]);

	    qx_control = q_corr * qx_control;
	    qy_control = q_corr * qy_control;
	    qz_control = q_corr * qz_control;
	    
	    pose = TableAdherence(rotaryTablePolygons[nofRotaryTablePolygon], rotaryTableNormals[nofRotaryTablePolygon], q_bb_w, l_bb_w_w);
	    intMarkerSrv->setPose( feedback->marker_name, pose );
	}
	    
	tf::Transform tr;
	tr.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
	tr.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));   		
	tfSrv->set("world", feedback->marker_name + "_control", tr);
	
	MakeInteractiveMarker(feedback->marker_name, qx_control, qy_control, qz_control);
	intMarkerSrv->applyChanges();			
    }	
}

int InteractiveMarkerObject::GetQuasiZ(tf::Matrix3x3 mat)
{
    if(fabs(mat.getRow(2)[0]) >= fabs(mat.getRow(2)[1]) && fabs(mat.getRow(2)[0]) >= fabs(mat.getRow(2)[2]))
    {
	if(mat.getRow(2)[0] > 0.0)	return 0;
	else				return 1;
    }
    
    else if(fabs(mat.getRow(2)[1]) >= fabs(mat.getRow(2)[0]) && fabs(mat.getRow(2)[1]) >= fabs(mat.getRow(2)[2]))
    {
	if(mat.getRow(2)[1] > 0.0)	return 2;
	else				return 3;
    }
    
    else if(fabs(mat.getRow(2)[2]) >= fabs(mat.getRow(2)[0]) && fabs(mat.getRow(2)[2]) >= fabs(mat.getRow(2)[1]))
    {
	if(mat.getRow(2)[2] > 0.0)	return 4;
	else				return 5;
    }
	    
    return -1;
}

geometry_msgs::Pose InteractiveMarkerObject::ViceGripAdherence(tf::Vector3 viceGripNormal, tf::Quaternion q_bb_w)
{
    geometry_msgs::Pose pose;	
    tf::Vector3 l_bb_w_w;	
    tf::Matrix3x3 tau_bb_w = tf::Matrix3x3(q_bb_w);
    float dimension;

    switch(GetQuasiZ(tau_bb_w))
    {
	case 0:
	    q_bb_w = CalcRotation('x', true, tau_bb_w.getColumn(1), viceGripNormal);
	    dimension = 0.5 * dimensions.getX();
	    break;
	case 1:
	    q_bb_w = CalcRotation('x', false, tau_bb_w.getColumn(1), viceGripNormal);
	    dimension = 0.5 * dimensions.getX();
	    break;
	case 2:
	    q_bb_w = CalcRotation('y', true, tau_bb_w.getColumn(0), viceGripNormal);
	    dimension = 0.5 * dimensions.getY();
	    break;			
	case 3:
	    q_bb_w = CalcRotation('y', false, tau_bb_w.getColumn(0), viceGripNormal);
	    dimension = 0.5 * dimensions.getY();
	    break;
	case 4:
	    q_bb_w = CalcRotation('z', true, tau_bb_w.getColumn(0), viceGripNormal);
	    dimension = 0.5 * dimensions.getZ();
	    break;			
	case 5:
	    q_bb_w = CalcRotation('z', false, tau_bb_w.getColumn(0), viceGripNormal);
	    dimension = 0.5 * dimensions.getZ(); 
	    break;  
	default:
	    break;
    }

    l_bb_w_w = CalcPositionInViceGrip(viceGripCenter, viceGripNormal, dimension); 

    tf::pointTFToMsg(l_bb_w_w, pose.position);
    tf::quaternionTFToMsg(q_bb_w, pose.orientation);
		    
    return pose;
};

geometry_msgs::Pose InteractiveMarkerObject::TableAdherence(std::vector< tf::Vector3 > tablePolygon, tf::Vector3 tableNormal, tf::Quaternion q_bb_w, tf::Vector3 l_bb_w_w)
{
    geometry_msgs::Pose pose;	
    tf::Matrix3x3 tau_bb_w = tf::Matrix3x3(q_bb_w);
    float dimension;

    switch(GetQuasiZ(tau_bb_w))
    {
	case 0:
	    q_bb_w = CalcRotation('x', true, tau_bb_w.getColumn(1), tableNormal);
	    dimension = 0.5 * dimensions.getX(); 
	    break;			
	case 1:
	    q_bb_w = CalcRotation('x', false, tau_bb_w.getColumn(1), tableNormal);
	    dimension = 0.5 * dimensions.getX();
	    break;
	case 2:
	    q_bb_w = CalcRotation('y', true, tau_bb_w.getColumn(0), tableNormal);
	    dimension = 0.5 * dimensions.getY();
	    break;			
	case 3:
	    q_bb_w = CalcRotation('y', false, tau_bb_w.getColumn(0), tableNormal);
	    dimension = 0.5 * dimensions.getY();
	    break;
	case 4:
	    q_bb_w = CalcRotation('z', true, tau_bb_w.getColumn(0), tableNormal);
	    dimension = 0.5 * dimensions.getZ();
	    break;			
	case 5:
	    q_bb_w = CalcRotation('z', false, tau_bb_w.getColumn(0), tableNormal);
	    dimension = 0.5 * dimensions.getZ();
	    break;  
	default:
	    break;
    }
	
    l_bb_w_w = CalcPositionOnTable(l_bb_w_w, tablePolygon[0], tableNormal, dimension);
    
    tf::pointTFToMsg(l_bb_w_w, pose.position);
    tf::quaternionTFToMsg(q_bb_w, pose.orientation);
		    
    return pose;
}

int InteractiveMarkerObject::CheckAbove(std::vector< std::vector< tf::Vector3 > > polygons, std::vector< tf::Vector3 > normals, tf::Vector3 l_BB_w_w)
{
    tf::Vector3 l_a_w_w, l_b_w_w, l_c_w_w, l_ab_w_w, l_ac_w_w, l_BBa_w_w, l_BBb_w_w, l_BBc_w_w;
    float sABC, sAOB, sBOC, sCOA;
    int nofPolygon = -1;

    for(int i = 0; i < polygons.size(); i++)
    {
	l_BB_w_w = GetPointOfIntersectionStraightWithPlane(l_BB_w_w, tf::Vector3(0.0, 0.0, 1.0), polygons[i][0], normals[i]);
	
	l_a_w_w = polygons[i][0];
	l_b_w_w = polygons[i][1];
	l_c_w_w = polygons[i][2];
	
	l_ab_w_w = l_b_w_w - l_a_w_w;
	l_ac_w_w = l_c_w_w - l_a_w_w;
	
	l_BBa_w_w = l_a_w_w - l_BB_w_w; 
	l_BBb_w_w = l_b_w_w - l_BB_w_w; 
	l_BBc_w_w = l_c_w_w - l_BB_w_w; 
	
	sABC = (l_ab_w_w.cross(l_ac_w_w)).length() / 2.0;
	sAOB = (l_BBa_w_w.cross(l_BBb_w_w)).length() / 2.0;
	sBOC = (l_BBb_w_w.cross(l_BBc_w_w)).length() / 2.0;
	sCOA = (l_BBc_w_w.cross(l_BBa_w_w)).length() / 2.0;
	
	if(sAOB + sBOC + sCOA - sABC < 0.001)
	{
	    nofPolygon = i;
	    break;
	}
    }
    return nofPolygon;
};
    
tf::Vector3 InteractiveMarkerObject::GetPointOfIntersectionStraightWithPlane(tf::Vector3 pointOfStraight, tf::Vector3 guideOfStraight, tf::Vector3 pointOfPlane, tf::Vector3 normalOfPlane)
{
    tf::Vector3 pointOfIntersection;

    float t = (normalOfPlane.getX() * (pointOfPlane.getX() - pointOfStraight.getX()) + 
		    normalOfPlane.getY() * (pointOfPlane.getY() - pointOfStraight.getY()) + 
		    normalOfPlane.getZ() * (pointOfPlane.getZ() - pointOfStraight.getZ())) / 
		    (normalOfPlane.getX() * guideOfStraight.getX() + normalOfPlane.getY() * guideOfStraight.getY() + 
		    normalOfPlane.getZ() * guideOfStraight.getZ());
	
    pointOfIntersection = pointOfStraight + t * guideOfStraight;

    return pointOfIntersection;
}
    
tf::Quaternion InteractiveMarkerObject::CalcRotation(char axis, bool positive, tf::Vector3 vec, tf::Vector3 normal)
{
    tf::Quaternion qCalk_bb_w;
    tf::Matrix3x3 tauCalk_w_bb;	
    tf::Vector3 a, b, c;

    if(positive)	a = normal;
    else		a = - normal;

    b = vec;	
    c = a.cross(b);
    b = c.cross(a);

    switch (axis)
    {
	case 'x':
	    tauCalk_w_bb.setValue(a.getX(), a.getY(), a.getZ(), b.getX(), b.getY(), b.getZ(), c.getX(), c.getY(), c.getZ());
	    break;
	case 'y':
	    tauCalk_w_bb.setValue(b.getX(), b.getY(), b.getZ(), a.getX(), a.getY(), a.getZ(), - c.getX(), - c.getY(), - c.getZ());
	    break;
	case 'z':
	    tauCalk_w_bb.setValue(b.getX(), b.getY(), b.getZ(), c.getX(), c.getY(), c.getZ(), a.getX(), a.getY(), a.getZ());
	    break;
    }

    tauCalk_w_bb.transpose().getRotation(qCalk_bb_w);

    return qCalk_bb_w.normalize();
}
    
tf::Vector3 InteractiveMarkerObject::CalcPositionInViceGrip(tf::Vector3 center, tf::Vector3 normalOfPolygon, float dimension)
{    
    return center + normalOfPolygon * (dimension + 0.001);
}
    
tf::Vector3 InteractiveMarkerObject::CalcPositionOnTable(tf::Vector3 l_BB_w_w, tf::Vector3 vertexOfPolygon, tf::Vector3 normalOfPolygon, float dimension)
{
    return GetPointOfIntersectionStraightWithPlane(l_BB_w_w, normalOfPolygon, vertexOfPolygon, normalOfPolygon) + normalOfPolygon * (dimension + 0.001);
}

