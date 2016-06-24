#ifndef MASTER_SLAVE_PANEL_H
#define MASTER_SLAVE_PANEL_H

#include <sstream>
#include <fstream>
#include <iostream>
#include <string.h>
#include <thread>

#include <ros/ros.h>

#include <rviz/panel.h>
#include <stdio.h>

#include <operation_panel/GUI/MasterSlaveGUI.h>
#include <operation_panel/ActionClients/ActionMasterSlaveClient.h>

#include <QShowEvent>

namespace rviz
{

class MasterSlavePanel: public rviz::Panel
{

Q_OBJECT
public:

    MasterSlavePanel( QWidget* parent = 0 );

    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;
    
private:
    ros::NodeHandle nh;
    
    GuiMasterSlave* masterSlaveGUI;
    ActionMasterSlaveClient* masterSlaveClient;
    
    
    
protected:

};
//-------------------------------------------------------------------------------------------------------------------------------------------------------------   

} //end namespace rviz

#endif 
