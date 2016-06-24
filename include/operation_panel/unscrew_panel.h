#ifndef UNSCREW_PANEL_H
#define UNSCREW_PANEL_H

#include <sstream>
#include <fstream>
#include <iostream>
#include <string.h>
#include <thread>

#include <ros/ros.h>

#include <rviz/panel.h>
#include <stdio.h>

#include <operation_panel/GUI/UnscrewGUI.h>
#include <operation_panel/Facades/UnscrewFacade.h>
#include <operation_panel/ActionClients/ActionUnscrewClient.h>

#include <QShowEvent>

namespace rviz
{

class UnscrewPanel: public rviz::Panel
{

Q_OBJECT
public:

    UnscrewPanel( QWidget* parent = 0 );

    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;
    
private:
    ros::NodeHandle nh;
    
    UnscrewFacade* unscrewFacade;
    GuiUnscrew* unscrewGUI;    
    ActionUnscrewClient* unscrewClient;
         
    
protected:

};
//-------------------------------------------------------------------------------------------------------------------------------------------------------------   

} //end namespace rviz

#endif 
