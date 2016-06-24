#ifndef OPEN_BAG_PANEL_H
#define OPEN_BAG_PANEL_H

#include <sstream>
#include <fstream>
#include <iostream>
#include <string.h>
#include <thread>

#include <ros/ros.h>

#include <rviz/panel.h>
#include <stdio.h>

#include <operation_panel/GUI/OpenBagGUI.h>
#include <operation_panel/Facades/OpenBagFacade.h>

#include <QShowEvent>

namespace rviz
{

class OpenBagPanel: public rviz::Panel
{

Q_OBJECT
public:

    OpenBagPanel( QWidget* parent = 0 );

    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;
    
private:
    ros::NodeHandle nh;
   
protected:

};


} //end namespace rviz

#endif //OPEN_BAG_PANEL_H
