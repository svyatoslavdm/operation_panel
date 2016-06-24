#include <operation_panel/master_slave_panel.h>

namespace rviz
{

MasterSlavePanel::MasterSlavePanel( QWidget* parent )
: rviz::Panel( parent )

{
    masterSlaveClient = new ActionMasterSlaveClient();
    masterSlaveGUI = new GuiMasterSlave(masterSlaveClient);
    masterSlaveClient->SetGui(masterSlaveGUI);
    setLayout(masterSlaveGUI);
}

void MasterSlavePanel::save( rviz::Config config ) const
{
    rviz::Panel::save( config );

}
void MasterSlavePanel::load( const rviz::Config& config )
{
    rviz::Panel::load( config );
}

} // end of rviz namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::MasterSlavePanel, rviz::Panel)
