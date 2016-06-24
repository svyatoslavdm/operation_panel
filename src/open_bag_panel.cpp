#include <operation_panel/open_bag_panel.h>

namespace rviz
{

OpenBagPanel::OpenBagPanel( QWidget* parent )
: rviz::Panel( parent )

{    
    OpenBagGUI* openBagGUI = new OpenBagGUI();
    OpenBagFacade* openBagFacadeFacade = new OpenBagFacade(nh);
    openBagGUI->SetFacade(openBagFacadeFacade);
    openBagFacadeFacade->SetGui(openBagGUI);
    setLayout(openBagGUI);
}

void OpenBagPanel::save( rviz::Config config ) const
{
    rviz::Panel::save( config );

}
void OpenBagPanel::load( const rviz::Config& config )
{
    rviz::Panel::load( config );
}

} // end of rviz namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::OpenBagPanel, rviz::Panel)