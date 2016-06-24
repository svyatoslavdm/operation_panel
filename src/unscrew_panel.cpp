#include <operation_panel/unscrew_panel.h>

namespace rviz
{

UnscrewPanel::UnscrewPanel( QWidget* parent )
: rviz::Panel( parent )

{
    unscrewFacade = new UnscrewFacade(nh);     
    unscrewGUI = new GuiUnscrew(unscrewFacade);     
    unscrewFacade->SetGUI(unscrewGUI);       
    setLayout(unscrewGUI);
    unscrewClient = new ActionUnscrewClient(unscrewGUI);
    unscrewFacade->SetClient(unscrewClient);
}

void UnscrewPanel::save( rviz::Config config ) const
{
    rviz::Panel::save( config );

}
void UnscrewPanel::load( const rviz::Config& config )
{
    rviz::Panel::load( config );
}

} // end of rviz namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::UnscrewPanel, rviz::Panel)
