/*
 * Copyright (c) 2015, MSD
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <operation_panel/operation_panel.h>

namespace rviz
{

OperationPanel::OperationPanel( QWidget* parent )
: rviz::Panel( parent )

{
    pick_and_place_facade = new PickAndPlaceFacade(nh);
    
    pick_and_place_GUI = new GuiPickAndPlace(pick_and_place_facade);

    pick_and_place_facade->SetGUI(pick_and_place_GUI);
    
    setLayout(pick_and_place_GUI);

}

void OperationPanel::save( rviz::Config config ) const
{
    rviz::Panel::save( config );

}
void OperationPanel::load( const rviz::Config& config )
{
    rviz::Panel::load( config );
}

} // end of rviz namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::OperationPanel, rviz::Panel)
