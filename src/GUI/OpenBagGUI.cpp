#include "operation_panel/GUI/OpenBagGUI.h"

namespace rviz
{
    
OpenBagGUI::OpenBagGUI()
{    
    AddWidgets();
    DisableButtons();
    ConnectSignals();
}

void OpenBagGUI::SetFacade(OpenBagFacade* openBagFacade_)
{
    openBagFacade = openBagFacade_;
}

void OpenBagGUI::AddWidgets()
{
    ActivCheckBox = new QCheckBox("Activate");   
    GoToViewpointButton = new QPushButton("Go to viewpoint");
    MarkersPosesCheckBox = new QCheckBox("Markers' poses is correct");    
    PlanTrajectoryButton = new QPushButton("Plan trajectory");
    ExecuteTrajectoryButton = new QPushButton("Execute trajectory");
    
    this->addWidget(ActivCheckBox);
    this->addWidget(GoToViewpointButton);
    this->addWidget(MarkersPosesCheckBox);
    this->addWidget(PlanTrajectoryButton);
    this->addWidget(ExecuteTrajectoryButton);    
    this->setAlignment(Qt::AlignTop);
}

void OpenBagGUI::DisableButtons()
{
    ActivCheckBox->setCheckState(Qt::Unchecked);

    ActivCheckBox->setEnabled(true);    
    GoToViewpointButton->setEnabled(false);
    MarkersPosesCheckBox->setEnabled(false);
    PlanTrajectoryButton->setEnabled(false);
    ExecuteTrajectoryButton->setEnabled(false);
}

void OpenBagGUI::ConnectSignals()
{
    connect(ActivCheckBox, SIGNAL (clicked()), this, SLOT (ActivCheckBox_clicked()));
    connect(GoToViewpointButton, SIGNAL (clicked()), this, SLOT (GoToViewpointButton_clicked()));
    connect(MarkersPosesCheckBox, SIGNAL (clicked()), this, SLOT (MarkersPosesCheckBox_clicked()));
    connect(PlanTrajectoryButton, SIGNAL (clicked()), this, SLOT (PlanTrajectoryButtonButton_clicked()));
    connect(ExecuteTrajectoryButton, SIGNAL (clicked()), this, SLOT (ExecuteTrajectoryButtonButton_clicked()));    
}

void OpenBagGUI::ActivCheckBox_clicked()
{
    if(ActivCheckBox->checkState() == Qt::Checked)
    {
	GoToViewpointButton->setEnabled(true);
	openBagFacade->CreateTF();
	openBagFacade->CreateMarkers();
    }
    
    if(ActivCheckBox->checkState() == Qt::Unchecked)
    {
	GoToViewpointButton->setEnabled(false);
	MarkersPosesCheckBox->setCheckState(Qt::Unchecked);
	MarkersPosesCheckBox->setEnabled(false);
	PlanTrajectoryButton->setEnabled(false);
	ExecuteTrajectoryButton->setEnabled(false);
	
	openBagFacade->DeleteTF();
	openBagFacade->DeleteMarkers();
	openBagFacade->DeleteIntMarkers();
	openBagFacade->DeleteTrajectory();
    }    
}

void OpenBagGUI::GoToViewpointButton_clicked()
{ 
    openBagFacade->GoToViewpoint();
    std::cout<<"0"<<std::endl;
}

void OpenBagGUI::GoToViewpoint_done()
{
    GoToViewpointButton->setEnabled(false);
    MarkersPosesCheckBox->setCheckState(Qt::Unchecked);
    MarkersPosesCheckBox->setEnabled(true); 

    openBagFacade->CreateTF();
    openBagFacade->CreateMarkers();
    openBagFacade->CreateIntMarkers();
}

void OpenBagGUI::MarkersPosesCheckBox_clicked() 
{
    if(MarkersPosesCheckBox->checkState() == Qt::Checked)
    {
	PlanTrajectoryButton->setEnabled(true);
	
	openBagFacade->DeleteIntMarkers();
    }    
    if(MarkersPosesCheckBox->checkState() != Qt::Checked)
    {
	PlanTrajectoryButton->setEnabled(false);
	ExecuteTrajectoryButton->setEnabled(false);
	
	openBagFacade->CreateIntMarkers();
	openBagFacade->DeleteTrajectory();
    }
}

void OpenBagGUI::PlanTrajectoryButtonButton_clicked() 
{
    MarkersPosesCheckBox->setEnabled(false);
    PlanTrajectoryButton->setEnabled(false);  
    openBagFacade->PlanTrajectory();
} 

void OpenBagGUI::PlanTrajectory_done()
{    
    MarkersPosesCheckBox->setEnabled(true);
    ExecuteTrajectoryButton->setEnabled(true); 
}

void OpenBagGUI::ExecuteTrajectoryButtonButton_clicked() 
{    
    MarkersPosesCheckBox->setCheckState(Qt::Unchecked);
    MarkersPosesCheckBox->setEnabled(false);
    ExecuteTrajectoryButton->setEnabled(false); 
    
    openBagFacade->DeleteTrajectory();
    openBagFacade->ExecutTrajectory();
}

} //end namespace rviz