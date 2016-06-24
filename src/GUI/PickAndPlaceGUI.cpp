#include <operation_panel/GUI/PickAndPlaceGUI.h>

namespace rviz
{

GuiPickAndPlace::GuiPickAndPlace(PickAndPlaceFacade* _facade)
{
    AddWidgets();
    DisableButtons();
    ConnectSignals();
    deactivate();
    facade = _facade;
};
void GuiPickAndPlace::SetFacade(PickAndPlaceFacade* _facade)
{
    facade = _facade;
}

void GuiPickAndPlace::DetectionRadioButton_clicked()
{
    if (ManualDetectionModeRadio->isChecked())
    {
	facade->SetObjectDetection(false);
    }
    else if (AutoDetectionModeRadio->isChecked())
    {
	facade->SetObjectDetection(true);
    }
}
void GuiPickAndPlace::ClearOctomapButton_clicked()
{
    facade->ClearOctomap();
}
void GuiPickAndPlace::DetectButton_clicked()
{
    facade->SetObject();
    AdhesionCheckBox->setEnabled(true);
    AdhesionCheckBox_stateChanged();
}

void GuiPickAndPlace::AdhesionCheckBox_stateChanged()
{
    if (AdhesionCheckBox->checkState())
    {
	facade->SetObjectAdhesion(true);
    }
    else 
    {
	facade->SetObjectAdhesion(false);
    }
}
void GuiPickAndPlace::StartPoseCheckBox_stateChanged()
{
    if (StartPoseCheckBox->checkState())
    {
	facade->SetStartPose(true);
    }
    else 
    {
	facade->SetStartPose(false);
    }
    checkCheckBoxes();
}
void GuiPickAndPlace::FinishPoseCheckBox_stateChanged()
{
    if (FinishPoseCheckBox->checkState())
    {
	facade->SetFinishPose(true);
    }
    else 
    {
	facade->SetFinishPose(false);
    }
    checkCheckBoxes();
}

void GuiPickAndPlace::GraspRadioButton_clicked()
{
    if (ManualGraspModeRadio->isChecked())
    {
	facade->SetGraspChoser(false);
    }
    else if (AutoGraspModeRadio->isChecked())
    {
	facade->SetGraspChoser(true);
    }
}

void GuiPickAndPlace::PlanButton_clicked()
{
    PlanButton->setEnabled(false);
    StopPlanButton->setEnabled(true);
    StatusLabel->setText(QString().fromStdString("Status: Planning ..."));
    
    facade->Plan();
}
void GuiPickAndPlace::StopPlanButton_clicked()
{
    PlanButton->setEnabled(true);
    StopPlanButton->setEnabled(false);
    facade->StopPlan();
}

void GuiPickAndPlace::CorrectTrajectoryCheckBox_stateChanged()
{
    if (CorrectTrajectoryCheckBox->checkState())
    {
	facade->SetCorrectTrajectory(true);
    }
    else 
    {
	facade->SetCorrectTrajectory(false);
    }
    checkCheckBoxes();
}
void GuiPickAndPlace::ExecuteButton_clicked()
{
    if (facade->IsPlanned())
    {
	{  // GUI work
	    StartPoseCheckBox->setCheckState(Qt::Unchecked);
	    FinishPoseCheckBox->setCheckState(Qt::Unchecked);
	    CorrectTrajectoryCheckBox->setCheckState(Qt::Unchecked);
	    CorrectTrajectoryCheckBox->setEnabled(false);
	    StartPoseCheckBox->setEnabled(false);
	    FinishPoseCheckBox->setEnabled(false);
	    checkCheckBoxes();
	}	    
	StatusLabel->setText(QString().fromStdString("Status: Executing ..."));
	
	facade->Execute();
    }
}
void GuiPickAndPlace::OpenStaticGripperButton_clicked()
{
    facade->OpenStaticGripper();
}
void GuiPickAndPlace::CloseStaticGripperButton_clicked()
{
    facade->CloseStaticGripper();
}

void GuiPickAndPlace::checkCheckBoxes()
{
    if (StartPoseCheckBox->checkState() && FinishPoseCheckBox->checkState())
    {
	PlanButton->setEnabled(true);
    }
    else 
    {
	PlanButton->setEnabled(false);
    }
    
    if (StartPoseCheckBox->checkState() && FinishPoseCheckBox->checkState() && CorrectTrajectoryCheckBox->checkState())
    {
	if (facade->IsPlanned())
	{
	    ExecuteButton->setEnabled(true);  
	}
    }
    else 
    {
	ExecuteButton->setEnabled(false);
    }
}
void GuiPickAndPlace::EnableCheckBox_stateChanged()
{
    if (!enableCheckBox->checkState())
    {
	deactivate();
	facade->Clear();
    }
    else 
    {
	activate();
	// 	facade->SetCorrectTrajectory(false);
    }

}

void GuiPickAndPlace::OnDetected()
{
    StartPoseCheckBox->setCheckState(Qt::Unchecked);
    FinishPoseCheckBox->setCheckState(Qt::Unchecked);
    StartPoseCheckBox->setEnabled(true);
    FinishPoseCheckBox->setEnabled(true);
    
    ManualGraspModeRadio->setEnabled(true);
    AutoGraspModeRadio->setEnabled(true);
}
void GuiPickAndPlace::OnPlanned()
{
    if (facade->IsPlanned())
    {
	CorrectTrajectoryCheckBox->setEnabled(true);
	StatusLabel->setText(QString().fromStdString("Status: Succesfully planned"));
    }
    else 
    {
	StatusLabel->setText(QString().fromStdString("Status: Plan failed"));
    }
    PlanButton->setEnabled(true);
    StopPlanButton->setEnabled(false);
}
void GuiPickAndPlace::OnExecuted()
{
    StatusLabel->setText(QString().fromStdString("Status: Path execution done!"));
    StartPoseCheckBox->setEnabled(true);
    FinishPoseCheckBox->setEnabled(true);
}
void GuiPickAndPlace::activate()
{
    ManualDetectionModeRadio->setEnabled(true);
    AutoDetectionModeRadio->setEnabled(true);
    DetectButton->setEnabled(true);      
    ClearOctomapButton->setEnabled(true);
    OpenStaticGripperButton->setEnabled(true);
    CloseStaticGripperButton->setEnabled(true);
    
    AutoGraspModeRadio->setChecked(true);
}
void GuiPickAndPlace::deactivate()
{
    ManualDetectionModeRadio->setEnabled(false);
    AutoDetectionModeRadio->setEnabled(false);
    DetectButton->setEnabled(false);
    ClearOctomapButton->setEnabled(false);
    AdhesionCheckBox->setEnabled(false);
    StartPoseCheckBox->setEnabled(false);
    FinishPoseCheckBox->setEnabled(false);
    
    ManualGraspModeRadio->setEnabled(false);
    AutoGraspModeRadio->setEnabled(false);
    
    PlanButton->setEnabled(false);
    StopPlanButton->setEnabled(false);
    CorrectTrajectoryCheckBox->setEnabled(false);   
    ExecuteButton->setEnabled(false);
    
    OpenStaticGripperButton->setEnabled(false);
    CloseStaticGripperButton->setEnabled(false);
    
}

void GuiPickAndPlace::AddWidgets()
{
    SelectedObjectLabel = new QLabel("Selected object: ");
    
    enableCheckBox = new QCheckBox("Activate"); 
    enableCheckBox->setChecked(false);
    QGroupBox* DetectionGroupBox = new QGroupBox("Detection");
    
    QVBoxLayout* DetectionLayout = new QVBoxLayout;    
    DetectionModeLabel = new QLabel("Pose detection mode:");
    ManualDetectionModeRadio = new QRadioButton("Manual detection");
    AutoDetectionModeRadio = new QRadioButton("Auto detection");
    ManualDetectionModeRadio->setChecked(true);
    DetectButton = new QPushButton("Get object");   
    
    AdhesionCheckBox = new QCheckBox("Object's adhesion");    
    AdhesionCheckBox->setCheckState(Qt::Checked);
    
    QHBoxLayout* StartPoseLayout = new QHBoxLayout;
    StartPoseLayout->addWidget( new QLabel( "Start pose is correct " ));
    StartPoseCheckBox = new QCheckBox;
    StartPoseLayout->addWidget( StartPoseCheckBox );
    
    QHBoxLayout* FinishPoseLayout = new QHBoxLayout;
    FinishPoseLayout->addWidget( new QLabel( "Finish pose is correct " ));
    FinishPoseCheckBox = new QCheckBox;
    FinishPoseLayout->addWidget( FinishPoseCheckBox );    
    
    DetectionLayout->addWidget(DetectionModeLabel);
    DetectionLayout->addWidget(ManualDetectionModeRadio);
    DetectionLayout->addWidget(AutoDetectionModeRadio);
    DetectionLayout->addWidget(DetectButton);
    DetectionLayout->addWidget(AdhesionCheckBox);
    DetectionLayout->addLayout(StartPoseLayout);
    DetectionLayout->addLayout(FinishPoseLayout);
    
    DetectionGroupBox->setLayout(DetectionLayout);

    QGroupBox* GraspGroupBox = new QGroupBox("Grasping");    
        
    QVBoxLayout* GraspLayout = new QVBoxLayout;
    GraspModeLabel = new QLabel("Grasp plan mode:");
    ManualGraspModeRadio = new QRadioButton("Manual");
    AutoGraspModeRadio = new QRadioButton("Automatic");
    AutoGraspModeRadio->setChecked(true);
    
    ManualGraspModeRadio->setEnabled(false);
    AutoGraspModeRadio->setEnabled(false);
    
    ClearOctomapButton = new QPushButton("Clear Octomap"); 
    
    GraspLayout->addWidget(GraspModeLabel);
    GraspLayout->addWidget(ManualGraspModeRadio);
    GraspLayout->addWidget(AutoGraspModeRadio);
    GraspLayout->addWidget(ClearOctomapButton);
    GraspLayout->setAlignment(Qt::AlignTop);
    GraspGroupBox->setLayout(GraspLayout);

    QHBoxLayout* Detection_and_Grasp_layout = new QHBoxLayout;
    Detection_and_Grasp_layout->addWidget(DetectionGroupBox);
    Detection_and_Grasp_layout->addWidget(GraspGroupBox);

    PlanButton = new QPushButton("Plan trajectory");
    StopPlanButton = new QPushButton("Stop planning");        
    
    QHBoxLayout* CorrectTrajectoryLayout = new QHBoxLayout;
    CorrectTrajectoryLayout->addWidget( new QLabel( "The trajectory is correct " ));
    CorrectTrajectoryCheckBox = new QCheckBox;
    CorrectTrajectoryLayout->addWidget( CorrectTrajectoryCheckBox );
    
    ExecuteButton = new QPushButton("Execute trajectory");
    
    QHBoxLayout* StaticGripperLayout = new QHBoxLayout;
    OpenStaticGripperButton = new QPushButton("Open static gripper");
    CloseStaticGripperButton = new QPushButton("Close static gripper");
    StaticGripperLayout->addWidget(OpenStaticGripperButton);
    StaticGripperLayout->addWidget(CloseStaticGripperButton);
    
    this->setAlignment(Qt::AlignTop);  
    this->addWidget(enableCheckBox); 
    this->addWidget(SelectedObjectLabel);
    this->addLayout(Detection_and_Grasp_layout);
    this->addWidget(PlanButton);
    this->addWidget(StopPlanButton);
    this->addLayout(CorrectTrajectoryLayout);
    this->addWidget(ExecuteButton);
    StatusLabel = new QLabel("Status: Idle");
    this->addWidget(StatusLabel);
    MessageLabel = new QLabel("");
    this->addWidget(MessageLabel);
    this->addLayout(StaticGripperLayout);
}
void GuiPickAndPlace::DisableButtons()
{
    StartPoseCheckBox->setEnabled(false);
    FinishPoseCheckBox->setEnabled(false);
    CorrectTrajectoryCheckBox->setEnabled(false);
    PlanButton->setEnabled(false);
    ExecuteButton->setEnabled(false);
}
void GuiPickAndPlace::ConnectSignals()
{
    connect(enableCheckBox, SIGNAL (clicked()), this, SLOT (EnableCheckBox_stateChanged()));
    
    connect(DetectButton, SIGNAL (clicked()), this, SLOT (DetectButton_clicked()));
    connect(ClearOctomapButton, SIGNAL (clicked()), this, SLOT (ClearOctomapButton_clicked()));
    connect(PlanButton, SIGNAL (clicked()), this, SLOT (PlanButton_clicked()));
    connect(StopPlanButton, SIGNAL (clicked()), this, SLOT (StopPlanButton_clicked()));
    connect(ExecuteButton, SIGNAL (clicked()), this, SLOT (ExecuteButton_clicked()));
    connect(StartPoseCheckBox, SIGNAL (clicked()), this, SLOT (StartPoseCheckBox_stateChanged()));
    connect(FinishPoseCheckBox, SIGNAL (clicked()), this, SLOT (FinishPoseCheckBox_stateChanged()));
    connect(CorrectTrajectoryCheckBox, SIGNAL (clicked()), this, SLOT (CorrectTrajectoryCheckBox_stateChanged()));
    
    connect(ManualDetectionModeRadio, SIGNAL (clicked()), this, SLOT (DetectionRadioButton_clicked()));
    connect(AutoDetectionModeRadio, SIGNAL (clicked()), this, SLOT (DetectionRadioButton_clicked()));
    connect(AdhesionCheckBox, SIGNAL (clicked()), this, SLOT (AdhesionCheckBox_stateChanged()));
    
    connect(AutoGraspModeRadio, SIGNAL (clicked()), this, SLOT(GraspRadioButton_clicked()));
    connect(ManualGraspModeRadio, SIGNAL (clicked()), this, SLOT (GraspRadioButton_clicked()));
    
    connect(OpenStaticGripperButton, SIGNAL (clicked()), this, SLOT (OpenStaticGripperButton_clicked()));
    connect(CloseStaticGripperButton, SIGNAL (clicked()), this, SLOT (CloseStaticGripperButton_clicked()));
}
} //end namespace rviz