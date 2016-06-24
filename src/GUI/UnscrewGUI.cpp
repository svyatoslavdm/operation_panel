#include <operation_panel/GUI/UnscrewGUI.h>
#include <boost/graph/graph_concepts.hpp>
#include <qt4/QtCore/qnamespace.h>

namespace rviz
{

GuiUnscrew::GuiUnscrew(UnscrewFacade* _facade)
{
    AddWidgets();
    DisableButtons();
    ConnectSignals();
    facade = _facade;
}

void GuiUnscrew::SetFacade(UnscrewFacade* _facade)
{
    facade = _facade;
}

void GuiUnscrew::AddWidgets()
{       
    QVBoxLayout* mainLayout = new QVBoxLayout;    
    
    enableCheckBox = new QCheckBox("Activate");   
    viewPointButton = new QPushButton("Go to viewpoint");  
    
    graspMarkerCheckBox = new QCheckBox("Grasp pose is correct");
    
    parametersGroupBox = new QGroupBox("Operation parameters");    
    QVBoxLayout* parametersLayout = new QVBoxLayout; 
    
    directionCheckBox = new QCheckBox("Clockwise rotation");
    manualTuneCheckBox = new QCheckBox("Manual position tune");    
    
    QHBoxLayout* maxGraspForceLayout = new QHBoxLayout;
    
    QLabel* maxGraspForceLabel = new QLabel("Maximum Grasp Force (N)");
    maxGraspForceSpinbox = new QSpinBox;
    maxGraspForceSpinbox->setRange(1, 80);
    maxGraspForceSpinbox->setValue(40);
    maxGraspForceLayout->addWidget(maxGraspForceLabel); 
    maxGraspForceLayout->addWidget(maxGraspForceSpinbox); 
    
    QHBoxLayout* maxPullForceLayout = new QHBoxLayout;    
    QLabel* maxPullForceLabel = new QLabel("Maximum Pull Force (N)");
    maxPullForceSpinbox = new QSpinBox;
    maxPullForceSpinbox->setRange(0, 20);
    maxPullForceSpinbox->setValue(4);
    maxPullForceLayout->addWidget(maxPullForceLabel); 
    maxPullForceLayout->addWidget(maxPullForceSpinbox);     
    
    QHBoxLayout* maxAxisToleranceLayout = new QHBoxLayout;
    QLabel* maxAxisToleranceLabel = new QLabel("Maximum Axis Deviation (rad)");
    axisToleranceSpinbox = new QDoubleSpinBox;
    axisToleranceSpinbox->setRange(0, 0.5);
    axisToleranceSpinbox->setSingleStep(0.01);
    axisToleranceSpinbox->setValue(0);
    maxAxisToleranceLayout->addWidget(maxAxisToleranceLabel); 
    maxAxisToleranceLayout->addWidget(axisToleranceSpinbox); 
    
    QHBoxLayout* maxCyclesLayout = new QHBoxLayout;
    QLabel* maxCyclesLabel = new QLabel("Number of rotation cycles");
    maxCyclesSpinbox = new QSpinBox;
    maxCyclesSpinbox->setRange(-1, 100);
    maxCyclesSpinbox->setValue(0);
    maxCyclesLayout->addWidget(maxCyclesLabel); 
    maxCyclesLayout->addWidget(maxCyclesSpinbox); 
    
    QHBoxLayout* rotatePerCycleLayout = new QHBoxLayout;
    QLabel* rotatePerCycleLabel = new QLabel("Rotate per cycle (grad)");
    rotatePerCycleSpinbox = new QSpinBox;
    rotatePerCycleSpinbox->setRange(30, 360);
    rotatePerCycleSpinbox->setValue(90);
    rotatePerCycleLayout->addWidget(rotatePerCycleLabel); 
    rotatePerCycleLayout->addWidget(rotatePerCycleSpinbox);     

    statusLabel = new QLabel("");
    planButton = new QPushButton("Plan");
    executeButton = new QPushButton("Execute");
    abortButton = new QPushButton("Abort"); 
    
    placeButton = new QPushButton("Place"); 
    
    parametersLayout->addWidget(directionCheckBox);
    parametersLayout->addLayout(maxGraspForceLayout);
    parametersLayout->addLayout(maxPullForceLayout);    
    parametersLayout->addLayout(maxCyclesLayout);  
    parametersLayout->addLayout(rotatePerCycleLayout);     
    parametersLayout->addLayout(maxAxisToleranceLayout);
    parametersLayout->addWidget(manualTuneCheckBox);
    
    parametersLayout->setAlignment(Qt::AlignTop);
    parametersGroupBox->setLayout(parametersLayout);
    
    tuneGroupBox = new QGroupBox("Tune position");   
    QHBoxLayout* tuneLayout = new QHBoxLayout;
    tuneLeftButton = new QPushButton("-x");
    tuneRightButton = new QPushButton("+x");
    tuneUpButton = new QPushButton("+z"); 
    tuneDownButton = new QPushButton("-z");      
    tuneApplyCheckBox = new QCheckBox("Apply");       
    tuneLayout->addWidget(tuneLeftButton);
    tuneLayout->addWidget(tuneRightButton);
    tuneLayout->addWidget(tuneUpButton);
    tuneLayout->addWidget(tuneDownButton);
    tuneLayout->addWidget(tuneApplyCheckBox);  
    tuneLayout->setAlignment(Qt::AlignTop);
    tuneGroupBox->setLayout(tuneLayout);
    
    mainLayout->addWidget(enableCheckBox);
    mainLayout->addWidget(viewPointButton);
    mainLayout->addWidget(graspMarkerCheckBox);
    mainLayout->addWidget(parametersGroupBox);
    mainLayout->addWidget(tuneGroupBox);    
    mainLayout->addWidget(statusLabel); 
    mainLayout->addWidget(planButton);    
    mainLayout->addWidget(executeButton);
    mainLayout->addWidget(abortButton);    
    mainLayout->addWidget(placeButton);     
    
    mainLayout->setAlignment(Qt::AlignTop);

    this->addLayout(mainLayout);
    this->setAlignment(Qt::AlignTop);    
}
void GuiUnscrew::DisableButtons()
{
    viewPointButton->setEnabled(false);
    
    parametersGroupBox->setEnabled(false);
    graspMarkerCheckBox->setEnabled(false);
    graspMarkerCheckBox->setCheckState(Qt::Unchecked);
    directionCheckBox->setCheckState(Qt::Unchecked); 
    manualTuneCheckBox->setCheckState(Qt::Unchecked); 
    
    tuneGroupBox->setEnabled(false);
    
    statusLabel->clear();
    planButton->setEnabled(false);
    executeButton->setEnabled(false);
    abortButton->setEnabled(false);
    placeButton->setEnabled(false);
}
void GuiUnscrew::ConnectSignals()
{
    connect(enableCheckBox, SIGNAL (clicked()), this, SLOT (EnableCheckBox_stateChanged()));
    connect(viewPointButton, SIGNAL (clicked()), this, SLOT (ViewPointButton_clicked()));
    connect(graspMarkerCheckBox, SIGNAL (clicked()), this, SLOT (GraspMarkerCheckBox_stateChanged()));
    connect(directionCheckBox, SIGNAL (clicked()), this, SLOT (DirectionCheckBox_stateChanged()));
    connect(manualTuneCheckBox, SIGNAL (clicked()), this, SLOT (ManualTuneCheckBox_stateChanged()));    
    connect(planButton, SIGNAL (clicked()), this, SLOT (PlanButton_clicked()));
    connect(executeButton, SIGNAL (clicked()), this, SLOT (ExecuteButton_clicked()));
    connect(abortButton, SIGNAL (clicked()), this, SLOT (AbortButton_clicked()));
    connect(placeButton, SIGNAL (clicked()), this, SLOT (PlaceButton_clicked()));     
    
    connect(maxGraspForceSpinbox, SIGNAL (valueChanged(int)), this, SLOT (MaxGraspForceSpinbox_changed(int)));
    connect(maxPullForceSpinbox, SIGNAL (valueChanged(int)), this, SLOT (MaxPullForceSpinbox_changed(int)));    
    connect(axisToleranceSpinbox, SIGNAL (valueChanged(double)), this, SLOT (AxisToleranceSpinbox_changed(double)));
    connect(maxCyclesSpinbox, SIGNAL (valueChanged(int)), this, SLOT (MaxCyclesSpinbox_changed(int)));
    connect(rotatePerCycleSpinbox, SIGNAL (valueChanged(int)), this, SLOT (RotatePerCycleSpinbox_changed(int)));    
}

void GuiUnscrew::EnableCheckBox_stateChanged()
{
    if (enableCheckBox->checkState() == Qt::Checked)
    {
	viewPointButton->setEnabled(true);
	
	facade->SetGraspMarker(true); 
	graspMarkerCheckBox->setEnabled(true);	
    }
    else
    {
	DisableButtons();
	facade->SetGraspMarker(false);
    }
}

void GuiUnscrew::ViewPointButton_clicked()
{
    viewPointButton->setEnabled(false);
    statusLabel->clear();
    
    facade->StartViewPointThread();
}

void GuiUnscrew::InViewPoint()
{
    facade->SetGraspMarker(true);    
//parametersGroupBox->setEnabled(true);
    graspMarkerCheckBox->setEnabled(true);
}

void GuiUnscrew::GraspMarkerCheckBox_stateChanged()
{
    if (graspMarkerCheckBox->checkState() == Qt::Checked)
    {
	facade->SetGraspMarker(false);
	facade->SetUnscrewGrasp();
	facade->SetUnscrewAxis();
	
	parametersGroupBox->setEnabled(true);
	planButton->setEnabled(true);
	executeButton->setEnabled(false);
	abortButton->setEnabled(false);	
    }
    else
    {
	parametersGroupBox->setEnabled(false);
	planButton->setEnabled(false);
	executeButton->setEnabled(false);
	abortButton->setEnabled(false);	
	facade->SetGraspMarker(true);
    }
}

void GuiUnscrew::DirectionCheckBox_stateChanged()
{
    bool flag = (directionCheckBox->checkState() == Qt::Checked);
    facade->SetDirection(flag);
}

void GuiUnscrew::ManualTuneCheckBox_stateChanged()
{
    bool flag = (manualTuneCheckBox->checkState() == Qt::Checked);
    facade->SetManualTune(flag);    
}

void GuiUnscrew::AxisToleranceSpinbox_changed(double k)
{
    facade->SetAxisTolerance(k);
}

void GuiUnscrew::MaxGraspForceSpinbox_changed(int k)
{
    facade->SetMaxGraspForce(float(k));
}

void GuiUnscrew::MaxPullForceSpinbox_changed(int k)
{
    facade->SetMaxPullForce(float(k));
}

void GuiUnscrew::MaxCyclesSpinbox_changed(int k)
{
    facade->SetMaxCycles(k);
}

void GuiUnscrew::RotatePerCycleSpinbox_changed(int k)
{
    facade->SetRotatePerCycle(k);    
}

void GuiUnscrew::PlanButton_clicked()
{
    DirectionCheckBox_stateChanged();
    MaxGraspForceSpinbox_changed(maxGraspForceSpinbox->value());
    MaxPullForceSpinbox_changed(maxPullForceSpinbox->value());    
    AxisToleranceSpinbox_changed(axisToleranceSpinbox->value());
    MaxCyclesSpinbox_changed(maxCyclesSpinbox->value());
    RotatePerCycleSpinbox_changed(rotatePerCycleSpinbox->value());  
    ManualTuneCheckBox_stateChanged();  
    
    facade->Plan();
    
    DisableButtons();
    planButton->setEnabled(false);
    executeButton->setEnabled(false);    
    abortButton->setEnabled(true);
}

void GuiUnscrew::ExecuteButton_clicked()
{
    DirectionCheckBox_stateChanged();
    MaxGraspForceSpinbox_changed(maxGraspForceSpinbox->value());
    MaxPullForceSpinbox_changed(maxPullForceSpinbox->value());    
    AxisToleranceSpinbox_changed(axisToleranceSpinbox->value());
    MaxCyclesSpinbox_changed(maxCyclesSpinbox->value());
    RotatePerCycleSpinbox_changed(rotatePerCycleSpinbox->value());
    
    facade->Execute();
    
    DisableButtons();
    planButton->setEnabled(false);
    executeButton->setEnabled(false);    
    abortButton->setEnabled(true);
}

void GuiUnscrew::AbortButton_clicked()
{   
    facade->Abort();
}

void GuiUnscrew::PlaceButton_clicked()
{   
    facade->Place();
}

void GuiUnscrew::SetActionState(const std::string& _state)
{
    statusLabel->setText(QString(_state.c_str()));
}

void GuiUnscrew::SetReturnCode(const std::string& _returnCode)
{
    statusLabel->setText(QString(_returnCode.c_str()));
}

void GuiUnscrew::SetActionDone(int err_code, bool plan_only)
{   
    actionDone = true;
    
    if(plan_only)
    {
        //if(err_code == 0)
	{	
	  executeButton->setEnabled(true);
	}
    }
    else
    {    
	viewPointButton->setEnabled(true);
	abortButton->setEnabled(false);
	placeButton->setEnabled(true);
    }
}


} //end namespace rviz