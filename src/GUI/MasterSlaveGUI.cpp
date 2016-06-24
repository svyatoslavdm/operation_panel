#include <operation_panel/GUI/MasterSlaveGUI.h>
#include <boost/graph/graph_concepts.hpp>
#include <qt4/QtCore/qnamespace.h>

namespace rviz
{

GuiMasterSlave::GuiMasterSlave(ActionMasterSlaveClient* _client)
{
    AddWidgets();
    ConnectSignals();
    DisableButtons();
    client = _client;
}

void GuiMasterSlave::SetClient(ActionMasterSlaveClient* _client)
{
    client = _client;
}

void GuiMasterSlave::AddWidgets()
{
    enableCheckBox = new QCheckBox("Activate");   
        
    parametersGroupBox = new QGroupBox("Operation parameters");
    
    nullPositionCheckBox = new QCheckBox("Send master robot to null position before master-slave mode start");
    synchronizeCheckBox = new QCheckBox("Synchronize gripper orientation");

    armLatchModeCheckBox = new QCheckBox("Arm latch mode");
    gripperLatchModeCheckBox = new QCheckBox("Gripper latch mode");

    QHBoxLayout* postionScaleLayout = new QHBoxLayout;
    QLabel* positionScaleLabel = new QLabel("Slave position scale");
    positionScaleSpinbox = new QDoubleSpinBox;
    positionScaleSpinbox->setRange(0.1, 2.5);
    positionScaleSpinbox->setSingleStep(0.1);
    positionScaleSpinbox->setValue(1.0);
    postionScaleLayout->addWidget(positionScaleLabel);
    postionScaleLayout->addWidget(positionScaleSpinbox);

    QHBoxLayout* effortScaleLayout = new QHBoxLayout;
    QLabel* effortScaleLabel = new QLabel("Slave effort scale");
    effortScaleSpinbox = new QDoubleSpinBox;
    effortScaleSpinbox->setRange(0.1, 2.5);
    effortScaleSpinbox->setSingleStep(0.1);
    effortScaleSpinbox->setValue(1.0);
    effortScaleLayout->addWidget(effortScaleLabel);
    effortScaleLayout->addWidget(effortScaleSpinbox);

    QVBoxLayout* parametersLayout = new QVBoxLayout;
    parametersLayout->addWidget(nullPositionCheckBox);
    parametersLayout->addWidget(synchronizeCheckBox);
    parametersLayout->addWidget(armLatchModeCheckBox);
    parametersLayout->addWidget(gripperLatchModeCheckBox);
    parametersLayout->addLayout(postionScaleLayout);
    parametersLayout->addLayout(effortScaleLayout);
    parametersLayout->setAlignment(Qt::AlignTop);
    parametersGroupBox->setLayout(parametersLayout);
    
    statusLabel = new QLabel("");

    QHBoxLayout* buttonsLayout = new QHBoxLayout;
    startButton = new QPushButton("Start");
    stopButton = new QPushButton("Stop");
    buttonsLayout->addWidget(startButton);
    buttonsLayout->addWidget(stopButton);

    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addWidget(enableCheckBox);
    mainLayout->addWidget(parametersGroupBox);
    mainLayout->addWidget(statusLabel);
    mainLayout->addLayout(buttonsLayout);    
    mainLayout->setAlignment(Qt::AlignTop);

    this->addLayout(mainLayout);
    this->setAlignment(Qt::AlignTop);    
}

void GuiMasterSlave::ConnectSignals()
{
    connect(enableCheckBox, SIGNAL (clicked()), this, SLOT (EnableCheckBox_stateChanged()));
    connect(nullPositionCheckBox, SIGNAL (clicked()), this, SLOT (NullPositionCheckBox_stateChanged()));
    connect(synchronizeCheckBox, SIGNAL (clicked()), this, SLOT (SynchronizeCheckBox_stateChanged()));
    connect(startButton, SIGNAL (clicked()), this, SLOT (StartButton_clicked()));    
    connect(stopButton, SIGNAL (clicked()), this, SLOT (StopButton_clicked()));
}

void GuiMasterSlave::DisableButtons()
{
    parametersGroupBox->setEnabled(false);

    nullPositionCheckBox->setCheckState(Qt::Checked);
    synchronizeCheckBox->setCheckState(Qt::Unchecked);
    armLatchModeCheckBox->setCheckState(Qt::Unchecked);
    gripperLatchModeCheckBox->setCheckState(Qt::Unchecked);
    positionScaleSpinbox->setValue(1.0);
    effortScaleSpinbox->setValue(1.0);

    startButton->setEnabled(false);
    stopButton->setEnabled(false);    
    statusLabel->clear();
}

void GuiMasterSlave::EnableCheckBox_stateChanged()
{
    if (enableCheckBox->checkState() == Qt::Checked)
    {
        parametersGroupBox->setEnabled(true);
	startButton->setEnabled(true);
	stopButton->setEnabled(false);
    }
    else
    {
	DisableButtons();
    }
}

void GuiMasterSlave::NullPositionCheckBox_stateChanged()
{
    goal.start_with_null_position = (nullPositionCheckBox->checkState() == Qt::Checked);
}

void GuiMasterSlave::SynchronizeCheckBox_stateChanged()
{
    goal.synchronize_orientation = (synchronizeCheckBox->checkState() == Qt::Checked);
}

void GuiMasterSlave::StartButton_clicked()
{
    NullPositionCheckBox_stateChanged();
    SynchronizeCheckBox_stateChanged();
    
    goal.master_auto_stop = (armLatchModeCheckBox->checkState() == Qt::Checked);
    goal.gripper_latch_mode = (gripperLatchModeCheckBox->checkState() == Qt::Checked);
    goal.slave_position_scale = positionScaleSpinbox->value();
    goal.slave_effort_scale = effortScaleSpinbox->value();

    try
    {
      client->Send(goal);
      startButton->setEnabled(false);
      parametersGroupBox->setEnabled(false);
      stopButton->setEnabled(true);
    }
    catch(...)
    {
      statusLabel->setText("Unknown error");
    }
}

void GuiMasterSlave::StopButton_clicked()
{
    stopButton->setEnabled(false);
    client->Abort();
}

void GuiMasterSlave::SetActionState(const std::string& _state)
{
    QString stateString = QString(_state.c_str());
    statusLabel->setText(stateString);
}

void GuiMasterSlave::SetActionDone(const std::string& _returnCode)
{
    statusLabel->setText(QString(_returnCode.c_str()));
    startButton->setEnabled(true);
    parametersGroupBox->setEnabled(true);
}

} //end namespace rviz
