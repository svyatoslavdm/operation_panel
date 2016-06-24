#ifndef GUIMASTERSLAVE_H
#define GUIMASTERSLAVE_H

#include <QHBoxLayout>
#include <QCheckBox>
#include <QPushButton>
#include <QRadioButton>
#include <QLabel>
#include <QGroupBox>
#include <QSpinBox>
#include <QDoubleSpinBox>

#include <operation_panel/ActionClients/ActionMasterSlaveClient.h>

namespace rviz
{
    
class GuiMasterSlave: public QVBoxLayout
{
Q_OBJECT

public:
    
    GuiMasterSlave(ActionMasterSlaveClient* _client);
    
    void SetClient(ActionMasterSlaveClient*);
    
    void SetActionState(const std::string&);
    void SetActionDone(const std::string&);
               
private: 
    
    ActionMasterSlaveClient* client;
    ActionMasterSlaveClient::Goal goal;
    
    QGroupBox* parametersGroupBox;
    QCheckBox* enableCheckBox;
    QCheckBox* nullPositionCheckBox;
    QCheckBox* synchronizeCheckBox;
    QCheckBox* armLatchModeCheckBox;
    QCheckBox* gripperLatchModeCheckBox;
    QDoubleSpinBox* positionScaleSpinbox;
    QDoubleSpinBox* effortScaleSpinbox;

    QPushButton* startButton;   
    QPushButton* stopButton;
    QLabel* statusLabel;
                
    void AddWidgets();
    void DisableButtons();
    void ConnectSignals();    
    
    
private Q_SLOTS:
        
    void EnableCheckBox_stateChanged();
    void NullPositionCheckBox_stateChanged();
    void SynchronizeCheckBox_stateChanged();
    void StartButton_clicked();
    void StopButton_clicked();
    
};

} //end namespace rviz

#endif // GUIMASTERSLAVE_H
