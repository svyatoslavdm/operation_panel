#ifndef GUIUNSCREW_H
#define GUIUNSCREW_H

#include <QHBoxLayout>
#include <QCheckBox>
#include <QPushButton>
#include <QRadioButton>
#include <QLabel>
#include <QGroupBox>
#include <QSpinBox>
#include <QDoubleSpinBox>

#include <operation_panel/Facades/UnscrewFacade.h>
#include <operation_panel/ActionClients/ActionUnscrewClient.h>

namespace rviz
{
    
class GuiUnscrew: public QVBoxLayout
{
Q_OBJECT

public:
    
    GuiUnscrew(UnscrewFacade* _facade);
    
    void SetFacade(UnscrewFacade*);
    void SetActionState(const std::string&);
    void SetReturnCode(const std::string&);
    void SetActionDone(int err_code, bool plan_only);
    
    void InViewPoint();
    
    bool actionDone;
       
private: 
    
    UnscrewFacade* facade;
    
    QGroupBox* parametersGroupBox;
    
    QCheckBox* enableCheckBox;
    QCheckBox* graspMarkerCheckBox;
    QCheckBox* directionCheckBox;
    QCheckBox* manualTuneCheckBox;
    
    QDoubleSpinBox* axisToleranceSpinbox;
    QSpinBox* maxGraspForceSpinbox;
    QSpinBox* maxPullForceSpinbox;    
    QSpinBox* maxCyclesSpinbox;
    QSpinBox* rotatePerCycleSpinbox;    
    
    QPushButton* viewPointButton;
    QPushButton* planButton;
    QPushButton* executeButton;
    QPushButton* abortButton;    
    
    QPushButton* placeButton;  
    
    QGroupBox* tuneGroupBox;    
    
    QPushButton* tuneLeftButton;
    QPushButton* tuneRightButton;
    QPushButton* tuneUpButton;
    QPushButton* tuneDownButton;
    QCheckBox*   tuneApplyCheckBox;   
    
    QLabel* statusLabel;
                
    void AddWidgets();
    void DisableButtons();
    void ConnectSignals();    
    
    
private Q_SLOTS:
    
    void EnableCheckBox_stateChanged();
    void GraspMarkerCheckBox_stateChanged();
    void DirectionCheckBox_stateChanged();
    void ManualTuneCheckBox_stateChanged();    
    
    void AxisToleranceSpinbox_changed(double);
    void MaxGraspForceSpinbox_changed(int);
    void MaxPullForceSpinbox_changed(int);    
    void MaxCyclesSpinbox_changed(int);
    void RotatePerCycleSpinbox_changed(int);    
    
    void ViewPointButton_clicked();
    void PlanButton_clicked();    
    void ExecuteButton_clicked();
    void AbortButton_clicked();  
    
    void PlaceButton_clicked();     
    
};

} //end namespace rviz

#endif // GUIUNSCREW_H