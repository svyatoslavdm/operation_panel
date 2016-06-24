#ifndef GUI_PICK_AND_PLACE_H
#define GUI_PICK_AND_PLACE_H

#include <QHBoxLayout>
#include <QCheckBox>
#include <QPushButton>
#include <QRadioButton>
#include <QLabel>
#include <QGroupBox>

#include <operation_panel/Facades/PickAndPlaceFacade.h>

namespace rviz
{
    
class GuiPickAndPlace: public QVBoxLayout
{
Q_OBJECT

public:
    GuiPickAndPlace(PickAndPlaceFacade* _facade);
    
    void SetFacade(PickAndPlaceFacade*);
    
    void OnPlanned();
    
    void OnExecuted();
    
    void OnDetected();
    
    
private: 
    
    PickAndPlaceFacade* facade;
    
    QCheckBox* enableCheckBox;
    
    QCheckBox* StartPoseCheckBox;
    QCheckBox* FinishPoseCheckBox;
    QCheckBox* CorrectTrajectoryCheckBox;
    QCheckBox* AdhesionCheckBox;
    QPushButton* DetectButton;
    QPushButton* PlanButton;
    QPushButton* StopPlanButton;
    QPushButton* ExecuteButton;    
    QPushButton* OpenStaticGripperButton;
    QPushButton* CloseStaticGripperButton;    
    QPushButton* ClearOctomapButton; 
    QRadioButton* ManualDetectionModeRadio;
    QRadioButton* AutoDetectionModeRadio;
    QLabel* DetectionModeLabel;    
    QRadioButton* ManualGraspModeRadio;
    QRadioButton* AutoGraspModeRadio;
    QLabel* GraspModeLabel;    
    QLabel* StatusLabel;
    QLabel* MessageLabel;
    QLabel* SelectedObjectLabel;
    
    void AddWidgets();
    void DisableButtons();
    void ConnectSignals();
    
private Q_SLOTS:
    void EnableCheckBox_stateChanged();
    void DetectButton_clicked();
    void ClearOctomapButton_clicked();
    void PlanButton_clicked();
    void StopPlanButton_clicked();
    void ExecuteButton_clicked();
    void StartPoseCheckBox_stateChanged();
    void FinishPoseCheckBox_stateChanged();
    void AdhesionCheckBox_stateChanged();
    void CorrectTrajectoryCheckBox_stateChanged();
    void DetectionRadioButton_clicked();
    void GraspRadioButton_clicked();    
    void OpenStaticGripperButton_clicked();
    void CloseStaticGripperButton_clicked();   
    
    void activate();
    void deactivate();
    
    void checkCheckBoxes();
};

} //end namespace rviz

#endif // GUI_PICK_AND_PLACE_H