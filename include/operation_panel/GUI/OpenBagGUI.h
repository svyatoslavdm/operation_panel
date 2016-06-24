#ifndef OPEN_BAG_GUI_H_
#define OPEN_BAG_GUI_H_

#include <QHBoxLayout>
#include <QCheckBox>
#include <QPushButton>

#include <operation_panel/Facades/OpenBagFacade.h>

namespace rviz
{
    
class OpenBagGUI: public QVBoxLayout
{
Q_OBJECT

public:
    OpenBagGUI();     
    void SetFacade(OpenBagFacade*);
    void GoToViewpoint_done();
    void PlanTrajectory_done();

private: 
    OpenBagFacade* openBagFacade;
    
    QCheckBox* ActivCheckBox;    
    QPushButton* GoToViewpointButton;
    QCheckBox* MarkersPosesCheckBox;    
    QPushButton* PlanTrajectoryButton;
    QPushButton* ExecuteTrajectoryButton;
    
    void AddWidgets();
    void DisableButtons();
    void ConnectSignals();

    
private Q_SLOTS:
    void ActivCheckBox_clicked();
    void GoToViewpointButton_clicked();
    void MarkersPosesCheckBox_clicked();    
    void PlanTrajectoryButtonButton_clicked();    
    void ExecuteTrajectoryButtonButton_clicked();    
};

} //end namespace rviz

#endif // OPEN_BAG_GUI_H_