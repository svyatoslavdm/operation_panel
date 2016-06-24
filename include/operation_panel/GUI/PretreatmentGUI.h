#ifndef PRETREATMENT_GUI_H_
#define PRETREATMENT_GUI_H_

#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QRadioButton>
#include <QLineEdit>
#include <QPushButton>

#include <tf/tf.h>

#include <operation_panel/Facades/PretreatmentFacade.h>

namespace rviz
{
    
class PretreatmentGUI : public QGridLayout
{
Q_OBJECT

public:
    PretreatmentGUI();
    void SetFacade(PretreatmentFacade*);
    void Viewpoint_done();
    void SetBias_done();
    void Scan_done();
    void SetMass(float);
    void SetDimensions(tf::Vector3);
    
private:
    PretreatmentFacade* pretreatmentFacade;
    
    QLabel* plyFilePath;
    QLabel* stlFilePath;   
    QLabel* mLabel;
    QLabel* xSizeLabel;
    QLabel* ySizeLabel;
    QLabel* zSizeLabel;
    QLineEdit* fileNameLineEdit;
    QPushButton* enterFileNameButton;
    QPushButton* viewpointButton;
    QPushButton* setBiasButton;
    QPushButton* scanButton;
    
    void AddWidgets();
    void DisableButtons();
    void ConnectSignals();
    
private Q_SLOTS:
    void EnterFileNameButton_clicked();
    void ViewpointButton_clicked();
    void SetBiasButton_clicked();
    void ScanButton_clicked();
};

} //end namespace rviz

#endif // PRETREATMENT_GUI_H_