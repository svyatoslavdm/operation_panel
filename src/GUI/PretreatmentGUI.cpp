#include "operation_panel/GUI/PretreatmentGUI.h"

namespace rviz
{
    
PretreatmentGUI::PretreatmentGUI()
{
    AddWidgets();
    DisableButtons();
    ConnectSignals();
}
void PretreatmentGUI::SetFacade(PretreatmentFacade* pretreatmentFacade_)
{
    pretreatmentFacade = pretreatmentFacade_;
}
void PretreatmentGUI::AddWidgets()
{    
    QLabel* fileNameLabel = new QLabel("The name of the scanned object: ");
    QLabel* plyFilePathLabel = new QLabel("PLY file's path: ");
    QLabel* stlFilePathLabel = new QLabel("STL file's path: ");
    QLabel* dimensionLabel = new QLabel("Dimensions:");
    QLabel* xLabel = new QLabel("X-axis dimension");
    QLabel* yLabel = new QLabel("Y-axis dimension");
    QLabel* zLabel = new QLabel("Z-axis dimension");
    QLabel* massLabel = new QLabel("Mass:");
    plyFilePath = new QLabel("");
    stlFilePath = new QLabel("");
    xSizeLabel = new QLabel("");
    ySizeLabel = new QLabel("");
    zSizeLabel = new QLabel("");
    mLabel = new QLabel("");
    
    fileNameLineEdit = new QLineEdit();
    
    enterFileNameButton = new QPushButton("Enter");
    viewpointButton = new QPushButton("Viewpoint");
    setBiasButton = new QPushButton("Set bias");
    scanButton = new QPushButton("Scan");
    
    plyFilePath->setAlignment(Qt::AlignHCenter);
    stlFilePath->setAlignment(Qt::AlignHCenter);
    massLabel->setAlignment(Qt::AlignHCenter);
    mLabel->setAlignment(Qt::AlignHCenter);
    dimensionLabel->setAlignment(Qt::AlignHCenter);
    xSizeLabel->setAlignment(Qt::AlignHCenter);
    ySizeLabel->setAlignment(Qt::AlignHCenter);
    zSizeLabel->setAlignment(Qt::AlignHCenter);

    fileNameLabel->setText("The <b>name</b> of the scanned object: ");
    massLabel->setText("<b>Mass:</b>");
    dimensionLabel->setText("<b>Dimensions:</b>");
    
    this->addWidget(fileNameLabel, 0, 0, 1, 2);
    this->addWidget(fileNameLineEdit, 0, 2, 1, 3);
    this->addWidget(enterFileNameButton, 0, 5, 1, 1);
    this->addWidget(plyFilePathLabel, 1, 0, 1, 1);
    this->addWidget(stlFilePathLabel, 2, 0, 1, 1);
    this->addWidget(plyFilePath, 1, 1, 1, 5);
    this->addWidget(stlFilePath, 2, 1, 1, 5);
    this->addWidget(viewpointButton, 3, 0, 1, 2, 0);
    this->addWidget(setBiasButton, 3, 2, 1, 2, 0);
    this->addWidget(scanButton, 3, 4, 1, 2, 0);
    this->addWidget(dimensionLabel, 4, 0, 1, 2);
    this->addWidget(xLabel, 5, 0, 1, 1);
    this->addWidget(yLabel, 6, 0, 1, 1);
    this->addWidget(zLabel, 7, 0, 1, 1);
    this->addWidget(xSizeLabel, 5, 1, 1, 1);
    this->addWidget(ySizeLabel, 6, 1, 1, 1);
    this->addWidget(zSizeLabel, 7, 1, 1, 1); 
    this->addWidget(massLabel, 4, 2, 1, 2);
    this->addWidget(mLabel, 5, 2, 1, 2);
    this->setAlignment(Qt::AlignTop);
}
void PretreatmentGUI::DisableButtons()
{
    viewpointButton->setEnabled(false);
    setBiasButton->setEnabled(false);
    scanButton->setEnabled(false);
}
void PretreatmentGUI::ConnectSignals()
{
    connect(enterFileNameButton, SIGNAL (clicked()), this, SLOT (EnterFileNameButton_clicked()));
    connect(viewpointButton, SIGNAL (clicked()), this, SLOT (ViewpointButton_clicked()));
    connect(setBiasButton, SIGNAL (clicked()), this, SLOT (SetBiasButton_clicked()));
    connect(scanButton, SIGNAL (clicked()), this, SLOT (ScanButton_clicked()));
}
void PretreatmentGUI::EnterFileNameButton_clicked()
{ 
    xSizeLabel->clear();
    ySizeLabel->clear();
    zSizeLabel->clear();
    mLabel->clear();
    
    std::string name;
    std::vector<std::string> pathes;
    
    name = fileNameLineEdit->text().toUtf8().constData();
    pathes = pretreatmentFacade->EnterFileNameButtonClicked(name);
    
    if(pathes.size() > 0)
    {
	if(pathes[0].length() > 68)
	{
	    int n = pathes[0].length() - 65;
	    for(int i = 0; i < n; i++)
	    {
		pathes[0].erase(0, 1);
		pathes[1].erase(0, 1);
	    }
	    pathes[0] = "..." + pathes[0];
	    pathes[1] = "..." + pathes[1];
	}
	plyFilePath->setText(QString::fromUtf8(pathes[0].c_str()));
	stlFilePath->setText(QString::fromUtf8(pathes[1].c_str()));
	
	viewpointButton->setEnabled(true);
    }
    else
    {
	name = "Name \"" + name + "\" is not correct";
	plyFilePath->setText(QString::fromUtf8(name.c_str()));
	stlFilePath->setText(QString::fromUtf8(""));
    }
}
void PretreatmentGUI::ViewpointButton_clicked()
{
    pretreatmentFacade->ViewpointButtonClicked();
}
void PretreatmentGUI::SetBiasButton_clicked()
{
    pretreatmentFacade->SetBiasButtonClicked();
}
void PretreatmentGUI::ScanButton_clicked()
{
    fileNameLineEdit->setEnabled(false);
    enterFileNameButton->setEnabled(false);
    pretreatmentFacade->ScanButtonClicked();
}
void PretreatmentGUI::Viewpoint_done()
{
    setBiasButton->setEnabled(true);
}
void PretreatmentGUI::SetBias_done()
{
    scanButton->setEnabled(true);
}
void PretreatmentGUI::Scan_done()
{
    fileNameLineEdit->clear();
    fileNameLineEdit->setEnabled(true);    
    enterFileNameButton->setEnabled(true);
    viewpointButton->setEnabled(false);
    setBiasButton->setEnabled(false);
    scanButton->setEnabled(false);
}
void PretreatmentGUI::SetDimensions(tf::Vector3 dimensions)
{
    std::string xStr = std::to_string(dimensions.getX() * 1000) + " mm";
    std::string yStr = std::to_string(dimensions.getY() * 1000) + " mm";
    std::string zStr = std::to_string(dimensions.getZ() * 1000) + " mm";
    
    xSizeLabel->setText(QString::fromUtf8(xStr.c_str())); 
    ySizeLabel->setText(QString::fromUtf8(yStr.c_str())); 
    zSizeLabel->setText(QString::fromUtf8(zStr.c_str())); 
}
void PretreatmentGUI::SetMass(float mass)
{
    std::string massStr = std::to_string(mass) + " kg";
    mLabel->setText(QString::fromUtf8(massStr.c_str()));    
}

}//end namespace rviz