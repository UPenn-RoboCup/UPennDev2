#include "rightwindow.h"
#include "ui_rightwindow.h"
#include "osgwidget.h"

#include <QLabel>
#include <QGridLayout>

#include <osgDB/ReadFile>

RightWindow::RightWindow() :
    QWidget()
{

}

RightWindow::~RightWindow()
{

}

void RightWindow::initialize() {
    // Load models
    osg::Node *ashModel = osgDB::readNodeFile("/home/afalendy/Models/ASH_Model_STL/SAFFiR_Simple.stl");
    OsgWidget *robot3DView = new OsgWidget();
    robot3DView->initialize();
    robot3DView->setSceneData(ashModel);

    // Create Widgets
    QLabel *labelLeft = new QLabel("Mode-specific Telemetry");
    labelLeft->setAlignment(Qt::AlignCenter);

    QLabel *labelRight = new QLabel("Modeul-specific Telemetry");
    labelRight->setAlignment(Qt::AlignCenter);

    // Perform layout
    QGridLayout *layout = new QGridLayout();
    layout->setSpacing(0);
    layout->setMargin(2);
    layout->setColumnStretch(0,3);
    layout->setColumnStretch(1,2);
    layout->setColumnStretch(2,2);
    layout->addWidget( labelLeft,   0, 0 );
    layout->addWidget( robot3DView, 0, 1 );
    layout->addWidget( labelRight,  0, 2 );
    setLayout( layout );
}

