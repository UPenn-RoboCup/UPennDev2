#include "leftwindow.h"
#include "ui_leftwindow.h"
#include "osgwidget.h"

#include <QLabel>
#include <QPushButton>
#include <QGridLayout>

#include <osgDB/ReadFile>

LeftWindow::LeftWindow() :
    QWidget()
{

}

LeftWindow::~LeftWindow()
{

}

void LeftWindow::initialize()
{
    // Load models
    osg::Node *glider = osgDB::readNodeFile("../Operator/assets/test/spaceship.osgt");
    osg::Node *cow = osgDB::readNodeFile("../Operator/assets/test/cow.osgt");
    osg::Node *truck = osgDB::readNodeFile("../Operator/assets/test/dumptruck.osg");

    // Create and initialize widgets
    OsgWidget *main3DView = new OsgWidget();
    main3DView->initialize();
    main3DView->setSceneData(glider);
    OsgWidget *model3DView = new OsgWidget();
    model3DView->initialize();
    model3DView->setSceneData(cow);
    OsgWidget *godsEye3DView = new OsgWidget();
    godsEye3DView->initialize();
    godsEye3DView->setSceneData(truck);

    // Create dummy label to fill telemetry space
    QLabel *label = new QLabel("Model information and telemetry");
    label->setAlignment(Qt::AlignCenter);

    // Perform layout
    QGridLayout *layout = new QGridLayout();
    layout->setSpacing(0);
    layout->setMargin(2);
    layout->addWidget( model3DView,   0, 0, 2, 1);
    layout->addWidget( godsEye3DView, 2, 0, 2, 1);
    layout->addWidget( main3DView,    0, 1, 4, 2 );
    layout->addWidget( label,         4, 0, 1, 3 );
    setLayout( layout );
}
