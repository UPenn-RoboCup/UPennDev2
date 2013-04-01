#include "leftwindow.h"
#include "ui_leftwindow.h"

#include "osgwidget.h"
#include <QLabel>

LeftWindow::LeftWindow() :
    QWidget()
{
    osg::Node *glider = osgDB::readNodeFile("glider.osgt");
    osg::Node *cow = osgDB::readNodeFile("/home/afalendy/Downloads/OpenSceneGraph-Data-3.0.0/cow.osgt");
    osg::Node *truck = osgDB::readNodeFile("/home/afalendy/Downloads/OpenSceneGraph-Data-3.0.0/dumptruck.osgt");
    OsgWidget *main3DView = new OsgWidget(glider);
    OsgWidget *model3DView = new OsgWidget(cow);
    OsgWidget *godsEye3DView = new OsgWidget(truck);

    QLabel *label = new QLabel("Model information and telemetry");
    label->setAlignment(Qt::AlignCenter);

    QGridLayout *layout = new QGridLayout();
    layout->setSpacing(0);
    layout->setMargin(2);
    layout->addWidget( model3DView,   0, 0, 2, 1);
    layout->addWidget( godsEye3DView, 2, 0, 2, 1);
    layout->addWidget( main3DView,    0, 1, 4, 2 );
    layout->addWidget( label,         4, 0, 1, 3 );
    setLayout( layout );
}

LeftWindow::~LeftWindow()
{

}

void LeftWindow::keyPressEvent(QKeyEvent *key)
{
    if(key->key() == Qt::Key_Escape)
        QApplication::exit();
}
