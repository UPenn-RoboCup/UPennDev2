#include "osgwidget.h"
#include <osgViewer/ViewerEventHandlers>

#include <QtGui/QGridLayout>

#include <osgGA/TrackballManipulator>

#include <osgDB/ReadFile>

#include <osgQt/GraphicsWindowQt>

OsgWidget::OsgWidget()
{

}

void OsgWidget::initialize()
{
    // Set up OpenSceneGraph
    setRunFrameScheme( osgViewer::ViewerBase::ON_DEMAND );
    setThreadingModel(osgViewer::ViewerBase::CullDrawThreadPerContext);

    setBaseSize(100,100);
    setMinimumSize(100,100);

    // Attach a camera
    OsgCamera *camera = new OsgCamera;
    setCamera(camera->get());

    // add the stats handler
    addEventHandler(new osgViewer::StatsHandler);
    setCameraManipulator( new osgGA::TrackballManipulator );

    osgQt::GraphicsWindowQt* gw = dynamic_cast<osgQt::GraphicsWindowQt*>( camera->get()->getGraphicsContext() );

    // Construct the layout
    QVBoxLayout *layout = new QVBoxLayout();
    layout->addWidget(gw->getGLWidget());
    setLayout(layout);

    connect( &_timer, SIGNAL(timeout()), this, SLOT(drawFrame()) );
    _timer.start( 0 );
}

void OsgWidget::drawFrame()
{
    frame();
}
