#include "osgwidget.h"

OsgWidget::OsgWidget(osg::Node *node, osgViewer::ViewerBase::ThreadingModel threadingModel)
{
    setThreadingModel(threadingModel);
    setBaseSize(100,100);
    setMinimumSize(100,100);

    addEventHandler( new osgViewer::StatsHandler );
    setCameraManipulator( new osgGA::TrackballManipulator );

    OsgCamera *camera = new OsgCamera;
    setCamera(camera->get());

    osgQt::GraphicsWindowQt* gw = dynamic_cast<osgQt::GraphicsWindowQt*>( camera->get()->getGraphicsContext() );

    setSceneData( node );

    QVBoxLayout *layout = new QVBoxLayout();
    layout->addWidget(gw->getGLWidget());
    setLayout(layout);

    connect( &_timer, SIGNAL(timeout()), this, SLOT(update()) );
    _timer.start( 10 );
}
