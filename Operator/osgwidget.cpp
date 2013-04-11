#include "osgwidget.h"
#include "osgmousemanipulator.h"
#include <osgViewer/ViewerEventHandlers>

#include <QtGui/QGridLayout>

#include <osgDB/ReadFile>

#include <osgQt/GraphicsWindowQt>

OsgWidget::OsgWidget() :
    m_RenderThread(0)
{

}

OsgWidget::~OsgWidget()
{
    if(m_RenderThread != 0)
    {
        delete m_RenderThread;
        m_RenderThread = 0;
    }
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
    setCameraManipulator( new OsgMouseManipulator );

    osgQt::GraphicsWindowQt* gw = dynamic_cast<osgQt::GraphicsWindowQt*>( camera->get()->getGraphicsContext() );

    // Construct the layout
    QVBoxLayout *layout = new QVBoxLayout();
    layout->addWidget(gw->getGLWidget());
    setLayout(layout);

    m_RenderThread = new OsgRenderThread(this);
    m_RenderThread->setViewer(this);
    m_RenderThread->start();
}
