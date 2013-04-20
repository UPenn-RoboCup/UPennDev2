#include "osgrenderthread.h"

OsgRenderThread::OsgRenderThread(QObject *parent) :
    QThread(parent)
{
}

OsgRenderThread::~OsgRenderThread()
{
    if (m_ViewerPtr != 0)
    {
        m_ViewerPtr->setDone(true);
    }
    wait();
}

void OsgRenderThread::run()
{
    if (m_ViewerPtr)
    {
        m_ViewerPtr->run();
    }
}

void OsgRenderThread::setViewer(osgViewer::Viewer* viewer)
{
    m_ViewerPtr = viewer;
}
