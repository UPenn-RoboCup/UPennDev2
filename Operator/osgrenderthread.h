#ifndef OSGRENDERTHREAD_H
#define OSGRENDERTHREAD_H

#include <QThread>

#include <osgViewer/Viewer>

class OsgRenderThread : public QThread
{
public:
    OsgRenderThread(QObject *parent);

    virtual ~OsgRenderThread();

    void setViewer(osgViewer::Viewer* viewer);

protected:
    virtual void run();

    osgViewer::Viewer* m_ViewerPtr;

};

#endif // OSGRENDERTHREAD_H
