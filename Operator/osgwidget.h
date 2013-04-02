/*!
 This class represents the OpenSceneGraph Widget.

 It encapsulates an OpenSceneGraph view.

 @author jdavis
 @date 02/21/2013

 TODO build widget, use OsgWidgetQT example

 */

#ifndef OSGWIDGET_H
#define OSGWIDGET_H

#include <QWidget>
#include "osgcamera.h"

#include <QtCore/QTimer>
#include <QtGui/QApplication>
#include <QtGui/QGridLayout>

#include <osgViewer/ViewerEventHandlers>

#include <osgGA/TrackballManipulator>

#include <osgDB/ReadFile>

#include <osgQt/GraphicsWindowQt>

#include <iostream>


class OsgWidget : public QWidget, public osgViewer::Viewer
{
    Q_OBJECT
public:
    explicit OsgWidget(osg::Node *node, osgViewer::ViewerBase::ThreadingModel threadingModel=osgViewer::Viewer::SingleThreaded);
    QWidget* addViewWidget( OsgCamera camera, osg::Node* scene );
    virtual void paintEvent( QPaintEvent* event ) { frame(); }

signals:

public slots:

protected:

    QTimer _timer;

private:
    OsgCamera _camera;


};

#endif // OSGWIDGET_H
