/*!
 This class represents an OpenSceneGraph view of a 3D model

 @author jdavis
 @date 02/21/2013

 TODO Add camera, camera manipulator specific for 3D model viewing

 */

#ifndef OSGMODELWIDGET_H
#define OSGMODELWIDGET_H

#include "osgwidget.h"

class OsgModelWidget : public OsgWidget
{
public:
    OsgModelWidget(osg::Node *node, osgViewer::ViewerBase::ThreadingModel threadingModel=osgViewer::Viewer::SingleThreaded);
};

#endif // OSGMODELWIDGET_H
