/*!
 This class represents an OpenSceneGraph view of
 a point cloud.

 @author jdavis
 @date 02/21/2013

 TODO build camera and camera manipulation for Point Cloud views
 */

#ifndef OSGCLOUDWIDGET_H
#define OSGCLOUDWIDGET_H

#include "osgwidget.h"

class OsgCloudWidget : public OsgWidget
{
public:
    OsgCloudWidget(osg::Node *node, osgViewer::ViewerBase::ThreadingModel threadingModel=osgViewer::Viewer::SingleThreaded);
};

#endif // OSGCLOUDWIDGET_H
