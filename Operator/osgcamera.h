/*!
 This class represents a camera in openscenegraph.
 This will inherit from osg::Camera and provide hardware
 lens accessor/modifiers that translate to 3D camera paramters.

 @author jdavis
 @date 02/21/2013

 TODO encapsulate VLC

 */

#ifndef OSGCAMERA_H
#define OSGCAMERA_H

#include <osg/Camera>
#include <osg/DisplaySettings>
#include <osgQt/GraphicsWindowQt>

class OsgCamera
{
public:
    OsgCamera();
    ~OsgCamera();

    osg::Camera* get();

private:
    osg::ref_ptr<osg::Camera> _camera;
};

#endif // OSGCAMERA_H
