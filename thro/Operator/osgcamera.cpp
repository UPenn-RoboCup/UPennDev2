#include "osgcamera.h"

OsgCamera::OsgCamera()
{
    osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->windowDecoration = false;
    traits->supportsResize = true;
    traits->x = 0;
    traits->y = 0;
    traits->width = 100;
    traits->height = 100;
    traits->doubleBuffer = true;
    traits->alpha = ds->getMinimumNumAlphaBits();
    traits->stencil = ds->getMinimumNumStencilBits();
    traits->sampleBuffers = ds->getMultiSamples();
    traits->samples = ds->getNumMultiSamples();

    _camera = new osg::Camera;
    _camera->setGraphicsContext( new osgQt::GraphicsWindowQt(traits.get()) );

    _camera->setClearColor( osg::Vec4(0.1, 0.2, 0.3, 1.0) );
    _camera->setViewport( new osg::Viewport(0, 0, traits->width, traits->height) );
    _camera->setProjectionMatrixAsPerspective(
        90.0f, static_cast<double>(traits->width)/static_cast<double>(traits->height), 0.1f, 100000.0f );
}

OsgCamera::~OsgCamera()
{

}

osg::Camera* OsgCamera::get()
{
    return _camera;
}
