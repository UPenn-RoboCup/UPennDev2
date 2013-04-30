/*
  Swig interface which maps the OpenRAVE C API into a Lua module
*/

%module openrave

%{
#include <rave/plugin.h>
#include <rave/rave.h>
#include <openrave-core.h>
		
#include <openrave/collisionchecker.h>
#include <openrave/config.h>
#include <openrave/controller.h>
#include <openrave/environment.h>
#include <openrave/geometry.h>
#include <openrave/iksolver.h>
#include <openrave/interface.h>
#include <openrave/interfacehashes.h>
#include <openrave/kinbody.h>
#include <openrave/mathextra.h>
#include <openrave/module.h>
#include <openrave/openrave.h>
#include <openrave/physicsengine.h>
#include <openrave/planner.h>
#include <openrave/plannerparameters.h>
#include <openrave/planningutils.h>
#include <openrave/plugin.h>
#include <openrave/plugininfo.h>
#include <openrave/robot.h>
#include <openrave/sensor.h>
#include <openrave/sensorsystem.h>
#include <openrave/spacesampler.h>
#include <openrave/trajectory.h>
#include <openrave/utils.h>
#include <openrave/viewer.h>
#include <openrave/xmlreaders.h>
%}

%{
#include <string.h>
%}