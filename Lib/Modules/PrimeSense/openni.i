/*
  Swig interface which maps Webots C API into a Lua module
*/

%module libMonitor

%{
#include <MacOSX/XnPlatformMacOSX.h>
#include <XnCppWrapper.h>
#include <XnContext.h>
%}

