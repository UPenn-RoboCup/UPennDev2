/*
  Swig Interface which maps lcm API into a Lua Module
*/

%module lualcm

%{
#include <lcm/lcm.h>
%}


// Emit Functions
%include <lcm/lcm.h>
