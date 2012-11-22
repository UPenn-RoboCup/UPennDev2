/*
  Swig Interface which maps lcm API into a Lua Module
*/

%module lualcm

%{
#include <lcm/lcm.h>
#include <lcm/lcm_coretypes.h>
%}

// Emit Types
%include <lcm/lcm_coretypes.h>

// Emit Functions
%include <lcm/lcm.h>
