
% (c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
% ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
% University of Pennsylvania

function initMapProps
global MAPS POSE

MAPS.res        = 0.05;
%MAPS.res        = 0.1;
MAPS.invRes     = 1/MAPS.res;

MAPS.windowSize = 10;
MAPS.edgeProx   = 2;
MAPS.xmin       = POSE.xInit - MAPS.windowSize;
MAPS.ymin       = POSE.yInit - MAPS.windowSize;
MAPS.xmax       = POSE.xInit + MAPS.windowSize;
MAPS.ymax       = POSE.yInit + MAPS.windowSize;
MAPS.zmin       = 0;
MAPS.zmax       = 5;

MAPS.map.sizex  = (MAPS.xmax - MAPS.xmin) / MAPS.res + 1;
MAPS.map.sizey  = (MAPS.ymax - MAPS.ymin) / MAPS.res + 1;
