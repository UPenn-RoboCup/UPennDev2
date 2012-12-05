function loadConfig()
global POSE LIDAR0


%pose
POSE.xInit      = 0;
POSE.yInit      = 0;
POSE.zInit      = 0;
POSE.rollInit   = 0;
POSE.pitchInit  = 0;
POSE.yawInit    = 0;


%lidar0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
LIDAR0.resd    = 0.25;
%LIDAR0.resd    = 0.5;
LIDAR0.res     = LIDAR0.resd/180*pi; 
LIDAR0.nRays   = 1080;%1081;
LIDAR0.angles  = ((0:LIDAR0.resd:(LIDAR0.nRays-1)*LIDAR0.resd)-135)'*pi/180;
LIDAR0.cosines = cos(LIDAR0.angles);
LIDAR0.sines   = sin(LIDAR0.angles);
LIDAR0.offsetx = 0.137;
LIDAR0.offsety = 0;
LIDAR0.offsetz = 0.54;  %from the body origin (not floor)

LIDAR0.mask    = ones(size(LIDAR0.angles));
%{
LIDAR0.mask(10:40)    = 0;
LIDAR0.mask(80:120)   = 0;
LIDAR0.mask(970:1010) = 0;
LIDAR0.mask(1050:end) = 0;
%}
% Be very conservative to ensure no interpolated antenna obstacles
LIDAR0.mask(1:190) = 0;
LIDAR0.mask(end-189:end) = 0;
LIDAR0.present = 1;



%lidar1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%servo1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%kinect
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%motors and wheels
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

