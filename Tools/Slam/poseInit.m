
% (c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
% ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
% University of Pennsylvania

function poseInit
global POSE

%if isempty(POSE) || ~isfield(POSE,'initialized') ||(POSE.initialized ~= 1)

  POSE.pose = [];
  POSE.data.x       = 0;
  POSE.data.y       = 0;
  POSE.data.z       = 0;
  POSE.data.roll    = 0;
  POSE.data.pitch   = 0;
  POSE.data.yaw     = 0;
  
  POSE.initialized  = 1;
  disp('Pose initialized');
%end