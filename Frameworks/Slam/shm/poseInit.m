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