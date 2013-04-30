function publishPose(pose)

persistent initialized

if isempty(initialized)
  initialized = 1;
  ipcAPIDefine('Robot0/Pose',MagicMotionTrajSerializer('getFormat'));
end

ipcAPIPublishVC('Robot0/Pose',MagicPoseSerializer('serialize',pose));


