function  [Planes, metadata] = detectPlanes6(data, meta, ui)
% v6: corner of walls 
persistent MASK 
DEPTH_MAX = 2500; %8000;
DEPTH_MIN = 400;

if isempty(MASK)
    load('MASK2.mat')
end

if ~isempty(meta) 
    if isfield(meta,'tr')    
        [Rot, tr] = TransKinectToBody(meta);
    elseif isfield(meta,'tfL16')
       [Rot, tr] = TransKinectToBody_dale(meta);
    end
else
    Rot = eye(3);
    tr = zeros(3,1);
    meta.name = 'depth';
  
end

if ui.undistortDepth == 1
    data = undistort_depth(data); 
end

% pre-processing of depth image
data = flip(double(data)',2);
data(data(:) <= DEPTH_MIN) = 0;
data(data(:) >= DEPTH_MAX) = 0;   
data = MASK.*data;
[ Planes, nPlanes, PlaneOfInterest ] = detectPlaneInstances_kinect_v6(data,Rot,tr,ui);
metadata = struct('PlaneOfInterest',PlaneOfInterest,'numPlanes',nPlanes);

% elseif strcmp(char(meta.name),'lidar') % handles lidar here?? 
% Planes = detectPlaneInstances_lidar(data,param,3);
% end

end