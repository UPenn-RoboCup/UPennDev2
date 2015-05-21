function  [Planes, metadata] = detectPlanes7(data, meta, ui)
% v7: rough terrain (cinder blocks) 

if ~isempty(meta)   
     if isfield(meta,'tr')    
        [Rot, tr] = TransKinectToBody(meta);
    elseif isfield(meta,'tfL16')
       [Rot, tr] = TransKinectToBody_dale(meta);
    end
else
    Rot = eye(3);
    tr = zeros(3,1);
    meta.name = 'depth'
  
end

if ui.undistortDepth == 1
    data = undistort_depth(data); 
end

% if strcmp(char(meta.name),'depth')
[ Planes, nPlanes, PlaneOfInterest ] = detectPlaneInstances_kinect_v7(data,Rot,tr,ui);
metadata = struct('PlaneOfInterest',PlaneOfInterest,'numPlanes',nPlanes);

% elseif strcmp(char(meta.name),'lidar') % handles lidar here?? 
% Planes = detectPlaneInstances_lidar(data,param,3);
% end

end