function  [Planes, metadata] = detectPlanes(data, meta, ui)
 

if ~isempty(meta) && isfield(meta,'tr')    
    [Rot, tr] = TransKinectToBody(meta);
    %T = reshape(meta.tr,4,4)';
    %Rot = T(1:3,1:3);
    %tr = T(1:3,4);
else
    Rot = eye(3);
    tr = zeros(3,1);
end

if ui.undistortDepth == 1
    
end

if strcmp(char(meta.name),'depth')
    [ Planes, nPlanes, PlaneOfInterest ] = detectPlaneInstances_kinect_v3(data,Rot,tr,ui);
    metadata = struct('PlaneOfInterest',PlaneOfInterest,'numPlanes',nPlanes);

   % elseif strcmp(char(meta.name),'lidar') % ?? 
   % Planes = detectPlaneInstances_lidar(data,param,3);
end

end