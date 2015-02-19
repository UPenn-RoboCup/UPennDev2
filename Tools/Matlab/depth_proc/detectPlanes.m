function  [Planes, metadata] = detectPlanes(data, meta, ui)
 

if ~isempty(meta) && isfield(meta,'tr')
    % transformation to the global coordinate 
    T = reshape(meta.tr,4,4)';  % temporary offset eye-measured -> should be estimated properly
    roll = 5.9*pi/180; cr = cos(roll); sr = sin(roll);
    pitch = 4*pi/180; cp = cos(pitch); sp = sin(pitch);
    Rot = [cr 0 sr; 0 1 0; -sr 0 cr]*T(1:3,1:3)*[1 0 0; 0 cp sp; 0 -sp cp]; % orientation 
    % make a function like compensateMountPose() 
    tr = T(1:3,4) + [-0.4; 0; 0];     % translation
else
    Rot = eye(3);
    tr = zeros(3,1);
end

if strcmp(char(meta.name),'depth')
    [ Planes, nPlanes, PlaneOfInterest ] = detectPlaneInstances_kinect_v3(data,Rot,tr,ui);
    metadata = struct('PlaneOfInterest',PlaneOfInterest,'numPlanes',nPlanes);
    elseif strcmp(char(meta.name),'lidar') % ?? 
   % Planes = detectPlaneInstances_lidar(data,param,3);
end

end