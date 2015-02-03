function Planes = detectPlanes(data, meta, ui)
 
Planes = [];

% transformation to the global coordinate 
T = reshape(meta.tr,4,4)';  % temporary offset eye-measured -> should be estimated properly
roll = 5.9*pi/180; cr = cos(roll); sr = sin(roll);
pitch = 4*pi/180; cp = cos(pitch); sp = sin(pitch);
param{1} = [cr 0 sr; 0 1 0; -sr 0 cr]*T(1:3,1:3)*[1 0 0; 0 cp sp; 0 -sp cp]; % orientation 
param{2} = T(1:3,4) + [-0.4; 0; 0];     % translation

% any human input? 
param{3} = struct('mode',[],'data',[]); % reserved for human interaction

if strcmp(char(meta.name),'depth')
    figure(1), hold off;
    imagesc(data'); axis equal; colormap('gray');
     
    if ui.taskMode == 1     % ground detection         
    elseif ui.taskMode == 2 % wall detection
    elseif ui.taskMode == 3 % table detection
    elseif ui.taskMode == 4 % step-able planes detection
    else
        Planes = detectPlaneInstances_kinect_v2(data,param,3);
    end
elseif strcmp(char(meta.name),'lidar') % ?? 
   % Planes = detectPlaneInstances_lidar(data,param,3);
end

end