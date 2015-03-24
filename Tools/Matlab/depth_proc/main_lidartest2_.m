clear;
%close all

% Starting timestamp
load('~/Data/mesh_logs/Unpacked/03.11.2015.15.25.59l/20dar04d.mat');
    
n_scanlines = metal.dims(1);
n_returns = metal.dims(2);
s_angles = metal.a;
s_pitch = metal.pitch;
s_roll = metal.roll;
s_pose = metal.pose;

% the = 4.5*pi/180;
% ct = cos(the); st = sin(the);
 param{1} =  eye(3); % [ct 0 -st; 0 1 0; st 0 ct]; % orientation 
 param{2} = zeros(3,1);% translation

metal.flag = 1;

%tic,
[ Planes ] = detectPlaneInstances_lidar( meshRaw, param, 3, metal);
%toc
