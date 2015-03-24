clear;
%close all

% Starting timestamp
load('/home/leebhoram/Data/mesh_log/Unpacked/03.11.2015.15.24.14l/1dar04d.mat');
    
n_scanlines = metadata.dims(1);
n_returns = metadata.dims(2);
s_angles = metadata.a;
s_pitch = metadata.pitch;
s_roll = metadata.roll;
s_pose = metadata.pose;

the = 4.5*pi/180;
ct = cos(the); st = sin(the);
param{1} = [ct 0 -st; 0 1 0; st 0 ct]; % eye(3); % orientation 
param{2} = zeros(3,1);% translation
metadata.flag = 1;

tic,
[ Planes ] = detectPlaneInstances_lidar( raw, param, 3, metadata );
toc
