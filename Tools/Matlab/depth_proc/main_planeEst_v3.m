% Plane Instance Detection Example
% by Bhoram Lee
%
% Input: depth image, (depth)camera intrinsic parameters 
% Output: 
% (1) surface normals, 
% (2) some boundary points 
% All in camera centered coordinate
clear;
close all;

foldername = '/home/thor/Desktop/UPennDev/Tools/Matlab/depth_proc/UnpackedData/'; % change the folder
% datestamp = '12.04.2014.09.30.46';
 datestamp = '01.07.2015.16.42.55';
% datestamp = '01.09.2015.16.42.40';
% datestamp = '01.20.2015.12.02.25';

[ fileSequence] = getMatFilesFromFolder( strcat(foldername,datestamp));

for ilog=1:length(fileSequence)
    metad = [];
    load(fileSequence{ilog}); 
    % load the following 
    % depthRaw : should take transpose 
    % rgb_img : r-channel and b-channel might be flipped for 01.20.2015 .data
    % metar: meta data for rgb 
    % metad: meta data for depth 
   
    Planes = 0;
    
    if ~isempty(metad) && isfield(metad,'tr')
        T = reshape(metad.tr,4,4)';
        % temporary offset eye-measured -> should be estimated properly
        roll = 5.9*pi/180; cr = cos(roll); sr = sin(roll);
        pitch = 4*pi/180; cp = cos(pitch); sp = sin(pitch);
        param{1} = [cr 0 sr; 0 1 0; -sr 0 cr]*T(1:3,1:3)*[1 0 0; 0 cp sp; 0 -sp cp]; % orientation 
        param{2} = T(1:3,4) + [-0.4; 0; 0];     % translation
    else
        param{1} = eye(3);
        param{2} = zeros(3,1);
    end
    param{3} = struct('mode',[],'data',[]); % reserved for human interaction
 
 %   tic,
    Planes = detectPlaneInstances_kinect_v2(depthRaw,param,3);
 %   toc,
    pause;
    
end


