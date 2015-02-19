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

% foldername = '/home/thor/Desktop/UPennDev/Tools/Matlab/depth_proc/UnpackedData/'; % change the folder
foldername = '/home/leebhoram/matlab_workspace/DRC/Unpacked/'; % change the folder
% datestamp = '12.04.2014.09.30.46';

% datestamp = '01.07.2015.16.42.55';
 datestamp = '01.09.2015.16.42.40';
% datestamp = '01.20.2015.12.02.25';

[ fileSequence] = getMatFilesFromFolder( strcat(foldername,datestamp));

for ilog=2:length(fileSequence)
    metad = [];
    load(fileSequence{ilog}); 
    % load the following 
    % depthRaw : should take transpose 
    % rgb_img : r-channel and b-channel might be flipped for 01.20.2015 .data
    % metar: meta data for rgb 
    % metad: meta data for depth 
   
    Planes = 0;
    
    uisetting; % See uisetting.m       
%    tic,
    [res, meta] = depth_proc(depthRaw, metad, ui);
%    toc,
    
   
end 


