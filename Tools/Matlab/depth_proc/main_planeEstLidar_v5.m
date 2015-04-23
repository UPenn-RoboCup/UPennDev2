% Plane Instance Detection Example
% by Bhoram Lee
%
% Input: Lidar scans 
% Output: 
% (1) surface normals, 
% (2) some boundary points 
% All in camera centered coordinate for now
clear all;
close all;

foldername = '/home/leebhoram/Data/Webots_log/Unpacked/';
%datestamp = '04.21.2015.15.07.32l'
 datestamp = '04.22.2015.15.59.32l';

% foldername = '/home/leebhoram/Data/mesh_logs/Unpacked/';
% datestamp = '03.11.2015.15.30.41l'; % Testbed: walls (near valve)
% datestamp = '03.11.2015.15.45.07l';

% foldername = '/home/leebhoram/Data/LOGS_Lab_0325_3/Unpacked/';
% datestamp = '03.25.2015.16.36.42l'; 
% datestamp = '03.25.2015.16.40.20l';

[ fileSequence] = getMatFilesFromFolder( strcat(foldername,datestamp));
 
ts = 0;
prevts = 0;
for ilog=4:length(fileSequence)
    ilog
    metal = [];
    load(fileSequence{ilog}); 
    metal.dims = metal.dim;   
    
    if 1   
  
        metal.flag = 1;
        [ Planes ] = detectPlaneInstances_lidar_v5b( meshRaw', 3, metal);
     
    end 
  
  pause(0.01);
end 



