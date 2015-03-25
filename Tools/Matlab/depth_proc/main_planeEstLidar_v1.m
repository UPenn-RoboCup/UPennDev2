% Plane Instance Detection Example
% by Bhoram Lee
%
% Input: Lidar scans 
% Output: 
% (1) surface normals, 
% (2) some boundary points 
% All in camera centered coordinate
clear all;
close all;

% foldername = '/home/leebhoram/Data/mesh_logs/Unpacked/';
% datestamp = '03.11.2015.15.30.41l'; % Testbed: walls (near valve)
% datestamp = '03.11.2015.15.45.07l';

foldername = '/home/leebhoram/Data/LOGS_Lab_0324/Unpacked/';
datestamp = '03.25.2015.12.48.31l'; 

[ fileSequence] = getMatFilesFromFolder( strcat(foldername,datestamp));
 
ts = 0;
prevts = 0;
for ilog=1:length(fileSequence)
    ilog
    metal = [];
    load(fileSequence{ilog}); 
       
    if 1   
  
    metal.flag = 1;
    [ Planes ] = detectPlaneInstances_lidar_v1( meshRaw', 3, metal);
    % [res, meta] = detectPlanes5(depthRaw, metad, ui);
    
    ilog
     

    end
   % prevts = metad.t;   
  %  F(iLog) = getframe(gcf);    
end 



