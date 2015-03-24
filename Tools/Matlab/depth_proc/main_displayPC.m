% Plane Instance Detection Example
% by Bhoram Lee
%
% Input: depth image, (depth)camera intrinsic parameters 
% Output: 
% (1) surface normals, 
% (2) some boundary points 
% All in camera centered coordinate
clear all;
close all;

% foldername = '/home/leebhoram/Data/LOGS_THOR/Unpacked/';
% datestamp = '02.24.2015.17.21.57'; % lab setting : door/wall/motion/terrain object


foldername = '/home/leebhoram/Data/corner/Unpacked/';
datestamp = '03.19.2015.17.49.44'; % lab corner
datestamp = '03.19.2015.17.53.03';

% foldername = '/home/leebhoram/Data/LOGS_SC2/Unpacked/';
% datestamp = '03.12.2015.09.44.27'; % 
% datestamp = '03.12.2015.09.45.54'; % surprise (shower/triangle)
% datestamp = '03.12.2015.09.50.40'; % surprise (shower/triangle)

% datestamp = '03.12.2015.09.51.56'; % surprise (string)
% datestamp = '03.12.2015.09.54.55'; % surprise (lever)

% datestamp = '03.12.2015.09.56.30'; % surprise (string)
% datestamp = '03.12.2015.09.58.39'; % surprise (lever)
% datestamp = '03.12.2015.10.06.05'; % surprise (lever)
% datestamp = '03.12.2015.13.19.00'; % walls (other setting area, near valve) #407
% datestamp = '03.12.2015.13.21.59'; % % hose
% datestamp = '03.12.2015.13.23.19'; % Testbed: walls (near valve)


% foldername = '/home/leebhoram/Data/LOGS_SC_15/Unpacked/';
% % datestamp = '03.11.2015.14.52.01'; % debris
% % datestamp = '03.11.2015.14.56.17'; 
% % datestamp = '03.11.2015.14.58.56'; % hose
% % datestamp = '03.11.2015.15.00.21'; % terrain

% foldername = '/home/leebhoram/Data/LOGS_SC_13-14/Unpacked/';
% % datestamp = '03.11.2015.13.46.52'; % terrain
% % datestamp = '03.11.2015.13.48.08';   % not very useful
% % datestamp = '03.11.2015.13.51.55';   % debris
% % datestamp = '03.11.2015.13.52.56'; % debris
% datestamp = '03.11.2015.13.54.53';  % debris
% datestamp = '03.11.2015.13.57.00';  % debris


% foldername = '/home/leebhoram/Data/LOGS_SC_09-11/Unpacked/';
% % datestamp = '03.11.2015.08.31.38'; % door
% % datestamp = '03.11.2015.08.33.08'; % debris
% % datestamp = '03.11.2015.08.34.22'; % debris
% % datestamp = '03.11.2015.10.20.09'; % drill/valve (wall) 
% % datestamp = '03.11.2015.10.21.21'; % near door (opened)
% % datestamp = '03.11.2015.10.22.40'; % door from outside


% foldername = '/home/leebhoram/Data/LOGS_SC_11-13/Unpacked/';
% % datestamp = '03.11.2015.11.56.19'; % door from outside
% % datestamp = '03.11.2015.11.57.52'; % debris/drill/valve/hose/terrain	
% % datestamp = '03.11.2015.12.01.43'; % terrain
% % datestamp = '03.11.2015.12.03.08'; % stairs
% % datestamp = '03.11.2015.12.55.15'; % polaris (outer look)
% % datestamp = '03.11.2015.12.56.42'; % polaris (includes seat view)
% % datestamp = '03.11.2015.12.58.34'; % polaris (includes seat view)
% % datestamp = '03.11.2015.13.02.20'; % ground/door/debris
% % datestamp = '03.11.2015.13.05.05'; % terrain/stairs
% % datestamp = '03.11.2015.13.08.43'; % terrain

% foldername = '/home/leebhoram/Data/Log_win/'; 
% % transpose depth for window loggings
% % datestamp = '03.11.2015.09.32.28'; % terrain   
% % datestamp = '03.11.2015.09.33.35'; % terrain 
% % datestamp = '03.11.2015.09.37.48'; % outside the setting area & stairs (depth only) 
% % datestamp = '03.11.2015.09.39.49'; % stairs (depth only) 

[ fileSequence] = getMatFilesFromFolder( strcat(foldername,datestamp));
 
ts = 0;
prevts = 0;
N = length(fileSequence);
for ilog=2:2:N
    disp(strcat(int2str(ilog),'/(',int2str(N),')'));
    rgb_img = [];
    load(fileSequence{ilog}); 
  
    % load the following 
    % depthRaw : should take transpose 
    % rgb_img : r-channel and b-channel might be flipped for 01.20.2015 .data
    % metar: meta data for rgb 
    % metad: meta data for depth 
    depthRaw = undistort_depth(depthRaw);
    D = depthRaw';
    D(D>4000) = 0;
    D(D<400) = 0;
    margin = 15;
    D(1:margin,:) = 0;     D((end-margin):end,:) = 0;
    D(:,1:margin) = 0;     D(:,(end-margin):end) = 0;
    D = medfilt2(D,[7 7]);
   
    figure(1), imagesc(flip(D,2)); set(gca,'CLim',[0 5000]); colorbar;
   
   % load(fileSequence{ilog-1}); 
    if ~isempty(rgb_img)        
        figure(2), imshow(imresize(flip(rgb_img,2),0.25)); 
        
        [pcx, pcy, pcz, r, g ,b] = depthToCloud(D, rgb_img);
        figure(12), 
        showPointCloud([-pcx pcy pcz], [r' g' b']/255,'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
        % showPointCloud([-pcx pcy pcz],'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
        
    end
    
    pause(0.1);
    
end 



