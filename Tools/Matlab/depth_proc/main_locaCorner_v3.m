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

foldername = '/home/leebhoram/Data/LOGS_SC2/Unpacked/';
datestamp = '03.12.2015.13.19.00'; % Testbed: walls (near valve)

foldername = '/home/leebhoram/Data/LOGS_Lab_0325_3/Unpacked/';
datestamp = '03.25.2015.16.40.27'; % Testbed: walls (near valve)
% datestamp = '03.12.2015.13.23.19'; % Testbed: walls (near valve)

 foldername = '~/Data/LOGS_Lab_0327_2/Unpacked/';
 datestamp = [];
%   foldername = '/home/leebhoram/Data/LOGS_Lab_0325_2/Unpacked/';
%   datestamp = '03.25.2015.16.36.47';
%foldername = '/home/leebhoram/Data/corner/Unpacked/';
%datestamp = '03.19.2015.17.53.03'; % Testbed: walls (near valve)


% vidObj = VideoWriter('localCorner_0326.avi');
% set(vidObj,'FrameRate',10);
% open(vidObj);

[ fileSequence] = getMatFilesFromFolder( strcat(foldername,datestamp));
 
figure(13), 
figPos = get(gcf,'Position');
set(gcf,'Position', [figPos(1:2) 1024 420]);
for ilog=83:length(fileSequence)
    ilog
    metad = [];
    load(fileSequence{ilog}); 
   % sprintf('%0.4f',metad.t)
   
    %tic,
    if 1
        D = depthRaw'-20;
        D(D>4000) = 0;
        D(D<400) = 0;
        %load MASK2.mat;
        %D = D.*double(bw);
        [pcx, pcy, pcz, r, g ,b] = depthToCloud(D, rgb_img);
        figure(13), subplot(1,2,1), hold off;
        showPointCloud([-pcx pcy pcz]*0.001, [r' g' b']/255,'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down'); hold on;
        set(gca,'XTick',[],'YTick',[],'ZTick',[]);
        set(gca,'Color',0.95*ones(1,3));
        set(gca,'XColor',0.95*ones(1,3));
        set(gca,'YColor',0.95*ones(1,3));
        set(gca,'ZColor',0.95*ones(1,3));
        axis([-1 1 -1 1 0 2]);
        view(0,-85);
    end
  
 
    uisetting; % See uisetting.m       size(D)
    % ui.undistortDepth = 1;
     
    ui.taskMode = 11 ;
    ui.figures(3) = 1;
    % average 
    [res, meta] = detectPlanes6(depthRaw, metad, ui);   
    pose = localizeCorner_v3(res,metad);
    
   % toc,
    if 0% numel(res) > 0
        d1 = res{1}.Normal'*res{1}.Center
        if  numel(res) > 1
            d2= res{2}.Normal'*res{2}.Center
        end
    end
    

    pause(0.01);  
    
%     M = getframe(gcf);   
%     writeVideo(vidObj,M);
%    
end 

% close(vidObj);


