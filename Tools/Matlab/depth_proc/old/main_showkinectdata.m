close all;
clear all;

DEPTH_W = 512;
DEPTH_H = 424;
DEPTH_MAX = 4000;%8000;
DEPTH_MIN = 200;
IMCX = DEPTH_W/2;
IMCY = DEPTH_H/2;

fx = 391.1;
fy = 463.1;

% Input: depth image, (depth)camera intrinsic parameters 
% Output: 
% (1) surface normals, 
% (2) some boundary points 
% All in camera centered coordinate

% parameters 
normalComp_param = [5 5]; % 1: (odd nember)^2+1 <- window sizw,  2: increment  
thre_svalue = 0.03; % The smaller it is, the flatter the plane fit is 
thre_clusterSize = 50; % number of clusters
thre_memberSize = 30; % number of connected members (in the image domain)
ms_resol = 0.52;         % mean shift resolution
ms_weights = [0.1 1];   % mean shift weights (1:image distance, 2:angular distance in normal space) 


RGB_W = 1920;
RGB_H = 1080;

Tb = [0 0 1; -1 0 0; 0 -1 0];

Yind = repmat(1:DEPTH_W,1,DEPTH_H); % index in 1D array 
Xind = kron(1:DEPTH_H,ones(1,DEPTH_W)); 

Xind_c = (reshape(Xind,DEPTH_W,DEPTH_H)-IMCX)/IMCX; % index in 2D array representation
Yind_c = (reshape(Yind,DEPTH_W,DEPTH_H)-IMCY)/IMCY; 

% foldername = 'Data';
foldername = 'kinect2_on_robot';
% datestamp = '01.23.2015.14.48.29';
 datestamp = '01.09.2015.16.42.40';
% -- datestamp = '12.04.2014.09.24.33';
% datestamp = '12.04.2014.09.25.21'; % kitchen, fast moving 
% datestamp = '12.04.2014.09.29.52'; % backyard, mostly sky  
% datestamp = '12.04.2014.09.30.05'; % backyard, plastic cloth
% datestamp = '12.04.2014.09.30.15'; % % backyard, gounrd + plastic cloth
% datestamp = '12.04.2014.09.30.29'; % backyard,
% datestamp = '12.04.2014.09.30.46'; % backyard, stairs
% -- datestamp = '12.04.2014.09.30.56';
% -- datestamp = '12.04.2014.09.31.22';
% datestamp = '12.04.2014.09.31.54'; % backyard, stairs
% datestamp = '12.04.2014.09.32.04'; % backyard, stairs
f_depth = fopen(sprintf('%s/k2_depth_r_%s.log', foldername,datestamp));
f_rgb = fopen(sprintf('%s/k2_rgb_r_%s.log', foldername,datestamp));
fid = fopen(sprintf('%s/k2_rgb_m_%s.log',foldername,datestamp));
rgbMeta = fread(fid,Inf,'*uint8');
fclose(fid);
fid = fopen(sprintf('%s/k2_depth_m_%s.log', foldername,datestamp));
depthMeta = fread(fid,Inf,'*uint8');
depthMeta = msgpack('unpacker', depthMeta, 'uint8');
fclose(fid);
rgbMeta = msgpack('unpacker',rgbMeta,'uint8');

saveflag = 0;
visflag = 1;

ilog = 0;
prevm = [];
L2 = [];

while ~feof(f_depth)
    
    depthRaw = fread(f_depth, [DEPTH_W, DEPTH_H], '*single');
    if isempty(depthRaw)
        break;
    end
    
    ilog = ilog + 1;
    if 1
    metar = rgbMeta{ilog};
    rgbJPEG = fread(f_rgb, metar.rsz, '*uint8');
    rgb_img0 = djpeg(rgbJPEG);
    rgb_img(:,:,1) = rgb_img0(:,:,1);
    rgb_img(:,:,2) = rgb_img0(:,:,2);
    rgb_img(:,:,3) = rgb_img0(:,:,3);   
    
    figure(90), imshow(rgb_img);
    end
    
    figure(1), imagesc(depthRaw'); axis equal;
    
    metad = depthMeta{ilog};
    save(strcat('Unpacked/',datestamp,'/',sprintf('%04d.mat',ilog)),'depthRaw', 'rgb_img', 'metad', 'metar');
    
    pause(0.5);
   
end

fclose(f_depth);
fclose(f_rgb);


